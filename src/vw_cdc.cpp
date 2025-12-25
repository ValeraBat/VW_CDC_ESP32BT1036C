#include "vw_cdc.h"
#include "bt_webui.h"
#include <SPI.h>

// Флаг debug режима (определён в bt_webui.cpp)
extern bool g_debugMode;

// ---------------- BCD Conversion ----------------
// VW Radio expects BCD format: decimal 99 -> 0x99, not 0x63
static inline uint8_t toBCD(uint8_t val) {
    if (val > 99) val = 99;
    return ((val / 10) << 4) | (val % 10);
}

// ---------------- CDC Protocol Constants ----------------
#define CDC_PREFIX1 0x53
#define CDC_PREFIX2 0x2C

// ---------------- Internal State ----------------
static SPIClass *g_spi = &SPI;
static int g_sckPin = -1;
static int g_misoPin = -1; 
static int g_mosiPin = -1;
static int g_ssPin   = -1;
static int g_dataOutPin = -1;
static CdcButtonCallback g_btnCb = nullptr;

static CdcStatus g_status;
static uint8_t g_modeByte = 0x00;  // Байт 5: SCAN/MIX (vwcdpic: 0x00=norm, 0x04=MIX, 0xD0=SCAN, 0xD4=both)
static uint8_t g_scanByte = 0xCF;  // Байт 6: SCAN (0xCF=off, 0x4F=on ?)
static uint32_t g_prevMs = 0;
static uint8_t g_playMinutes = 0;  // Начинаем с 00:00
static uint8_t g_playSeconds = 0;  // Начинаем с 00:00
static uint32_t g_lastBtTimeUpdate = 0;  // Когда последний раз получили время от BT
static uint8_t g_discLoad = 0x2E;     // vwcdpic: StateInitPlay disc announce counter (0x2E=CD1)

// ---------------- Logging Helpers ----------------
static void cdc_log(const String &s) {
    btWebUI_log("[CDC] " + s);
}

static void cdc_log_nec(const String &s) {
    // Шлем в отдельный канал для RAW терминала
    btWebUI_broadcastCdcRaw(String("[CDC_NEC] ") + s);
}

// ---------------- RAW PULSE SNIFFER (Кольцевой буфер) ----------------
// Позволяет видеть "сырые" тайминги в логе, даже если декодер не узнал кнопку
#define RAW_BUF_SIZE 64
static volatile uint16_t g_rawBuf[RAW_BUF_SIZE];
static volatile uint8_t  g_rawHead = 0;
static uint8_t           g_rawTail = 0;

static void IRAM_ATTR log_raw_pulse(uint32_t dur) {
    if (dur > 60000) dur = 60000;
    g_rawBuf[g_rawHead] = (uint16_t)dur;
    g_rawHead = (g_rawHead + 1) % RAW_BUF_SIZE;
}

// Отправка накопленных RAW данных в WebUI (вызывается в loop)
static void processRawLog() {
    if (g_rawHead == g_rawTail) return; // Пусто

    String s = "RAW:";
    int count = 0;
    while (g_rawHead != g_rawTail) {
        uint16_t d = g_rawBuf[g_rawTail];
        g_rawTail = (g_rawTail + 1) % RAW_BUF_SIZE;
        s += " " + String(d);
        count++;
        if (count >= 20) {  // Увеличили до 20 чтобы видеть больше
            cdc_log_nec(s);
            s = "RAW:";
            count = 0;
        }
    }
    if (count > 0) cdc_log_nec(s);  // Отправим остаток
}

// ---------------- VW CDC DataOut Decoder (vwcdpic protocol) ----------------
// Protocol: Measures LOW pulse duration to determine bit values
// Packet format: 32 bits = 4 bytes: [0x53] [0x2C] [cmdcode] [~cmdcode]
// Timing (with ESP32 microsecond precision):
//   Start bit:  LOW > 3200µs (~4.57ms)
//   Bit '1':    LOW > 1248µs (~1.77ms)
//   Bit '0':    LOW < 1248µs (~650µs)
//   Noise filter: LOW > 256µs minimum

// Thresholds (in microseconds, matching vwcdpic with 32x prescaler)
static const uint32_t VW_START_THRESHOLD = 3200;  // vwcdpic: 100 * 32µs
static const uint32_t VW_HIGH_THRESHOLD  = 1248;  // vwcdpic: 39 * 32µs (bit 1)
static const uint32_t VW_LOW_THRESHOLD   = 256;   // vwcdpic: 8 * 32µs (noise)
static const uint8_t  VW_PKTSIZE         = 32;    // 32 bits per packet

// Ring buffer for captured packets (24 bytes = 6 packets of 4 bytes)
#define VW_CAPBUFFER_SIZE 24
volatile uint8_t vw_capBuffer[VW_CAPBUFFER_SIZE];
volatile uint8_t vw_capPtr = 0;        // Write pointer
volatile uint8_t vw_scanPtr = 0;       // Read pointer for parsing
volatile bool vw_capBusy = false;      // Capturing in progress

// Current bit capture state
volatile uint8_t vw_capBit = 0;        // Bits remaining in current byte (-8 to 0)
volatile uint8_t vw_capBitPacket = 0;  // Bits remaining in packet (-32 to 0)
volatile uint8_t vw_currentByte = 0;   // Byte being assembled

// Timing measurement
volatile uint32_t vw_lastFallingEdge = 0;  // Timestamp of last falling edge
volatile bool vw_measuringLow = false;     // True if currently measuring LOW pulse

volatile uint32_t vw_isr_counter = 0;  // ISR invocation counter
volatile uint32_t vw_falling_edges = 0; // Falling edge counter (for debug)
volatile uint32_t vw_rising_edges = 0;  // Rising edge counter (for debug)

// ISR: Triggered on BOTH edges (CHANGE mode)
static void IRAM_ATTR vw_dataout_isr() {
    vw_isr_counter++;
    
    if (g_dataOutPin < 0) return;
    
    uint32_t now = micros();
    bool level = digitalRead(g_dataOutPin);
    
    if (!level) {
        // FALLING EDGE: Start measuring LOW pulse
        vw_falling_edges++;
        vw_lastFallingEdge = now;
        vw_measuringLow = true;
    } else {
        // RISING EDGE: Capture LOW pulse duration
        vw_rising_edges++;
        if (!vw_measuringLow) return; // Spurious interrupt
        
        uint32_t lowDuration = now - vw_lastFallingEdge;
        vw_measuringLow = false;
        
        // Log ALL pulses including noise (for level shifter debugging)
        log_raw_pulse(lowDuration);
        
        // Also log if very short (might indicate inverted signal)
        if (lowDuration < 100) {
            // Extremely short - might be noise or inverted signal
            return;
        }
        
        // Filter noise (too short)
        if (lowDuration < VW_LOW_THRESHOLD) {
            return; // Ignore
        }
        
        // Check for START bit (begins new packet)
        if (lowDuration >= VW_START_THRESHOLD) {
            vw_capBusy = true;
            vw_capBitPacket = VW_PKTSIZE;  // Reset to 32 bits
            vw_capBit = 8;                 // Start fresh byte
            vw_currentByte = 0;
            // NOTE: НЕ вызываем cdc_log_nec() здесь - String запрещён в ISR!
            return; // Don't store start bit itself
        }
        
        // Only capture data if we're in a packet
        if (!vw_capBusy || vw_capBitPacket == 0) {
            return;
        }
        
        // Determine bit value from LOW duration
        uint8_t bitValue = (lowDuration >= VW_HIGH_THRESHOLD) ? 1 : 0;
        
        // Shift bit into current byte - vwcdpic uses rlf (rotate left)
        // New bit goes to LSB, byte shifts left
        vw_currentByte <<= 1;
        if (bitValue) {
            vw_currentByte |= 0x01;
        }
        
        vw_capBit--;
        vw_capBitPacket--;
        
        // Byte complete?
        if (vw_capBit == 0) {
            // Store byte in ring buffer
            vw_capBuffer[vw_capPtr] = vw_currentByte;
            vw_capPtr = (vw_capPtr + 1) % VW_CAPBUFFER_SIZE;
            
            // Prepare for next byte
            vw_capBit = 8;
            vw_currentByte = 0;
        }
        
        // Packet complete?
        if (vw_capBitPacket == 0) {
            vw_capBusy = false;
            // NOTE: Логирование перенесено в cdc_pollNec() - String запрещён в ISR!
        }
    }
}

// ---------------- VW Packet Parser ----------------
// Scans ring buffer for valid packets: [0x53] [0x2C] [cmdcode] [~cmdcode]
// Validation: byte1=0x53, byte2=0x2C, byte3+byte4=0xFF, byte3 multiple of 4

static void vw_scanCommandBytes() {
    // Search for 0x53 0x2C packet start
    while (vw_scanPtr != vw_capPtr) {
        uint8_t byte1 = vw_capBuffer[vw_scanPtr];
        
        if (byte1 != 0x53) {
            vw_scanPtr = (vw_scanPtr + 1) % VW_CAPBUFFER_SIZE;
            continue;
        }
        
        // Check if we have at least 4 bytes available
        uint8_t available = (vw_capPtr >= vw_scanPtr) ? 
                           (vw_capPtr - vw_scanPtr) : 
                           (VW_CAPBUFFER_SIZE - vw_scanPtr + vw_capPtr);
        
        if (available < 4) {
            return; // Wait for more data
        }
        
        // Read full packet
        uint8_t byte2 = vw_capBuffer[(vw_scanPtr + 1) % VW_CAPBUFFER_SIZE];
        uint8_t byte3 = vw_capBuffer[(vw_scanPtr + 2) % VW_CAPBUFFER_SIZE];
        uint8_t byte4 = vw_capBuffer[(vw_scanPtr + 3) % VW_CAPBUFFER_SIZE];
        
        // Validate packet
        if (byte2 != 0x2C) {
            vw_scanPtr = (vw_scanPtr + 1) % VW_CAPBUFFER_SIZE;
            continue;
        }
        
        // Check byte3 + byte4 = 0xFF
        if ((uint8_t)(byte3 + byte4) != 0xFF) {
            cdc_log_nec("VW: Invalid checksum: " + String(byte3, HEX) + " + " + String(byte4, HEX));
            vw_scanPtr = (vw_scanPtr + 1) % VW_CAPBUFFER_SIZE;
            continue;
        }
        
        // Check byte3 is multiple of 4 (vwcdpic requirement)
        if ((byte3 & 0x03) != 0) {
            cdc_log_nec("VW: cmdcode not multiple of 4: " + String(byte3, HEX));
            vw_scanPtr = (vw_scanPtr + 1) % VW_CAPBUFFER_SIZE;
            continue;
        }
        
        // Valid packet found!
        uint8_t cmdcode = byte3;
        // Логируем команды только в debug режиме
        if (g_debugMode) {
            cdc_log_nec("VW CMD: 0x" + String(cmdcode, HEX) + " (53 2C " + 
                        String(byte3, HEX) + " " + String(byte4, HEX) + ")");
        }
        
        // Map to button - РЕАЛЬНЫЕ КОДЫ от RNS-MFD
        CdcButton btn = CdcButton::UNKNOWN;
        switch (cmdcode) {
            // Треки
            case 0xF8: btn = CdcButton::NEXT_TRACK; break;
            case 0x78: btn = CdcButton::PREV_TRACK; break;
            
            // CD кнопки (коды подтверждены RAW логами)
            case 0x0C: btn = CdcButton::DISC_1; break;
            case 0x8C: btn = CdcButton::DISC_2; break;
            case 0x4C: btn = CdcButton::DISC_3; break;
            case 0xCC: btn = CdcButton::DISC_4; break;
            case 0x2C: btn = CdcButton::DISC_5; break;  // Подтверждено
            case 0xAC: btn = CdcButton::DISC_6; break;  // Подтверждено
            
            // Функции
            case 0xA0: btn = CdcButton::SCAN_TOGGLE; break;
            case 0xE0: btn = CdcButton::RANDOM_TOGGLE; break;
            
            // Игнорируем служебные коды
            case 0x14: break;  // Repeat-код (игнор)
            case 0x38: break;  // CD confirm (игнор)
        }
        
        // Просто передаём кнопку дальше, без debounce. Логика будет в main.cpp
        if (g_btnCb && btn != CdcButton::UNKNOWN) {
            g_btnCb(btn);
        }
        
        // Advance scan pointer past this packet
        vw_scanPtr = (vw_scanPtr + 4) % VW_CAPBUFFER_SIZE;
    }
}

static void cdc_pollNec() {
    // Send raw logs
    processRawLog();

    // Scan ring buffer for valid VW packets
    vw_scanCommandBytes();
}

// ---------------- SPI Functions ----------------
// BCD to decimal for logging
static inline uint8_t fromBCD(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

static void cdc_sendSpiPacket(const uint8_t frame[8]) {
    g_spi->beginTransaction(SPISettings(62500, MSBFIRST, SPI_MODE1));
    for (int i = 0; i < 8; ++i) {
        g_spi->transfer(frame[i]);
        delayMicroseconds(874);
    }
    g_spi->endTransaction();
}


static void cdc_sendPackage(const uint8_t frame[8]) {
    // Логируем ВСЕ отправляемые пакеты для диагностики
    String hex = "SPI TX: ";
    for(int i=0; i<8; i++) {
        if (frame[i] < 0x10) hex += "0";
        hex += String(frame[i], HEX) + " ";
    }
    
    // Расшифровка пакета (track и время в BCD!)
    uint8_t cmd = frame[0];
    if (cmd == 0x34) {
        uint8_t disc = 0xBF - frame[1];
        uint8_t trackBCD = 0xFF - frame[2];
        uint8_t minBCD = 0xFF - frame[3];
        uint8_t secBCD = 0xFF - frame[4];
        // Конвертируем BCD обратно в десятичные для читаемого лога
        hex += "→ PLAY CD" + String(disc) + " T" + String(fromBCD(trackBCD)) + 
               " " + String(fromBCD(minBCD)) + ":" + String(fromBCD(secBCD));
    } else if (cmd == 0x74) {
        hex += "→ IDLE";
    }
    cdc_log(hex);
    
    g_spi->beginTransaction(SPISettings(62500, MSBFIRST, SPI_MODE1));
    for (int i = 0; i < 8; ++i) {
        g_spi->transfer(frame[i]);
        delayMicroseconds(874);
    }
    g_spi->endTransaction();
}

// ---------------- Init / Loop ----------------
void cdc_init(int sckPin, int misoPin, int mosiPin, int ssPin, int necPin, CdcButtonCallback cb) {
    g_sckPin = sckPin; g_misoPin = misoPin; g_mosiPin = mosiPin; g_ssPin = ssPin;
    g_btnCb = cb;
    g_status.disc = 1; g_status.track = 1; g_status.state = CdcPlayState::PLAYING;

    g_spi->begin(g_sckPin, g_misoPin, g_mosiPin, g_ssPin);
    cdc_log("SPI initialized: SCK=" + String(g_sckPin) + 
            " MISO=" + String(g_misoPin) + " MOSI=" + String(g_mosiPin));

    // NEC decoder на отдельном пине (не MISO!)
    g_dataOutPin = necPin;
    
    // Инициализируем VW протокол
    vw_capPtr = 0;
    vw_scanPtr = 0;
    vw_capBusy = false;
    vw_capBit = 8;
    vw_capBitPacket = 0;
    vw_currentByte = 0;
    vw_measuringLow = false;
    vw_lastFallingEdge = 0;
    vw_isr_counter = 0;
    
    if (g_dataOutPin >= 0) {
        // Внешняя схемотехника уже задаёт подтяжку, поэтому внутренний pull-up отключаем,
        // иначе образуется делитель и уровень висит на ~1.5 В.
        pinMode(g_dataOutPin, INPUT);

        // Test: Read initial pin state
        bool initialState = digitalRead(g_dataOutPin);
        cdc_log("VW DataOut pin " + String(g_dataOutPin) + " initial state: " + String(initialState));
        
        attachInterrupt(digitalPinToInterrupt(g_dataOutPin), vw_dataout_isr, CHANGE);
        cdc_log("VW DataOut ISR attached (CHANGE mode, with PULLUP)");
    }
    
    // НЕ отправляем инициализацию здесь!
    // Магнитола может быть еще не в режиме CDC.
    // Вместо этого будем отправлять последовательность IDLE→LOAD→IDLE→PLAY в cdc_loop()
    // Это активирует пункт CDC в меню магнитолы.
    cdc_log("=== CDC INIT: Will send init sequence (10s warmup) ===");
    g_prevMs = millis();
}

void cdc_loop() {
    // vwcdpic state machine: StateIdleThenPlay → StateInitPlay → StatePlayLeadIn → StatePlay
    static enum { ST_IDLE_THEN_PLAY, ST_INIT_PLAY, ST_PLAY_LEAD_IN, ST_PLAY } state = ST_IDLE_THEN_PLAY;
    // cppcheck-suppress variableScope  ; static needed for state machine persistence
    static int stateCounter = 0;  // BIDIcount equivalent (negative countdown)
    // cppcheck-suppress variableScope  ; static needed for state machine persistence
    static bool initStarted = false;
    
    // Debug: Log ISR counter every 5 seconds (only in debug mode)
    static uint32_t lastIsrLog = 0;
    uint32_t nowMs = millis();
    if (g_debugMode && (nowMs - lastIsrLog >= 5000)) {
        lastIsrLog = nowMs;
        cdc_log("VW ISR: total=" + String(vw_isr_counter) + 
                " fall=" + String(vw_falling_edges) + 
                " rise=" + String(vw_rising_edges) + 
                " | CapPtr:" + String(vw_capPtr) + " ScanPtr:" + String(vw_scanPtr));
    }
    
    processRawLog();
    cdc_pollNec();
    
    uint32_t now = millis();
    
    // Инкремент времени каждую секунду (только если НЕ получаем от BT)
    // Если BT присылает TRACKSTAT, используем его время, иначе считаем сами
    static uint32_t lastSecond = 0;
    bool btTimeActive = (g_lastBtTimeUpdate > 0) && ((now - g_lastBtTimeUpdate) < 3000);
    
    if (!btTimeActive && state == ST_PLAY && g_status.state == CdcPlayState::PLAYING && (now - lastSecond >= 1000)) {
        lastSecond = now;
        g_playSeconds++;
        if (g_playSeconds >= 60) {
            g_playSeconds = 0;
            g_playMinutes++;
            if (g_playMinutes >= 100) g_playMinutes = 0;
        }
    }
    
    if (now - g_prevMs >= 50) {  // 50ms = 20 packets/sec (vwcdpic timing)
        g_prevMs = now;
        
        uint8_t disc = g_status.disc; if(disc<1) disc=1; if(disc>6) disc=6;
        uint8_t track = g_status.track; if(track<1) track=1; if(track>99) track=99;
        
        if (!initStarted) {
            initStarted = true;
            cdc_log("=== CDC Init: StateIdleThenPlay (20 packets) ===");
            state = ST_IDLE_THEN_PLAY;
            stateCounter = -20;  // vwcdpic: BIDIcount = -20
        }
        
        // ====== STATE: IdleThenPlay (vwcdpic lines 2203-2212) ======
        if (state == ST_IDLE_THEN_PLAY) {
            // Send IDLE packet: 74 BE FE FF FF FF 8F 7C
            uint8_t idle[8] = {
                0x74, 
                (uint8_t)(0xBF - disc),
                (uint8_t)(0xFF - track),
                0xFF, 0xFF, 0xFF,  // vwcdpic uses 0xFF for mode in IDLE
                0x8F, 0x7C
            };
            
            if (stateCounter >= -5 || (stateCounter % 5) == 0) {  // Log first 5 and every 5th
                String hex = "[IdleThenPlay " + String(-stateCounter) + "/20] ";
                for(int i=0; i<8; i++) {
                    if (idle[i] < 0x10) hex += "0";
                    hex += String(idle[i], HEX) + " ";
                }
                cdc_log(hex);
            }
            
            cdc_sendSpiPacket(idle);
            
            stateCounter++;
            if (stateCounter >= 0) {  // incfsz BIDIcount, f → goto StateIdle (then call SetStateInitPlay)
                cdc_log("=== Transition: StateInitPlay (24 packets) ===");
                state = ST_INIT_PLAY;
                stateCounter = -24;  // vwcdpic: BIDIcount = -24
                g_discLoad = 0x2E;   // vwcdpic: discload = 0x2E (CD1 announce)
            }
        }
        
        // ====== STATE: InitPlay (vwcdpic lines 2226-2268) ======
        else if (state == ST_INIT_PLAY) {
            // Alternating packets: odd=announce CD info, even=normal display
            // btfss BIDIcount, 0 → goto StateInitPlayAnnounceCD (bit 0 clear = even counter)
            bool isAnnounce = ((stateCounter & 1) == 0);  // Even countdown = announce
            
            if (isAnnounce) {
                // StateInitPlayAnnounceCD: 34 2E XX XX XX B7 FF 3C
                // Sends CD info with discload cycling 0x2E→0x2D→...→0x29→0x2E
                uint8_t frame[8] = {
                    0x34,
                    g_discLoad,  // 0x29..0x2F = AUDIO CD Loaded
                    0xFF - 0x99, // 99 tracks
                    0xFF - 0x99, // 99 minutes  
                    0xFF - 0x59, // 59 seconds
                    0xB7,        // vwcdpic constant (B7, AC, CE, DA, C8 seen)
                    0xFF,
                    0x3C
                };
                
                if (stateCounter >= -5) {  // Log first few
                    String hex = "[InitPlay-Announce " + String(-stateCounter) + "/24] discload=";
                    hex += String(g_discLoad, HEX) + " → ";
                    for(int i=0; i<8; i++) {
                        if (frame[i] < 0x10) hex += "0";
                        hex += String(frame[i], HEX) + " ";
                    }
                    cdc_log(hex);
                }
                
                cdc_sendSpiPacket(frame);
                
                // Cycle discload: 0x29 → reached CD6? → 0x2E : decf discload
                if (g_discLoad == 0x29) {
                    g_discLoad = 0x2E;  // Loop back to CD1
                } else {
                    g_discLoad--;  // 0x2E→0x2D→0x2C→0x2B→0x2A→0x29
                }
            }
            else {
                // Normal packet: 34 BE FE FF FF FF EF 3C
                uint8_t frame[8] = {
                    0x34,
                    (uint8_t)(0xBF - disc),
                    (uint8_t)(0xFF - track),
                    0xFF, 0xFF, 0xFF,  // minute/second/mode all 0xFF during init
                    0xEF,  // vwcdpic init mute byte
                    0x3C
                };
                
                if (stateCounter >= -5) {
                    String hex = "[InitPlay-Normal " + String(-stateCounter) + "/24] ";
                    for(int i=0; i<8; i++) {
                        if (frame[i] < 0x10) hex += "0";
                        hex += String(frame[i], HEX) + " ";
                    }
                    cdc_log(hex);
                }
                
                cdc_sendSpiPacket(frame);
            }
            
            stateCounter++;
            if (stateCounter >= 0) {  // incfsz → goto SetStatePlayLeadIn
                cdc_log("=== Transition: StatePlayLeadIn (10 packets) ===");
                state = ST_PLAY_LEAD_IN;
                stateCounter = -10;
                // vwcdpic: sets time 0xFF here (already initialized)
            }
        }
        
        // ====== STATE: PlayLeadIn (vwcdpic lines 2278-2303) ======
        else if (state == ST_PLAY_LEAD_IN) {
            // Alternating announce/normal like InitPlay but different mute byte
            bool isAnnounce = ((stateCounter & 1) == 0);
            
            if (isAnnounce) {
                // StatePlayLeadInAnnounceCD: disc's lower nibble | 0x20
                uint8_t frame[8] = {
                    0x34,
                    (uint8_t)((disc & 0x0F) | 0x20),  // vwcdpic: andlw 0x0F; iorlw 0x20
                    0xFF - 0x99,
                    0xFF - 0x99,
                    0xFF - 0x59,
                    0xB7,
                    0xFF,
                    0x3C
                };
                
                cdc_sendSpiPacket(frame);
            }
            else {
                // Normal: 34 BE FE FF FF FF AE 3C
                uint8_t frame[8] = {
                    0x34,
                    (uint8_t)(0xBF - disc),
                    (uint8_t)(0xFF - track),
                    0xFF, 0xFF, 0xFF,
                    0xAE,  // PlayLeadIn mute byte
                    0x3C
                };
                
                cdc_sendSpiPacket(frame);
            }
            
            stateCounter++;
            if (stateCounter >= 0) {
                cdc_log("=== Transition: StatePlay (normal operation) ===");
                state = ST_PLAY;
                // Start time from 00:00 (vwcdpic increments 0xFF→0x00→0x01...)
            }
        }
        
        // ====== STATE: Play (vwcdpic lines 2329-2340) ======
        else if (state == ST_PLAY) {
            // StatePlay: 34 BE FE MM SS FB CF 3C (continuous)
            // Track и время в BCD формате!
            uint8_t trackBCD = toBCD(track);
            uint8_t minBCD = toBCD(g_playMinutes);
            uint8_t secBCD = toBCD(g_playSeconds);
            
            uint8_t frame[8] = {
                0x34,
                (uint8_t)(0xBF - disc),
                (uint8_t)(0xFF - trackBCD),
                (uint8_t)(0xFF - minBCD),
                (uint8_t)(0xFF - secBCD),
                g_modeByte,  // Байт 5: MIX (0xFF=off, 0x55=on)
                g_scanByte,  // Байт 6: SCAN (0xCF=off, 0x4F=on)
                0x3C
            };
            
            // Логируем [PLAY] только в debug режиме
            if (g_debugMode) {
                static int playCount = 0;
                playCount++;
                if (playCount <= 10 || playCount % 20 == 0) {  // Log first 10, then every second
                    String hex = "[PLAY] ";
                    for(int i=0; i<8; i++) {
                        if (frame[i] < 0x10) hex += "0";
                        hex += String(frame[i], HEX) + " ";
                    }
                    hex += "→ CD" + String(disc) + " T" + String(track) + " ";
                    // Время всегда показываем (начинаем с 00:00)
                    if (g_playMinutes < 10) hex += "0";
                    hex += String(g_playMinutes) + ":";
                    if (g_playSeconds < 10) hex += "0";
                    hex += String(g_playSeconds);
                    cdc_log(hex);
                }
            }
            
            cdc_sendSpiPacket(frame);
        }
    }
}

// Setters
void cdc_setDiscTrack(uint8_t d, uint8_t t) { 
    g_status.disc=d; 
    g_status.track=t; 
    g_playMinutes = 0;  // Сброс времени на 00:00 при смене трека
    g_playSeconds = 0;
}
void cdc_setPlayState(CdcPlayState s) { g_status.state=s; }

// Таблица из vwcdpic (без инверсий!):
// 0x00 = scan off, mix off (норма)
// 0x04 = scan off, mix on
// 0xD0 = scan on, mix off
// 0xD4 = scan on, mix on
static void updateModeBytes() {
    uint8_t oldMode = g_modeByte;
    
    if (g_status.scanOn && g_status.randomOn) {
        g_modeByte = 0xD4;
    } else if (g_status.scanOn) {
        g_modeByte = 0xD0;
    } else if (g_status.randomOn) {
        g_modeByte = 0x04;
    } else {
        g_modeByte = 0x00;
    }
    
    // Байт 6: НЕ трогаем! Всегда 0xCF
    g_scanByte = 0xCF;
    
    if (oldMode != g_modeByte) {
        cdc_log("ModeByte[5]: 0x" + String(oldMode, HEX) + " → 0x" + String(g_modeByte, HEX));
    }
}

void cdc_resetModeFF() {
    g_status.scanOn = false;
    g_status.randomOn = false;
    g_modeByte = 0xFF;
    cdc_log("ModeByte[5] reset to 0xFF");
}

void cdc_setPlayTime(uint8_t minutes, uint8_t seconds) {
    if (minutes > 99) minutes = 99;
    if (seconds > 59) seconds = 59;
    g_playMinutes = minutes;
    g_playSeconds = seconds;
    g_lastBtTimeUpdate = millis();  // Отмечаем что получили время от BT
}

void cdc_setRandom(bool o) { 
    g_status.randomOn = o; 
    updateModeBytes(); 
}

void cdc_setScan(bool o) { 
    g_status.scanOn = o; 
    updateModeBytes(); 
}
CdcStatus cdc_getStatus() { return g_status; }

void cdc_pause(bool pause) {
    if (g_dataOutPin < 0) return;

    if (pause) {
        detachInterrupt(digitalPinToInterrupt(g_dataOutPin));
        cdc_log("VW DataOut ISR detached for OTA");
    } else {
        attachInterrupt(digitalPinToInterrupt(g_dataOutPin), vw_dataout_isr, CHANGE);
        cdc_log("VW DataOut ISR re-attached after OTA");
    }
}