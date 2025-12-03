#include "bt1036_at.h"
#include "bt_webui.h"  // для btWebUI_log() и LogLevel
#include "vw_cdc.h"    // для cdc_setPlayTime()

static HardwareSerial *bt = nullptr;

// ---------- очередь команд ----------
static const size_t   CMD_QUEUE_SIZE    = 10;
static String         cmdQueue[CMD_QUEUE_SIZE];
static uint8_t        queueHead         = 0;
static uint8_t        queueTail         = 0;
static bool           cmdInProgress     = false;
static uint32_t       cmdTimestamp      = 0;
static const uint32_t CMD_TIMEOUT_MS    = 2000;

static String          rxLine;
static BTConnState     btState           = BTConnState::DISCONNECTED;
static BtDevStat       devStat{};

// Информация о текущем треке (из +TRACKSTAT и +TRACKINFO)
static TrackInfo       g_trackInfo       = {0, 0, "", "", "", false};

// Callback для смены состояния
static BtStateCallback stateCb           = nullptr;

// фоновый опрос статусов
static uint32_t        lastStatPollMs    = 0;

// ---------- helpers очереди ----------
static bool queueIsEmpty() {
    return queueHead == queueTail;
}

static bool queueIsFull() {
    return (uint8_t)((queueTail + 1) % CMD_QUEUE_SIZE) == queueHead;
}

static void queuePush(const String &cmd) {
    if (queueIsFull()) {
        btWebUI_log("[BT] queue FULL, drop: " + cmd, LogLevel::INFO);
        return;
    }
    cmdQueue[queueTail] = cmd;
    queueTail = (queueTail + 1) % CMD_QUEUE_SIZE;
}

static String queueFront() {
    if (queueIsEmpty()) return String();
    return cmdQueue[queueHead];
}

static void queuePop() {
    if (!queueIsEmpty()) {
        queueHead = (queueHead + 1) % CMD_QUEUE_SIZE;
    }
}

// ---------- изменение состояния + callback ----------
static void setBtState(BTConnState newState) {
    if (newState == btState) return;
    BTConnState old = btState;
    btState = newState;

    String msg = "[BT] State: ";
    switch (btState) {
        case BTConnState::DISCONNECTED:   msg += "DISCONNECTED";   break;
        case BTConnState::CONNECTING:     msg += "CONNECTING";     break;
        case BTConnState::CONNECTED_IDLE: msg += "CONNECTED_IDLE"; break;
        case BTConnState::PLAYING:        msg += "PLAYING";        break;
        case BTConnState::PAUSED:         msg += "PAUSED";         break;
    }
    btWebUI_log(msg, LogLevel::INFO);  // Важное событие - всегда

    if (stateCb) stateCb(old, newState);
}

// ---------- отправка ----------
static void sendCommandNow(const String &cmd) {
    if (!bt) return;

    btWebUI_log("[BT] >> " + cmd, LogLevel::VERBOSE);  // AT команды - verbose

    bt->print(cmd);
    bt->print("\r\n");

    cmdInProgress = true;
    cmdTimestamp  = millis();
}

// ---------- обновление DEVSTAT ----------
static void updateDevStat(int val) {
    devStat.powerOn        = (val & 0b00001) != 0;
    devStat.brDiscoverable = (val & 0b00010) != 0;
    devStat.bleAdvertising = (val & 0b00100) != 0;
    devStat.brScanning     = (val & 0b01000) != 0;
    devStat.bleScanning    = (val & 0b10000) != 0;

    // Сжатая строка - DEBUG уровень (опрос идёт каждые 3 сек)
    String line = "[BT] DEVSTAT=" + String(val) +
                  " P=" + String(devStat.powerOn) +
                  " DISC=" + String(devStat.brDiscoverable) +
                  " BLEADV=" + String(devStat.bleAdvertising) +
                  " BRSCAN=" + String(devStat.brScanning) +
                  " BLESCAN=" + String(devStat.bleScanning);
    btWebUI_log(line, LogLevel::DEBUG);
}

// ---------- разбор строк ----------
static void handleLine(const String &lineIn) {
    if (lineIn.isEmpty()) return;

    String line = lineIn;
    line.trim();

    // Ответы от модуля - VERBOSE (слишком часто)
    btWebUI_log("[BT] << " + line, LogLevel::VERBOSE);

    // --- базовые ответы ---
    if (line == F("OK")) {
        if (cmdInProgress) {
            cmdInProgress = false;
            if (!queueIsEmpty()) queuePop();
        }
        return;
    }

    if (line.startsWith(F("ERROR")) || line.startsWith(F("ERR"))) {
        if (cmdInProgress) {
            String cur = queueFront();
            btWebUI_log("[BT] CMD ERROR for: " + cur, LogLevel::INFO);
            cmdInProgress = false;
            if (!queueIsEmpty()) queuePop();
        }
        return;
    }

    // ---------- A2DP ----------
    if (line.startsWith(F("+A2DPSTAT="))) {
        int val = line.substring(10).toInt();
        switch (val) {
            case 0:
            case 1: setBtState(BTConnState::DISCONNECTED);   break;
            case 2: setBtState(BTConnState::CONNECTING);     break;
            case 3: setBtState(BTConnState::CONNECTED_IDLE); break;
            case 4: setBtState(BTConnState::PAUSED);         break;
            case 5: setBtState(BTConnState::PLAYING);        break;
        }
        return;
    }

    if (line.startsWith(F("+A2DPINFO="))) {
        btWebUI_log("[BT] A2DPINFO: " + line.substring(11), LogLevel::DEBUG);
        return;
    }

    // ---------- AVRCP ----------
    if (line.startsWith(F("+AVRCPSTAT="))) {
        int st = line.substring(12).toInt();
        btWebUI_log("[BT] AVRCP state=" + String(st), LogLevel::DEBUG);
        return;
    }

    // ---------- Browsing ----------
    if (line.startsWith(F("+BROWDATA="))) {
        btWebUI_log("[BT] BROWDATA: " + line, LogLevel::DEBUG);
        return;
    }

    // ---------- PLAYSTAT ----------
    if (line.startsWith(F("+PLAYSTAT="))) {
        int val = line.substring(10).toInt();
        // 0 Stopped, 1 Playing, 2 Paused, 3 FFwd, 4 FRew
        switch (val) {
            case 0: setBtState(BTConnState::CONNECTED_IDLE); break;
            case 1: setBtState(BTConnState::PLAYING);        break;
            case 2: setBtState(BTConnState::PAUSED);         break;
            case 3:
            case 4: setBtState(BTConnState::PLAYING);        break;
        }
        return;
    }

    // ---------- DEVSTAT ----------
    if (line.startsWith(F("+DEVSTAT="))) {
        int val = line.substring(9).toInt();
        updateDevStat(val);
        return;
    }

    // ---------- NAME / LENAME ----------
    if (line.startsWith(F("+NAME="))) {
        btWebUI_log("[BT] Device Name: " + line.substring(6), LogLevel::DEBUG);
        return;
    }

    if (line.startsWith(F("+LENAME="))) {
        btWebUI_log("[BT] BLE Name: " + line.substring(8), LogLevel::DEBUG);
        return;
    }

    // ---------- TRACKSTAT (прогресс воспроизведения) ----------
    // Формат: +TRACKSTAT=state,elapsed,total
    if (line.startsWith(F("+TRACKSTAT="))) {
        String params = line.substring(11);
        int comma1 = params.indexOf(',');
        int comma2 = params.indexOf(',', comma1 + 1);
        if (comma1 > 0 && comma2 > comma1) {
            // int state = params.substring(0, comma1).toInt();
            g_trackInfo.elapsedSec = params.substring(comma1 + 1, comma2).toInt();
            g_trackInfo.totalSec = params.substring(comma2 + 1).toInt();
            g_trackInfo.valid = true;
            
            // Конвертируем в минуты:секунды
            uint8_t elMin = g_trackInfo.elapsedSec / 60;
            uint8_t elSec = g_trackInfo.elapsedSec % 60;
            
            // Обновляем время на дисплее магнитолы!
            cdc_setPlayTime(elMin, elSec);
            
            // Логируем красиво (не каждую секунду, чтобы не спамить)
            static uint32_t lastLogTime = 0;
            if (millis() - lastLogTime > 5000) {  // раз в 5 сек
                lastLogTime = millis();
                int totMin = g_trackInfo.totalSec / 60;
                int totSec = g_trackInfo.totalSec % 60;
                char buf[32];
                snprintf(buf, sizeof(buf), "[BT] Track: %d:%02d / %d:%02d", elMin, elSec, totMin, totSec);
                btWebUI_log(String(buf), LogLevel::DEBUG);
            }
        }
        return;
    }

    // ---------- TRACKINFO (название трека) ----------
    // Формат: +TRACKINFO=title,artist,album
    if (line.startsWith(F("+TRACKINFO="))) {
        String params = line.substring(11);
        int comma1 = params.indexOf(',');
        int comma2 = params.indexOf(',', comma1 + 1);
        if (comma1 > 0) {
            g_trackInfo.title = params.substring(0, comma1);
            g_trackInfo.title.trim();
            if (comma2 > comma1) {
                g_trackInfo.artist = params.substring(comma1 + 1, comma2);
                g_trackInfo.artist.trim();
                g_trackInfo.album = params.substring(comma2 + 1);
                g_trackInfo.album.trim();
            } else {
                g_trackInfo.artist = params.substring(comma1 + 1);
                g_trackInfo.artist.trim();
                g_trackInfo.album = "";
            }
            g_trackInfo.valid = true;
            btWebUI_log("[BT] Now: " + g_trackInfo.title + " - " + g_trackInfo.artist, LogLevel::INFO);
        }
        return;
    }

    // Остальные ответы пока просто логируются выше как "<< ..."
}

// ---------- public API ----------

void bt1036_init(HardwareSerial &serial, uint8_t rxPin, uint8_t txPin) {
    bt = &serial;
    bt->begin(115200, SERIAL_8N1, rxPin, txPin);

    btWebUI_log("[BT] BT1036 init @115200", LogLevel::INFO);

    queueHead = queueTail = 0;
    cmdInProgress = false;
    rxLine.reserve(128);
    setBtState(BTConnState::DISCONNECTED);

    // Базовый стартовый набор
    queuePush(String(F("AT")));
    queuePush(String(F("AT+VER")));
    queuePush(String(F("AT+ADDR")));

    // Стартовый запрос статусов (пойдут из фонового опроса)
    lastStatPollMs = millis();
}

void bt1036_loop() {
    if (!bt) return;

    // приём UART
    while (bt->available()) {
        char c = bt->read();
        if (c == '\r') {
            // ignore
        } else if (c == '\n') {
            if (!rxLine.isEmpty()) {
                handleLine(rxLine);
                rxLine = "";
            }
        } else {
            rxLine += c;
            if (rxLine.length() > 250) rxLine = "";
        }
    }

    // таймаут команды
    if (cmdInProgress && (millis() - cmdTimestamp > CMD_TIMEOUT_MS)) {
        String cur = queueFront();
        if (cur.length()) {
            btWebUI_log("[BT] CMD TIMEOUT for: " + cur, LogLevel::INFO);
        }
        cmdInProgress = false;
        if (!queueIsEmpty()) queuePop();
    }

    // отправка следующей команды
    if (!cmdInProgress && !queueIsEmpty()) {
        sendCommandNow(queueFront());
    }

    // --- фоновый опрос статуса A2DP/DEVSTAT раз в 3 секунды ---
    uint32_t now = millis();
    if (!cmdInProgress && (now - lastStatPollMs > 3000)) {
        bt1036_requestA2dpStat();
        bt1036_requestDevStat();
        lastStatPollMs = now;
    }
}

// ---------- A2DP / AVRCP runtime ----------

void bt1036_startScan()      { queuePush(String(F("AT+SCAN=1"))); }
void bt1036_connectLast()    { queuePush(String(F("AT+A2DPCONN"))); }
void bt1036_disconnect()     { queuePush(String(F("AT+A2DPDISC"))); }

void bt1036_enterPairingMode() {
    // Отключаемся от текущего устройства и включаем режим сопряжения
    queuePush(String(F("AT+A2DPDISC")));
    queuePush(String(F("AT+HFPDISC")));
    queuePush(String(F("AT+SCAN=1")));
    btWebUI_log("[BT] Entering pairing mode...", LogLevel::INFO);
}

void bt1036_clearPairedDevices() {
    // Очищаем список сопряжённых устройств (AT+DELPD удаляет все)
    queuePush(String(F("AT+DELPD")));
    btWebUI_log("[BT] Paired devices list cleared", LogLevel::INFO);
}

void bt1036_playPause()      { queuePush(String(F("AT+PLAYPAUSE"))); }
void bt1036_play()           { queuePush(String(F("AT+PLAY"))); }
void bt1036_pause()          { queuePush(String(F("AT+PAUSE"))); }
void bt1036_stop()           { queuePush(String(F("AT+STOP"))); }
void bt1036_nextTrack()      { queuePush(String(F("AT+FORWARD"))); }
void bt1036_prevTrack()      { queuePush(String(F("AT+BACKWARD"))); }

void bt1036_requestA2dpStat()  { queuePush(String(F("AT+A2DPSTAT"))); }
void bt1036_requestA2dpInfo()  { queuePush(String(F("AT+A2DPINFO"))); }
void bt1036_requestAvrcpStat() { queuePush(String(F("AT+AVRCPSTAT"))); }

void bt1036_setAvrcpCfg(uint8_t cfg) {
    String cmd = String(F("AT+AVRCPCFG="));
    cmd += String(cfg);
    queuePush(cmd);
}

// ---------- HFP runtime ----------

void bt1036_hfpConnectLast() { queuePush(String(F("AT+HFPCONN"))); }
void bt1036_hfpDisconnect()  { queuePush(String(F("AT+HFPDISC"))); }
void bt1036_answerCall()     { queuePush(String(F("AT+HFPANSW"))); }
void bt1036_hangupCall()     { queuePush(String(F("AT+HFPCHUP"))); }

void bt1036_hfpThreeWay(uint8_t mode) {
    if (mode > 2) mode = 2;
    String cmd = String(F("AT+HFPMCAL="));
    cmd += String(mode);
    queuePush(cmd);
}

void bt1036_hfpVoiceRecognition(bool on) {
    String cmd = String(F("AT+HFPVR="));
    cmd += (on ? "1" : "0");
    queuePush(cmd);
}

void bt1036_setMicMute(bool muteOn) {
    String cmd = String(F("AT+MICMUTE="));
    cmd += (muteOn ? "1" : "0");
    queuePush(cmd);
}

// ---------- System ----------
void bt1036_softReboot() {
    queuePush(String(F("AT+REBOOT")));
}

void bt1036_setBtEnabled(bool enabled) {
    String cmd = String(F("AT+BTEN="));
    cmd += (enabled ? "1" : "0");
    queuePush(cmd);
}

// ---------- Геттеры / колбэки ----------
BTConnState bt1036_getState()      { return btState; }
BtDevStat   bt1036_getDevStat()    { return devStat; }

void bt1036_setStateCallback(BtStateCallback cb) {
    stateCb = cb;
}

// ---------- EEPROM / настройки ----------

void bt1036_getName() {
    queuePush(String(F("AT+NAME")));
}

void bt1036_setName(const String &name, bool suffix) {
    String cmd = String(F("AT+NAME="));
    cmd += name;
    cmd += ",";
    cmd += (suffix ? "1" : "0");
    queuePush(cmd);
}

void bt1036_getBLEName() {
    queuePush(String(F("AT+LENAME")));
}

void bt1036_setBLEName(const String &name, bool suffix) {
    String cmd = String(F("AT+LENAME="));
    cmd += name;
    cmd += ",";
    cmd += (suffix ? "1" : "0");
    queuePush(cmd);
}

void bt1036_setMicGain(uint8_t gain0_15) {
    if (gain0_15 > 15) gain0_15 = 15;
    String cmd = String(F("AT+MICGAIN="));
    cmd += String(gain0_15);
    queuePush(cmd);
}

void bt1036_setSpkVol(uint8_t a2dp0_15, uint8_t hfp0_15) {
    if (a2dp0_15 > 15) a2dp0_15 = 15;
    if (hfp0_15 > 15) hfp0_15 = 15;
    String cmd = String(F("AT+SPKVOL="));
    cmd += String(a2dp0_15);
    cmd += ",";
    cmd += String(hfp0_15);
    queuePush(cmd);
}

void bt1036_setTxPower(uint8_t level0_15) {
    if (level0_15 > 15) level0_15 = 15;
    String cmd = String(F("AT+TXPOWER="));
    cmd += String(level0_15);
    queuePush(cmd);
}

void bt1036_getProfile() {
    queuePush(String(F("AT+PROFILE")));
}

void bt1036_setProfile(uint16_t mask) {
    String cmd = String(F("AT+PROFILE="));
    cmd += String(mask);
    queuePush(cmd);
}

void bt1036_getAutoconn() {
    queuePush(String(F("AT+AUTOCONN")));
}

void bt1036_setAutoconn(uint16_t mask) {
    String cmd = String(F("AT+AUTOCONN="));
    cmd += String(mask);
    queuePush(cmd);
}

void bt1036_getSsp() {
    queuePush(String(F("AT+SSP")));
}

void bt1036_setSsp(uint8_t mode0_3) {
    if (mode0_3 > 3) mode0_3 = 3;
    String cmd = String(F("AT+SSP="));
    cmd += String(mode0_3);
    queuePush(cmd);
}

void bt1036_getCod() {
    queuePush(String(F("AT+COD")));
}

void bt1036_setCod(const String &codHex6) {
    String cmd = String(F("AT+COD="));
    cmd += codHex6;
    queuePush(cmd);
}

void bt1036_getSep() {
    queuePush(String(F("AT+SEP")));
}

void bt1036_setSep(uint8_t hexVal) {
    String cmd = String(F("AT+SEP="));
    cmd += String(hexVal);
    queuePush(cmd);
}

// ---------- HFP настройки ----------

void bt1036_requestHfpStat() {
    queuePush(String(F("AT+HFPSTAT")));
}

void bt1036_setHfpSampleRate(uint32_t rate) {
    // допустимые: 0 / 8000 / 16000 / 48000
    if (rate != 0 && rate != 8000 && rate != 16000 && rate != 48000) {
        rate = 16000;
    }
    String cmd = String(F("AT+HFPSR="));
    cmd += String(rate);
    queuePush(cmd);
}

void bt1036_setHfpConfig(uint8_t cfg) {
    // BIT0: auto reconnect
    // BIT1: echo cancellation
    // BIT2: 3-way calling
    String cmd = String(F("AT+HFPCFG="));
    cmd += String(cfg);
    queuePush(cmd);
}

// ---------- Диагностика ----------

void bt1036_requestDevStat() {
    queuePush(String(F("AT+DEVSTAT")));
}

void bt1036_requestStat() {
    queuePush(String(F("AT+STAT")));
}

// ---------- Одноразовая "фабричная" настройка (опционально) ----------
void bt1036_runFactorySetup() {
    btWebUI_log("[BT] Running factory setup...", LogLevel::INFO);

    // Имена
    bt1036_setName("VW_BT1036", false);
    bt1036_setBLEName("VW_BT1036", false);

    // Уровни
    bt1036_setMicGain(8);
    bt1036_setSpkVol(12, 12);
    bt1036_setTxPower(10);

    // Профили: HFP-HF + A2DP Sink + AVRCP Controller = 168
    const uint16_t profileMask  = 168;
    const uint16_t autoconnMask = 168;
    bt1036_setProfile(profileMask);
    bt1036_setAutoconn(autoconnMask);

    // SSP режим
    bt1036_setSsp(2);

    // Class of Device – car audio / hands-free
    bt1036_setCod("240404");

    // SEP — спец. режим, оставим 0
    bt1036_setSep(0);

    // HFP настройки
    bt1036_setHfpSampleRate(16000);
    uint8_t hfpCfg = 3; // BIT0=auto reconnect, BIT1=echo cancel, BIT2=0 (3-way off)
    bt1036_setHfpConfig(hfpCfg);
    
    // AVRCP настройки: автополучение ID3 + прогресс каждую секунду
    // BIT[0]=1 (auto ID3), BIT[1-3]=001 (1 sec interval) → 0b0011 = 3
    bt1036_setAvrcpCfg(3);

    btWebUI_log("[BT] Factory setup queued (check OKs, then reboot module).", LogLevel::INFO);
}

// ---------- Track Info getter ----------
TrackInfo bt1036_getTrackInfo() {
    return g_trackInfo;
}
