#include "bt1036_at.h"
#include "bt_webui.h"  // for btWebUI_log() and LogLevel
#include "vw_cdc.h"    // for cdc_setPlayTime()

#include <vector>

static HardwareSerial *bt = nullptr;

// ---------- command queue ----------
static const size_t   CMD_QUEUE_SIZE    = 40;
static const size_t   CMD_MAX_LEN       = 48;
static char           cmdQueue[CMD_QUEUE_SIZE][CMD_MAX_LEN];
static uint8_t        queueHead         = 0;
static uint8_t        queueTail         = 0;
static bool           cmdInProgress     = false;
static uint32_t       cmdTimestamp      = 0;
static const uint32_t CMD_TIMEOUT_MS    = 2000;

static String          rxLine;
static BTConnState     btState           = BTConnState::DISCONNECTED;
static BtDevStat       devStat{};

// HFP call/audio state
static BtHfpCallState  g_hfpCallState    = 1;     // 1=Standby (default)
static bool            g_hfpAudioActive  = false; // +HFPAUDIO=1

// Paired device storage and active MAC
static std::vector<PairedDevice> g_pairedDevices;
static String g_connectedMac = "";
static String g_connectedName = "";
static bool g_isParsingPList = false; // Flag: currently parsing +PLIST response
static uint32_t g_pListReqMs = 0;
static bool g_connInfoRequested = false;

// Current track information (from +TRACKSTAT and +TRACKINFO)
static TrackInfo       g_trackInfo       = {0, 0, "", "", "", false};
static bool            g_trackChanged    = false;

// AVRCP Ready state (module is ready to accept playback commands)
static bool            g_avrcpReady      = false;

// State change callback
static BtStateCallback stateCb           = nullptr;

// Background status polling
static uint32_t        lastStatPollMs    = 0;
static bool            g_pollingPaused   = false;

// ---------- queue helpers ----------
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
    strncpy(cmdQueue[queueTail], cmd.c_str(), CMD_MAX_LEN - 1);
    cmdQueue[queueTail][CMD_MAX_LEN - 1] = '\0'; // Ensure null-termination
    queueTail = (queueTail + 1) % CMD_QUEUE_SIZE;
}

void bt1036_clearQueue() {
    queueHead = queueTail;
    cmdInProgress = false;
    btWebUI_log("[BT] Command queue cleared", LogLevel::INFO);
}

static const char* queueFront() {
    if (queueIsEmpty()) return "";
    return cmdQueue[queueHead];
}

static void queuePop() {
    if (!queueIsEmpty()) {
        queueHead = (queueHead + 1) % CMD_QUEUE_SIZE;
    }
}

// ---------- state change + callback ----------
static void setBtState(BTConnState newState) {
    if (newState == btState) return;
    BTConnState old = btState;
    btState = newState;

    if (btState == BTConnState::DISCONNECTED) {
        g_connectedMac = "";
        g_connectedName = "";
        g_connInfoRequested = false;
        g_avrcpReady = false; 
    }

    String msg = "[BT] State: ";
    switch (btState) {
        case BTConnState::DISCONNECTED:   msg += "DISCONNECTED";   break;
        case BTConnState::CONNECTING:     msg += "CONNECTING";     break;
        case BTConnState::CONNECTED_IDLE: msg += "CONNECTED_IDLE"; break;
        case BTConnState::PLAYING:        msg += "PLAYING";        break;
        case BTConnState::PAUSED:         msg += "PAUSED";         break;
    }
    btWebUI_log(msg, LogLevel::INFO);  // Important event - always log

    if (stateCb) stateCb(old, newState);
}

static void requestConnectedDeviceInfoOnce() {
    if (g_connInfoRequested) return;
    g_connInfoRequested = true;

    // Best-effort: some firmwares support querying current device; if unsupported, it will just ERR once.
    queuePush(String(F("AT+A2DPDEV")));
    queuePush(String(F("AT+HFPDEV")));
}

// ---------- send command ----------
static void sendCommandNow(const char* cmd) {
    if (!bt || !cmd || cmd[0] == '\0') return;

    // Handle internal pseudo-commands
    if (strcmp(cmd, "AT+RESUMEPOLL") == 0) {
        bt1036_pausePolling(false); // Resume polling
        cmdInProgress = false;      // Consider command "executed"
        if (!queueIsEmpty()) queuePop(); // Move to next
        return;
    }

    btWebUI_log(String("[BT] >> ") + cmd, LogLevel::VERBOSE);  // AT commands - verbose

    bt->print(cmd);
    bt->print("\r\n");

    cmdInProgress = true;
    cmdTimestamp  = millis();
}

// ---------- DEVSTAT update ----------
static void updateDevStat(int val) {
    devStat.powerOn        = (val & 0b00001) != 0;
    devStat.brDiscoverable = (val & 0b00010) != 0;
    devStat.bleAdvertising = (val & 0b00100) != 0;
    devStat.brScanning     = (val & 0b01000) != 0;
    devStat.bleScanning    = (val & 0b10000) != 0;

    // Compact string - DEBUG level (polled every 3 sec)
    String line = "[BT] DEVSTAT=" + String(val) +
                  " P=" + String(devStat.powerOn) +
                  " DISC=" + String(devStat.brDiscoverable) +
                  " BLEADV=" + String(devStat.bleAdvertising) +
                  " BRSCAN=" + String(devStat.brScanning) +
                  " BLESCAN=" + String(devStat.bleScanning);
    btWebUI_log(line, LogLevel::DEBUG);
}

// Forward declarations (handleLine uses these before their definitions)
static void parsePList(const String &line);
static void parseA2dpDev(const String &line);

// ---------- line parsing ----------
static void handleLine(const String &lineIn) {
    if (lineIn.isEmpty()) return;

    String line = lineIn;
    line.trim();

    // Module responses - VERBOSE (too frequent to log at higher levels)
    btWebUI_log("[BT] << " + line, LogLevel::VERBOSE);

    // --- basic responses ---
    if (line == F("OK")) {
        if (g_isParsingPList) {
            g_isParsingPList = false;
            btWebUI_log("[BT] Paired list parsing finished (OK).", LogLevel::DEBUG);
        }
        if (cmdInProgress) {
            cmdInProgress = false;
            if (!queueIsEmpty()) queuePop();
        }
        return;
    }

    if (line.startsWith(F("ERROR")) || line.startsWith(F("ERR"))) {
        if (cmdInProgress) {
            const char* cur = queueFront();
            btWebUI_log(String("[BT] CMD ERROR for: ") + cur, LogLevel::INFO);
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

    // ---------- HFP ----------
    // +HFPSTAT=<state>{,<num>{,<num2>}}
    if (line.startsWith(F("+HFPSTAT="))) {
        int st = line.substring(9).toInt();
        if (st < 0) st = 0;
        if (st > 255) st = 255;
        g_hfpCallState = (BtHfpCallState)st;
        if (g_hfpCallState <= 3) {
            // Not in a call; voice audio should be inactive
            g_hfpAudioActive = false;
        }
        btWebUI_log(String("[BT] HFP state=") + String((int)g_hfpCallState), LogLevel::DEBUG);
        return;
    }

    // +HFPAUDIO=0/1 (voice audio routed to module)
    if (line.startsWith(F("+HFPAUDIO="))) {
        int v = line.substring(10).toInt();
        g_hfpAudioActive = (v == 1);
        btWebUI_log(String("[BT] HFP audio=") + (g_hfpAudioActive ? "ON" : "OFF"), LogLevel::DEBUG);
        return;
    }

    if (line.startsWith(F("+A2DPINFO="))) {
        btWebUI_log("[BT] A2DPINFO: " + line.substring(11), LogLevel::DEBUG);
        return;
    }

    // ---------- AVRCP ----------
    if (line.startsWith(F("+AVRCPSTAT="))) {
        int st = line.substring(11).toInt();
        btWebUI_log("[BT] AVRCP state=" + String(st), LogLevel::DEBUG);

        if (st >= 3) {
            g_avrcpReady = true;
        } else {
            g_avrcpReady = false;
        }

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
            case 1: 
                setBtState(BTConnState::PLAYING);
                g_avrcpReady = true; // We are playing, so AVRCP is active
                break;
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

    // ---------- TRACKSTAT (playback progress) ----------
    // Format: +TRACKSTAT=state,elapsed,total
    if (line.startsWith(F("+TRACKSTAT="))) {
        String params = line.substring(11);
        int comma1 = params.indexOf(',');
        int comma2 = params.lastIndexOf(','); // Use lastIndexOf for safer parsing if multiple commas exist
        
        // Some module versions send +TRACKSTAT=state,elapsed,total
        // Others might send +TRACKSTAT=elapsed,total
        // We try to be robust.
        
        if (comma1 > 0) {
            if (comma2 > comma1) {
                // Three components: state, elapsed, total
                g_trackInfo.elapsedSec = params.substring(comma1 + 1, comma2).toInt();
                g_trackInfo.totalSec = params.substring(comma2 + 1).toInt();
            } else {
                // Two components: elapsed, total (?) - unusual but possible
                // attempting to parse as elapsed, total
                g_trackInfo.elapsedSec = params.substring(0, comma1).toInt();
                g_trackInfo.totalSec = params.substring(comma1 + 1).toInt();
            }

            g_trackInfo.valid = true;
            
            // Convert to minutes:seconds
            uint8_t elMin = g_trackInfo.elapsedSec / 60;
            uint8_t elSec = g_trackInfo.elapsedSec % 60;
            
            // Display time is now updated in the main loop (main.cpp)
            
            // Log nicely (not every second, to avoid spam)
            static uint32_t lastLogTime = 0;
            if (millis() - lastLogTime > 5000) {  // every 5 sec
                lastLogTime = millis();
                int totMin = g_trackInfo.totalSec / 60;
                int totSec = g_trackInfo.totalSec % 60;
                char buf[64];
                snprintf(buf, sizeof(buf), "[BT] Track status: %d:%02d / %d:%02d", elMin, elSec, totMin, totSec);
                btWebUI_log(String(buf), LogLevel::DEBUG);
            }
        }
        return;
    }

    // ---------- TRACKINFO (track name) ----------
    // Format: +TRACKINFO=title,artist,album
    if (line.startsWith(F("+TRACKINFO="))) {
        g_avrcpReady = true; // Receiving track info implies ready state
        String params = line.substring(11);
        int comma1 = params.indexOf(',');
        int comma2 = params.indexOf(',', comma1 + 1);
        if (comma1 > 0) {
            String newTitle = params.substring(0, comma1);
            newTitle.trim();
            String newArtist = "";
            String newAlbum = "";

            if (comma2 > comma1) {
                newArtist = params.substring(comma1 + 1, comma2);
                newArtist.trim();
                newAlbum = params.substring(comma2 + 1);
                newAlbum.trim();
            } else {
                newArtist = params.substring(comma1 + 1);
                newArtist.trim();
            }

            // Check if track changed (ignore album changes)
            if (g_trackInfo.title != newTitle || g_trackInfo.artist != newArtist) {
                g_trackChanged = true;
            }

            g_trackInfo.title = newTitle;
            g_trackInfo.artist = newArtist;
            g_trackInfo.album = newAlbum;
            g_trackInfo.valid = true;
            btWebUI_log("[BT] Now: " + g_trackInfo.title + " - " + g_trackInfo.artist, LogLevel::INFO);
        }
        return;
    } else if (line.startsWith(F("+PLIST="))) {
        parsePList(line);
    } else if (line.startsWith(F("+A2DPDEV"))) {
        parseA2dpDev(line);
    } else if (line.startsWith(F("+HFPDEV"))) {
        // Some firmwares provide the connected phone info via HFPDEV as well.
        parseA2dpDev(line);
    }
}

/**
 * @brief Parser for AT+PLIST response
 * Format: +PLIST=<index>,<profiles>,<mac>,<name>
 * End of list: +PLIST=E
 */
static void parsePList(const String &line) {
    if (!g_isParsingPList) return;

    if (line.length() > 8 && line.startsWith("+PLIST=")) {
        if (line[7] == 'E') {
            g_isParsingPList = false;
            btWebUI_log("[BT] Paired list parsing finished.", LogLevel::DEBUG);
            return;
        }

        String data = line.substring(7);
        int first_comma = data.indexOf(',');
        int second_comma = data.indexOf(',', first_comma + 1);
        int third_comma = data.indexOf(',', second_comma + 1);

        if (first_comma > 0 && second_comma > 0 && third_comma > 0) {
            PairedDevice dev;
            dev.index = data.substring(0, first_comma).toInt();
            dev.mac = data.substring(second_comma + 1, third_comma);
            dev.name = data.substring(third_comma + 1);
            dev.mac.trim();
            dev.name.trim();
            g_pairedDevices.push_back(dev);
            btWebUI_log(String("[BT] Found paired device: ") + dev.name + " (" + dev.mac + ")", LogLevel::DEBUG);
        }
    }
}

/**
 * @brief Parser for +A2DPDEV event
 * Format: +A2DPDEV=<mac>,<name>
 */
static void parseA2dpDev(const String &line) {
    if (line.length() > 8 && (line.startsWith("+A2DPDEV") || line.startsWith("+HFPDEV"))) {
        int sep = line.indexOf('=');
        if (sep < 0) sep = line.indexOf(':');
        if (sep < 0) return;

        String data = line.substring(sep + 1);
        int comma = data.indexOf(',');
        if (comma > 0) {
            g_connectedMac = data.substring(0, comma);
            g_connectedMac.trim();
            g_connectedName = data.substring(comma + 1);
            g_connectedName.trim();
            btWebUI_log(String("[BT] Connected device: ") + g_connectedName + " (" + g_connectedMac + ")", LogLevel::INFO);
        }
    }
}


/**
 * @brief Parser for AT+A2DPSTAT response
 * Format: +A2DPSTAT:<state>,<bitpool>,<sample rate>,<channel mode>
 */
static void parseA2dpStat(const String &line) {
    if (line.length() > 11 && line.startsWith("+A2DPSTAT:")) {
        String data = line.substring(11);
        int first_comma = data.indexOf(',');
        int second_comma = data.indexOf(',', first_comma + 1);
        int third_comma = data.indexOf(',', second_comma + 1);

        if (first_comma > 0 && second_comma > 0 && third_comma > 0) {
            int state = data.substring(0, first_comma).toInt();
            int bitpool = data.substring(first_comma + 1, second_comma).toInt();
            int sampleRate = data.substring(second_comma + 1, third_comma).toInt();
            int channelMode = data.substring(third_comma + 1).toInt();

            // Log A2DP state
            String stateStr;
            switch (state) {
                case 0: stateStr = "DISCONNECTED"; break;
                case 1: stateStr = "CONNECTING";   break;
                case 2: stateStr = "CONNECTED";    break;
                case 3: stateStr = "DISCONNECTING";break;
                default: stateStr = "UNKNOWN";
            }

            btWebUI_log(String("[BT] A2DP State: ") + stateStr, LogLevel::DEBUG);
            btWebUI_log(String("[BT] A2DP Bitpool: ") + bitpool, LogLevel::DEBUG);
            btWebUI_log(String("[BT] A2DP Sample Rate: ") + sampleRate, LogLevel::DEBUG);
            btWebUI_log(String("[BT] A2DP Channel Mode: ") + channelMode, LogLevel::DEBUG);
        }
    }
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

    // Basic startup commands
    queuePush(String(F("AT")));
    queuePush(String(F("AT+VER")));
    queuePush(String(F("AT+ADDR")));

    // Initial status request (will be handled by background polling)
    lastStatPollMs = millis();
}

void bt1036_loop() {
    if (!bt) return;

    // UART receive
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

    // Command timeout
    if (cmdInProgress && (millis() - cmdTimestamp > CMD_TIMEOUT_MS)) {
        const char* cur = queueFront();
        if (cur && cur[0] != '\0') {
            btWebUI_log(String("[BT] CMD TIMEOUT for: ") + cur, LogLevel::INFO);
        }
        cmdInProgress = false;
        if (!queueIsEmpty()) queuePop();
    }

    // Paired list parsing timeout (prevent pending from hanging forever)
    if (g_isParsingPList && g_pListReqMs != 0 && (millis() - g_pListReqMs > 3000)) {
        g_isParsingPList = false;
        btWebUI_log("[BT] Paired list parsing timeout.", LogLevel::DEBUG);
    }

    // Send next command
    if (!cmdInProgress && !queueIsEmpty()) {
        sendCommandNow(queueFront());
    }

    // Polling logic for time updates and connection health
    if (!g_pollingPaused && (millis() - lastStatPollMs > 3000)) {
        lastStatPollMs = millis();
        // Request stats (A2DP status, Device status)
        queuePush(String(F("AT+A2DPSTAT")));
        queuePush(String(F("AT+DEVSTAT")));
    }
}

// ---------- A2DP / AVRCP runtime ----------

void bt1036_startScan()      { queuePush(String(F("AT+SCAN=1"))); }
void bt1036_connectLast()    { queuePush(String(F("AT+A2DPCONN"))); }
void bt1036_disconnect()     { queuePush(String(F("AT+A2DPDISC"))); }

void bt1036_enterPairingMode() {
    // Disconnect from current device and enable pairing mode
    queuePush(String(F("AT+A2DPDISC")));
    queuePush(String(F("AT+HFPDISC")));
    queuePush(String(F("AT+SCAN=1")));
    btWebUI_log("[BT] Entering pairing mode...", LogLevel::INFO);
}

void bt1036_clearPairedDevices() {
    // Clear all paired devices (AT+PLIST=0)
    queuePush(String(F("AT+PLIST=0")));
    btWebUI_log("[BT] Paired devices list cleared", LogLevel::INFO);
}

void bt1036_playPause()      { queuePush(String(F("AT+PLAYPAUSE"))); }
void bt1036_play()           { queuePush(String(F("AT+PLAYPAUSE"))); }
void bt1036_pause()          { queuePush(String(F("AT+PLAYPAUSE"))); }
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

void bt1036_setVolume(uint8_t volume) {
    if (volume > 15) volume = 15;
    String cmd = String(F("AT+SPKVOL="));
    cmd += String(volume);
    cmd += ",";
    cmd += String(volume);
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
    bt1036_clearQueue();
    queuePush(String(F("AT+REBOOT")));
}

void bt1036_setBtEnabled(bool enabled) {
    String cmd = String(F("AT+BTEN="));
    cmd += (enabled ? "1" : "0");
    queuePush(cmd);
}

void bt1036_sendRawCommand(const String& cmd) {
    if (cmd.length() > 0) {
        queuePush(cmd);
    }
}

// ---------- Getters / callbacks ----------
BTConnState bt1036_getState()      { return btState; }
BtDevStat   bt1036_getDevStat()    { return devStat; }

void bt1036_setStateCallback(BtStateCallback cb) {
    stateCb = cb;
}

// ---------- HFP state helpers ----------

BtHfpCallState bt1036_getHfpCallState() {
    return g_hfpCallState;
}

bool bt1036_isHfpCallIncoming() {
    return g_hfpCallState == 5;
}

bool bt1036_isHfpCallInProgress() {
    // 4..10 indicate outgoing/incoming/active/held/3-way states
    return g_hfpCallState >= 4;
}

bool bt1036_isHfpAudioActive() {
    return g_hfpAudioActive;
}

// ---------- EEPROM / configuration ----------

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

// ---------- HFP configuration ----------

void bt1036_requestHfpStat() {
    queuePush(String(F("AT+HFPSTAT")));
}

void bt1036_setHfpSampleRate(uint32_t rate) {
    // Allowed values: 0 / 8000 / 16000 / 48000
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

// ---------- Diagnostics ----------

void bt1036_requestDevStat() {
    queuePush(String(F("AT+DEVSTAT")));
}

void bt1036_requestStat() {
    queuePush(String(F("AT+STAT")));
}

// ---------- Polling control ----------

void bt1036_pausePolling(bool pause) {
    g_pollingPaused = pause;
    if (!pause) {
        // Reset timer on resume to avoid immediate polling
        lastStatPollMs = millis();
    }
    btWebUI_log(String("[BT] Polling ") + (pause ? "PAUSED" : "RESUMED"), LogLevel::INFO);
}

// ---------- One-time factory setup (optional) ----------
void bt1036_runFactorySetup() {
    btWebUI_log("[BT] Running factory setup...", LogLevel::INFO);
    bt1036_clearQueue();
    bt1036_pausePolling(true); // Pause polling during setup

    // 1. Stop all active connections
    queuePush(String(F("AT+SCAN=0")));     // Stop scanning
    queuePush(String(F("AT+A2DPDISC")));   // Disconnect A2DP
    queuePush(String(F("AT+HFPDISC")));    // Disconnect HFP
    
    // Names
    bt1036_setName("VW_BT1036", false);
    bt1036_setBLEName("VW_BT1036", false);

    // Levels
    bt1036_setMicGain(8);
    bt1036_setSpkVol(12, 15); // A2DP=12 (default), HFP=15 (MAX for louder calls)
    bt1036_setTxPower(2);     // Reduced RF power (2) to eliminate digital buzzing/interference

    // Profiles: HFP-HF + A2DP Sink + AVRCP Controller = 168
    // IMPORTANT: These commands often require a "clean" state
    const uint16_t profileMask  = 168;
    const uint16_t autoconnMask = 168;
    bt1036_setProfile(profileMask);
    bt1036_setAutoconn(autoconnMask);

    // SSP mode
    bt1036_setSsp(2);

    // Class of Device – car audio / hands-free
    bt1036_setCod("240404");

    // SEP — change separator to 0xFF for stable parsing
    bt1036_setSep(0);

    // I2S Config not modified: project uses analog output (SPK_*).

    // HFP configuration
    bt1036_setHfpSampleRate(16000);
    uint8_t hfpCfg = 3; // BIT0=auto reconnect, BIT1=echo cancel, BIT2=0 (3-way off)
    bt1036_setHfpConfig(hfpCfg);
    
    // AVRCP config: auto-fetch ID3 + progress every second
    // BIT[0]=1 (auto ID3), BIT[1-3]=001 (1 sec interval) → 0b0011 = 3
    bt1036_setAvrcpCfg(3);
    
    // Internal command to resume polling after all setup commands complete
    queuePush(String(F("AT+RESUMEPOLL")));

    btWebUI_log("[BT] Factory setup queued (check OKs, then reboot module).", LogLevel::INFO);
}

// ---------- Track Info getter ----------
TrackInfo bt1036_getTrackInfo() {
    return g_trackInfo;
}

bool bt1036_checkTrackChanged() {
    if (g_trackChanged) {
        g_trackChanged = false;
        return true;
    }
    return false;
}

// ---- Paired device management ----
void bt1036_requestPairedList() {
    g_isParsingPList = true;
    g_pListReqMs = millis();
    g_pairedDevices.clear();
    queuePush(F("AT+PLIST"));
}

void bt1036_deleteDevice(const String& mac) {
    if (mac.length() == 12) {
        queuePush("AT+PLIST=" + mac);
    }
}

const std::vector<PairedDevice>& bt1036_getPairedList() {
    return g_pairedDevices;
}

String bt1036_getConnectedMac() {
    return g_connectedMac;
}

String bt1036_getConnectedName() {
    return g_connectedName;
}

bool bt1036_isPairedListPending() {
    return g_isParsingPList;
}

bool bt1036_isAvrcpReady() {
    return g_avrcpReady;
}
