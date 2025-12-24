/**
 * @file main.cpp
 * @brief VW CDC Bluetooth Emulator - Main Application
 * 
 * Emulates a VW CD Changer (CDC) to integrate BT1036C Bluetooth module
 * with VW RNS-MFD head unit. Translates button presses from the radio
 * into Bluetooth A2DP/AVRCP/HFP controls.
 * 
 * Button Mapping:
 *   CD1 = Play/Pause toggle
 *   CD2 = Stop
 *   CD3 = HFP Mic Mute toggle
 *   CD4 = Enter Pairing Mode (TRACK 80)
 *   CD5 = Disconnect current device
 *   CD6 = Clear all paired devices
 *   CD6 (double press) = Toggle WiFi ON/OFF
 *   SCAN = Hangup call
 *   MIX = Answer call
 *   <</>>	= Prev/Next track
 * 
 * Track Display Status:
 *   TRACK 80 = Waiting for BT connection
 *   TRACK 10 = Just connected (5 sec)
 *   TRACK 1+ = Normal playback with time from BT
 *   TRACK 90 = WiFi OFF
 *   TRACK 91 = WiFi ON
 */

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <Preferences.h> // Для сохранения состояния WiFi

#include "vw_cdc.h"
#include "bt1036_at.h"
#include "bt_webui.h"

// ============================================================================
// PIN CONFIGURATION (ESP-WROVER-KIT / ESP32)
// ============================================================================
static const uint8_t BT_RX_PIN = 16;  // ESP RX ← BT1036 TX (Serial2)
static const uint8_t BT_TX_PIN = 17;  // ESP TX → BT1036 RX (Serial2)

static const uint8_t CDC_SCK_PIN  = 18;  // VSPI CLK → VW Radio
static const uint8_t CDC_MISO_PIN = -1;  // Not used (radio doesn't send SPI data back)
static const uint8_t CDC_MOSI_PIN = 23;  // VSPI MOSI → VW Radio
static const int8_t  CDC_SS_PIN   = -1;  // Not used (single device)
static const uint8_t CDC_NEC_PIN  =  4;  // VW DataOut ← Radio (button commands)

// ============================================================================
// WIFI STATE
// ============================================================================
static bool g_wifiEnabled = true;
static Preferences g_prefs;

// ============================================================================
// GLOBAL STATE
// ============================================================================
static uint8_t g_currentDisc  = 1;   // Current CD number (always 1)
static uint8_t g_currentTrack = 1;   // Current track number (1-99, or 80/10 for status)
static bool g_hfpMuted = false;      // HFP microphone mute state
static bool g_isPlaying = false;     // Playback state for toggle logic

// Timers for SCAN/MIX indicator pulse (500ms on, then off)
static uint32_t g_scanResetTime = 0;
static uint32_t g_mixResetTime = 0;

// Button Debounce & Double Press state
static CdcButton g_lastButton = CdcButton::UNKNOWN;
static uint32_t  g_lastButtonTime = 0;
static uint32_t  g_cd6PressTime = 0;   // Для отслеживания двойного нажатия CD6

// ============================================================================
// DISPLAY MODE STATE MACHINE
// ============================================================================
enum class DisplayMode {
    WAITING_FOR_BT,      // TRACK 80 - waiting for connection
    JUST_CONNECTED,      // TRACK 10 - just connected (5 sec delay)
    NORMAL_PLAYBACK      // Normal mode - track/time from BT
};

static DisplayMode g_displayMode = DisplayMode::WAITING_FOR_BT;
static uint32_t g_connectedShowTime = 0;             // Timestamp when TRACK 10 was shown
static BTConnState g_lastBtState = BTConnState::DISCONNECTED;
static bool g_autoPlaySent = false;                  // Auto-play command already sent
static bool g_isPairingMode = false;                 // true = waiting for NEW device (CD4/CD6)

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

/** Включение/выключение WiFi */
static void toggleWiFi() {
    g_wifiEnabled = !g_wifiEnabled;
    g_prefs.begin("sys_config", false);
    g_prefs.putBool("wifi_on", g_wifiEnabled);
    g_prefs.end();
    
    String msg = String("[MAIN] WiFi turned ") + (g_wifiEnabled ? "ON" : "OFF") + ". Rebooting to apply...";
    btWebUI_log(msg, LogLevel::INFO);
    
    // Показываем на дисплее статус
    cdc_setDiscTrack(1, g_wifiEnabled ? 91 : 90); // 91 = WIFI ON, 90 = WIFI OFF
    
    delay(2000); // Даем время прочитать сообщение и увидеть на дисплее
    ESP.restart();
}

/** Increment track number (1-99 wrap) */
static void bumpTrackForward() {
    g_currentTrack = (g_currentTrack < 99) ? g_currentTrack + 1 : 1;
}

/** Decrement track number (1-99 wrap) */
static void bumpTrackBackward() {
    g_currentTrack = (g_currentTrack > 1) ? g_currentTrack - 1 : 99;
}

/** Toggle HFP microphone mute */
static void toggleHfpMute() {
    g_hfpMuted = !g_hfpMuted;
    bt1036_setMicMute(g_hfpMuted);
    btWebUI_log(String("[MAIN] HFP mic mute: ") + (g_hfpMuted ? "ON" : "OFF"), LogLevel::INFO);
}

// ============================================================================
// BUTTON HANDLER
// ============================================================================

/** Get button name for logging */
static const char* getButtonName(CdcButton btn) {
    switch (btn) {
        case CdcButton::NEXT_TRACK:    return "NEXT_TRACK";
        case CdcButton::PREV_TRACK:    return "PREV_TRACK";
        case CdcButton::NEXT_DISC:     return "NEXT_DISC";
        case CdcButton::PREV_DISC:     return "PREV_DISC";
        case CdcButton::PLAY_PAUSE:    return "PLAY_PAUSE";
        case CdcButton::SCAN_TOGGLE:   return "SCAN";
        case CdcButton::RANDOM_TOGGLE: return "RANDOM/MIX";
        case CdcButton::STOP:          return "STOP";
        case CdcButton::DISC_1:        return "CD1";
        case CdcButton::DISC_2:        return "CD2";
        case CdcButton::DISC_3:        return "CD3";
        case CdcButton::DISC_4:        return "CD4";
        case CdcButton::DISC_5:        return "CD5";
        case CdcButton::DISC_6:        return "CD6";
        case CdcButton::DISC_6_DOUBLE_PRESS: return "CD6_DOUBLE_PRESS";
        case CdcButton::UNKNOWN:       return "UNKNOWN";
        default:                       return "???";
    }
}

// Новый обработчик с debounce и double-press логикой
static void onCdcButton(CdcButton btn) {
    uint32_t now = millis();
    String logMsg;

    // --- Debounce Filter ---
    if (btn == g_lastButton && (now - g_lastButtonTime < 300)) {
        return; 
    }
    g_lastButton = btn;
    g_lastButtonTime = now;

    // --- Double Press Logic for CD6 ---
    if (btn == CdcButton::DISC_6) {
        if (now - g_cd6PressTime < 500) {
            g_cd6PressTime = 0; 
            return onCdcButton(CdcButton::DISC_6_DOUBLE_PRESS);
        } else {
            g_cd6PressTime = now;
            return;
        }
    }
    
    const char* btnName = getButtonName(btn);

    switch (btn) {
        case CdcButton::NEXT_TRACK:
            if (g_displayMode != DisplayMode::NORMAL_PLAYBACK) {
                g_displayMode = DisplayMode::NORMAL_PLAYBACK;
                g_currentTrack = 1;
            }
            bumpTrackForward();
            cdc_setDiscTrack(g_currentDisc, g_currentTrack);
            bt1036_nextTrack();
            logMsg = String("[BTN] ") + btnName + " → BT: Next, Track " + String(g_currentTrack);
            break;

        case CdcButton::PREV_TRACK:
            if (g_displayMode != DisplayMode::NORMAL_PLAYBACK) {
                g_displayMode = DisplayMode::NORMAL_PLAYBACK;
                g_currentTrack = 2;
            }
            bumpTrackBackward();
            cdc_setDiscTrack(g_currentDisc, g_currentTrack);
            bt1036_prevTrack();
            logMsg = String("[BTN ") + btnName + " → BT: Prev, Track " + String(g_currentTrack);
            break;

        case CdcButton::PLAY_PAUSE:
            g_isPlaying = !g_isPlaying;
            if (g_isPlaying) {
                bt1036_play();
                cdc_setPlayState(CdcPlayState::PLAYING);
            } else {
                bt1036_pause();
                cdc_setPlayState(CdcPlayState::PAUSED);
            }
            logMsg = String("[BTN] ") + btnName + " → BT: " + (g_isPlaying ? "Play" : "Pause");
            break;

        case CdcButton::STOP:
            g_isPlaying = false;
            bt1036_stop();
            cdc_setPlayState(CdcPlayState::STOPPED);
            logMsg = String("[BTN] ") + btnName + " → BT: Stop";
            break;

        case CdcButton::NEXT_DISC:
        case CdcButton::PREV_DISC:
            logMsg = String("[BTN] ") + btnName + " → (ignored)";
            break;

        case CdcButton::DISC_1: {
            g_isPlaying = !g_isPlaying;
            if (g_isPlaying) {
                bt1036_play();
                cdc_setPlayState(CdcPlayState::PLAYING);
                logMsg = String("[BTN] ") + btnName + " → BT: Play";
            } else {
                bt1036_pause();
                cdc_setPlayState(CdcPlayState::PAUSED);
                logMsg = String("[BTN] ") + btnName + " → BT: Pause";
            }
            break;
        }

        case CdcButton::DISC_2:
            bt1036_stop();
            cdc_setPlayState(CdcPlayState::STOPPED);
            logMsg = String("[BTN] ") + btnName + " → BT: Stop";
            break;

        case CdcButton::DISC_3:
            toggleHfpMute();
            logMsg = String("[BTN] ") + btnName + " → Mic Mute: " + (g_hfpMuted ? "ON" : "OFF");
            break;

        case CdcButton::DISC_4:
            bt1036_enterPairingMode();
            g_displayMode = DisplayMode::WAITING_FOR_BT;
            g_isPairingMode = true;
            g_currentTrack = 80;
            cdc_setDiscTrack(g_currentDisc, g_currentTrack);
            logMsg = String("[BTN] ") + btnName + " → BT: Pairing Mode (TRACK 80)";
            break;

        case CdcButton::DISC_5:
            bt1036_disconnect();
            bt1036_hfpDisconnect();
            g_displayMode = DisplayMode::WAITING_FOR_BT;
            g_currentTrack = 80;
            cdc_setDiscTrack(g_currentDisc, g_currentTrack);
            logMsg = String("[BTN] ") + btnName + " → BT: Disconnect";
            break;

        case CdcButton::DISC_6:
            // Handled in loop() for delayed single press
            break;
            
        case CdcButton::DISC_6_DOUBLE_PRESS:
            toggleWiFi();
            logMsg = String("[BTN] ") + btnName + " → Toggle WiFi";
            break;
            
        case CdcButton::SCAN_TOGGLE:
            bt1036_hangupCall();
            cdc_setScan(true);
            g_scanResetTime = millis() + 500;
            logMsg = String("[BTN] ") + btnName + " → HFP: Hangup";
            break;

        case CdcButton::RANDOM_TOGGLE:
            bt1036_answerCall();
            cdc_setRandom(true);
            g_mixResetTime = millis() + 500;
            logMsg = String("[BTN] ") + btnName + " → HFP: Answer Call";
            break;

        case CdcButton::UNKNOWN:
        default:
            logMsg = String("[BTN] ") + btnName + " \u2192 (no action)";
            break;
    }
    
    if (logMsg.length() > 0) {
        btWebUI_log(logMsg, LogLevel::INFO);
    }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    esp_task_wdt_init(15, true); 
    esp_task_wdt_add(NULL);

    g_prefs.begin("sys_config", true);
    g_wifiEnabled = g_prefs.getBool("wifi_on", true);
    g_prefs.end();

    if (g_wifiEnabled) {
        btWebUI_init();
    }

    esp_reset_reason_t reason = esp_reset_reason();
    const char* reasonStr = "Unknown";
    switch (reason) {
        case ESP_RST_POWERON:   reasonStr = "Power-on"; break;
        case ESP_RST_EXT:       reasonStr = "External reset"; break;
        case ESP_RST_SW:        reasonStr = "Software reset"; break;
        case ESP_RST_PANIC:     reasonStr = "Panic/exception"; break;
        case ESP_RST_INT_WDT:   reasonStr = "Interrupt watchdog"; break;
        case ESP_RST_TASK_WDT:  reasonStr = "Task watchdog"; break;
        case ESP_RST_WDT:       reasonStr = "Other watchdog"; break;
        case ESP_RST_DEEPSLEEP: reasonStr = "Deep sleep wake"; break;
        case ESP_RST_BROWNOUT:  reasonStr = ">>> BROWNOUT <<<"; break;
        case ESP_RST_SDIO:      reasonStr = "SDIO"; break;
        default: break;
    }
    
    btWebUI_log(String("[MAIN] Reset reason: ") + reasonStr, LogLevel::INFO);
    btWebUI_log(String("[MAIN] WiFi is ") + (g_wifiEnabled ? "ENABLED" : "DISABLED"), LogLevel::INFO);
    btWebUI_log("[MAIN] VW CDC + BT1036 emulator start", LogLevel::INFO);

    bt1036_init(Serial2, BT_RX_PIN, BT_TX_PIN);

    g_currentTrack = 80;
    g_displayMode = DisplayMode::WAITING_FOR_BT;
    cdc_setDiscTrack(g_currentDisc, g_currentTrack);
    cdc_setPlayState(CdcPlayState::PLAYING);
    cdc_setRandom(false);
    cdc_setScan(false);
    cdc_init(CDC_SCK_PIN, CDC_MISO_PIN, CDC_MOSI_PIN, CDC_SS_PIN, CDC_NEC_PIN, onCdcButton);

    btWebUI_log("[MAIN] Init complete.", LogLevel::INFO);
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
    esp_task_wdt_reset();

    bt1036_loop();
    cdc_loop();
    if (g_wifiEnabled) {
        btWebUI_loop();
    }
    
    if (g_cd6PressTime > 0 && (millis() - g_cd6PressTime >= 500)) {
        g_cd6PressTime = 0; 
        
        // Теперь выполняем действие для ОДИНОЧНОГО нажатия CD6
        bt1036_clearPairedDevices();
        g_displayMode = DisplayMode::WAITING_FOR_BT;
        g_isPairingMode = true;
        g_currentTrack = 80;
        cdc_setDiscTrack(g_currentDisc, g_currentTrack);
        btWebUI_log("[BTN] CD6 → BT: Clear Paired Devices", LogLevel::INFO);
    }

    if (g_scanResetTime > 0 && millis() > g_scanResetTime) {
        g_scanResetTime = 0;
        cdc_setScan(false);
    }
    
    if (g_mixResetTime > 0 && millis() > g_mixResetTime) {
        g_mixResetTime = 0;
        cdc_setRandom(false);
        cdc_resetModeFF();
    }
    
    BTConnState currentBtState = bt1036_getState();
    
    if (g_lastBtState == BTConnState::DISCONNECTED && 
        (currentBtState == BTConnState::CONNECTED_IDLE || 
         currentBtState == BTConnState::PLAYING || 
         currentBtState == BTConnState::PAUSED)) {
        
        bt1036_setVolume(15);
        btWebUI_log("[MAIN] Set BT volume to MAX (15)", LogLevel::INFO);

        if (g_isPairingMode) {
            g_displayMode = DisplayMode::JUST_CONNECTED;
            g_connectedShowTime = millis();
            g_currentTrack = 10;
            cdc_setDiscTrack(g_currentDisc, g_currentTrack);
            g_autoPlaySent = false;
            btWebUI_log("[MAIN] New device connected! Showing TRACK 10 for 5 sec", LogLevel::INFO);
        } else {
            g_displayMode = DisplayMode::NORMAL_PLAYBACK;
            g_currentTrack = 1;
            g_isPlaying = true;
            cdc_setDiscTrack(g_currentDisc, g_currentTrack);
            cdc_setPlayState(CdcPlayState::PLAYING);
            
            if (!g_autoPlaySent) {
                g_autoPlaySent = true;
                bt1036_play();
                btWebUI_log("[MAIN] Auto-reconnect! Instant play sent", LogLevel::INFO);
            }
        }
    }
    
    if (currentBtState == BTConnState::DISCONNECTED && 
        g_lastBtState != BTConnState::DISCONNECTED) {
        g_displayMode = DisplayMode::WAITING_FOR_BT;
        g_currentTrack = 80;
        cdc_setDiscTrack(g_currentDisc, g_currentTrack);
        g_autoPlaySent = false;
        btWebUI_log("[MAIN] BT Disconnected. Showing TRACK 80", LogLevel::INFO);
    }
    
    g_lastBtState = currentBtState;
    
    if (g_displayMode == DisplayMode::JUST_CONNECTED) {
        if (millis() - g_connectedShowTime > 5000) {
            g_displayMode = DisplayMode::NORMAL_PLAYBACK;
            g_currentTrack = 1;
            g_isPlaying = true;
            g_isPairingMode = false;
            cdc_setDiscTrack(g_currentDisc, g_currentTrack);
            btWebUI_log("[MAIN] Switching to normal playback mode (TRACK 1)", LogLevel::INFO);
            
            if (!g_autoPlaySent) {
                g_autoPlaySent = true;
                bt1036_play();
                cdc_setPlayState(CdcPlayState::PLAYING);
                btWebUI_log("[MAIN] Auto-play sent", LogLevel::INFO);
            }
        }
    }
    
    if (g_displayMode == DisplayMode::NORMAL_PLAYBACK) {
        TrackInfo ti = bt1036_getTrackInfo();
        if (ti.valid && ti.elapsedSec > 0) {
            uint8_t mins = ti.elapsedSec / 60;
            uint8_t secs = ti.elapsedSec % 60;
            cdc_setPlayTime(mins, secs);
        }
    }
}