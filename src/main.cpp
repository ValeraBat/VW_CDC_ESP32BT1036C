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
 *   SCAN = Hangup call
 *   MIX = Answer call
 *   <</>>	= Prev/Next track
 * 
 * Track Display Status:
 *   TRACK 80 = Waiting for BT connection
 *   TRACK 10 = Just connected (5 sec)
 *   TRACK 1+ = Normal playback with time from BT
 */

#include <Arduino.h>

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
// GLOBAL STATE
// ============================================================================
static uint8_t g_currentDisc  = 1;   // Current CD number (always 1)
static uint8_t g_currentTrack = 1;   // Current track number (1-99, or 80/10 for status)
static bool g_hfpMuted = false;      // HFP microphone mute state
static bool g_isPlaying = false;     // Playback state for toggle logic

// Timers for SCAN/MIX indicator pulse (500ms on, then off)
static uint32_t g_scanResetTime = 0;
static uint32_t g_mixResetTime = 0;

// ============================================================================
// DISPLAY MODE STATE MACHINE
// Shows BT connection status via track number on radio display:
//   TRACK 80 = Waiting for BT connection (pairing mode)
//   TRACK 10 = Device just connected (shown for 5 seconds)
//   TRACK 1+ = Normal playback mode (time from BT module)
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
// Called by CDC decoder when a button press is detected from the radio
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
        case CdcButton::UNKNOWN:       return "UNKNOWN";
        default:                       return "???";
    }
}

static void onCdcButton(CdcButton btn) {
    const char* btnName = getButtonName(btn);
    String logMsg;  // Для WebUI

    switch (btn) {

        // ---- Треки ----
        case CdcButton::NEXT_TRACK:
            // Переключаем на нормальный режим если ещё не там
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
            // Переключаем на нормальный режим если ещё не там
            if (g_displayMode != DisplayMode::NORMAL_PLAYBACK) {
                g_displayMode = DisplayMode::NORMAL_PLAYBACK;
                g_currentTrack = 2;  // Чтобы после bumpBackward было 1
            }
            bumpTrackBackward();
            cdc_setDiscTrack(g_currentDisc, g_currentTrack);
            bt1036_prevTrack();
            logMsg = String("[BTN] ") + btnName + " → BT: Prev, Track " + String(g_currentTrack);
            break;

        // ---- Стандартные кнопки (если магнитола их отправит) ----
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
            // Магнитола не шлёт эти команды в CDC режиме
            logMsg = String("[BTN] ") + btnName + " → (ignored)";
            break;

        // ---- КНОПКИ CD1..CD3 ПЕРЕНАЗНАЧЕНИЕ ----

        case CdcButton::DISC_1: {
            // CD1 = Play/Pause toggle (локальный флаг, как CD3)
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
            // CD2 = Stop
            bt1036_stop();
            cdc_setPlayState(CdcPlayState::STOPPED);
            logMsg = String("[BTN] ") + btnName + " → BT: Stop";
            break;

        case CdcButton::DISC_3:
            // CD3 = HFP mic mute toggle
            toggleHfpMute();
            logMsg = String("[BTN] ") + btnName + " → Mic Mute: " + (g_hfpMuted ? "ON" : "OFF");
            break;

        // ---- CD4 = Режим сопряжения ----
        case CdcButton::DISC_4:
            bt1036_enterPairingMode();
            g_displayMode = DisplayMode::WAITING_FOR_BT;
            g_isPairingMode = true;  // Ждём новое устройство
            g_currentTrack = 80;  // Показываем TRACK 80
            cdc_setDiscTrack(g_currentDisc, g_currentTrack);
            logMsg = String("[BTN] ") + btnName + " → BT: Pairing Mode (TRACK 80)";
            break;

        // ---- CD5 = Отключить текущее устройство ----
        case CdcButton::DISC_5:
            bt1036_disconnect();
            bt1036_hfpDisconnect();
            g_displayMode = DisplayMode::WAITING_FOR_BT;
            g_currentTrack = 80;  // Показываем TRACK 80
            cdc_setDiscTrack(g_currentDisc, g_currentTrack);
            logMsg = String("[BTN] ") + btnName + " → BT: Disconnect";
            break;

        // ---- CD6 = Очистить список устройств ----
        case CdcButton::DISC_6:
            bt1036_clearPairedDevices();
            g_displayMode = DisplayMode::WAITING_FOR_BT;
            g_isPairingMode = true;  // Ждём новое устройство
            g_currentTrack = 80;  // Показываем TRACK 80
            cdc_setDiscTrack(g_currentDisc, g_currentTrack);
            logMsg = String("[BTN] ") + btnName + " → BT: Clear Paired Devices";
            break;

        // ---- SCAN = Hangup Call ----
        case CdcButton::SCAN_TOGGLE:
            bt1036_hangupCall();
            // Пульс: 0xD0 → через 500мс сброс в 0x00
            cdc_setScan(true);
            g_scanResetTime = millis() + 500;
            logMsg = String("[BTN] ") + btnName + " → HFP: Hangup";
            break;

        // ---- MIX = Answer Call ----
        case CdcButton::RANDOM_TOGGLE:
            bt1036_answerCall();
            // Пульс: 0x04 → через 500мс сброс в 0xFF
            cdc_setRandom(true);
            g_mixResetTime = millis() + 500;
            logMsg = String("[BTN] ") + btnName + " → HFP: Answer Call";
            break;

        case CdcButton::UNKNOWN:
        default:
            logMsg = String("[BTN] ") + btnName + " \u2192 (no action)";
            break;
    }
    
    // Единый лог - в btWebUI_log (Serial + WebSocket)
    btWebUI_log(logMsg, LogLevel::INFO);
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(200);
    
    // Логируем причину последней перезагрузки
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
    
    // Serial.println здесь OK - btWebUI ещё не инициализирован
    Serial.println();
    Serial.print("[MAIN] Reset reason: "); Serial.println(reasonStr);
    Serial.println("[MAIN] VW CDC + BT1036 emulator start");

    // BT1036 - используем Serial2 для GPIO16/17 (модуль пока не подключен)
    bt1036_init(Serial2, BT_RX_PIN, BT_TX_PIN);

    // CDC - начинаем с TRACK 80 (ожидание BT)
    g_currentTrack = 80;
    g_displayMode = DisplayMode::WAITING_FOR_BT;
    cdc_setDiscTrack(g_currentDisc, g_currentTrack);  // TRACK 80 = ждём подключения
    cdc_setPlayState(CdcPlayState::PLAYING);          // Принудительно PLAYING
    cdc_setRandom(false);
    cdc_setScan(false);
    cdc_init(CDC_SCK_PIN, CDC_MISO_PIN, CDC_MOSI_PIN, CDC_SS_PIN, CDC_NEC_PIN, onCdcButton);  // Потом инициализируем

    // Web UI
    btWebUI_init();

    btWebUI_log("[MAIN] Init complete.", LogLevel::INFO);
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
    // Process all subsystems
    bt1036_loop();
    cdc_loop();
    btWebUI_loop();
    
    // Reset SCAN indicator after 500ms pulse
    if (g_scanResetTime > 0 && millis() > g_scanResetTime) {
        g_scanResetTime = 0;
        cdc_setScan(false);
    }
    
    // Reset MIX indicator after 500ms pulse
    if (g_mixResetTime > 0 && millis() > g_mixResetTime) {
        g_mixResetTime = 0;
        cdc_setRandom(false);
        cdc_resetModeFF();
    }
    
    // ========== BT CONNECTION STATUS HANDLING ==========
    BTConnState currentBtState = bt1036_getState();
    
    // Detect transition: DISCONNECTED -> CONNECTED
    if (g_lastBtState == BTConnState::DISCONNECTED && 
        (currentBtState == BTConnState::CONNECTED_IDLE || 
         currentBtState == BTConnState::PLAYING || 
         currentBtState == BTConnState::PAUSED)) {
        
        if (g_isPairingMode) {
            // NEW device connected (after CD4/CD6) - show TRACK 10 for 5 sec
            g_displayMode = DisplayMode::JUST_CONNECTED;
            g_connectedShowTime = millis();
            g_currentTrack = 10;
            cdc_setDiscTrack(g_currentDisc, g_currentTrack);
            g_autoPlaySent = false;
            btWebUI_log("[MAIN] New device connected! Showing TRACK 10 for 5 sec", LogLevel::INFO);
        } else {
            // AUTO-RECONNECT to known device - instant play
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
    
    // Detect DISCONNECTION
    if (currentBtState == BTConnState::DISCONNECTED && 
        g_lastBtState != BTConnState::DISCONNECTED) {
        g_displayMode = DisplayMode::WAITING_FOR_BT;
        g_currentTrack = 80;
        cdc_setDiscTrack(g_currentDisc, g_currentTrack);
        g_autoPlaySent = false;
        btWebUI_log("[MAIN] BT Disconnected. Showing TRACK 80", LogLevel::INFO);
    }
    
    g_lastBtState = currentBtState;
    
    // Transition: JUST_CONNECTED -> NORMAL_PLAYBACK after 5 seconds
    if (g_displayMode == DisplayMode::JUST_CONNECTED) {
        if (millis() - g_connectedShowTime > 5000) {
            g_displayMode = DisplayMode::NORMAL_PLAYBACK;
            g_currentTrack = 1;
            g_isPlaying = true;
            g_isPairingMode = false;  // Reset pairing mode flag
            cdc_setDiscTrack(g_currentDisc, g_currentTrack);
            btWebUI_log("[MAIN] Switching to normal playback mode (TRACK 1)", LogLevel::INFO);
            
            // Send auto-play command
            if (!g_autoPlaySent) {
                g_autoPlaySent = true;
                bt1036_play();
                cdc_setPlayState(CdcPlayState::PLAYING);
                btWebUI_log("[MAIN] Auto-play sent", LogLevel::INFO);
            }
        }
    }
    
    // In normal playback mode, update time from BT module
    if (g_displayMode == DisplayMode::NORMAL_PLAYBACK) {
        TrackInfo ti = bt1036_getTrackInfo();
        if (ti.valid && ti.elapsedSec > 0) {
            uint8_t mins = ti.elapsedSec / 60;
            uint8_t secs = ti.elapsedSec % 60;
            cdc_setPlayTime(mins, secs);
        }
    }
}