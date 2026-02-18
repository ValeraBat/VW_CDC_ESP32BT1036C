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
 *   CD6 = Toggle WiFi ON/OFF
 *   SCAN = Hangup call
 *   MIX = Answer call
 *   <</>>	= Prev/Next track
 * 
 * Track Display Status:
 *   TRACK 80 = Waiting for BT connection
 *   TRACK 10 = Just connected (5 sec)
 *   TRACK 1+ = Normal playback with time from BT
 *   TRACK 60 = WiFi OFF
 *   TRACK 61 = WiFi ON
 */

// ============================================================================
// FIRMWARE VERSION
// ============================================================================
const char* FW_VERSION = "v1.3";

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <esp_log.h>
#include <Preferences.h> // For persisting WiFi state in NVS
#include <WiFi.h>
#include <cstdarg>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "vw_cdc.h"
#include "bt1036_at.h"
#include "bt_webui.h"

// ============================================================================
// CONSTANTS
// ============================================================================
const uint32_t DEBOUNCE_MS = 300;
const uint32_t MUTEX_WAIT_MS = 100;

// Track numbers used for status display
struct DisplayTracks {
    static const uint8_t WAITING_FOR_BT = 88; // Waiting for connection
    static const uint8_t WIFI_OFF       = 60; // WiFi Disabled
    static const uint8_t WIFI_ON        = 61; // WiFi Enabled
    static const uint8_t MUTE_ON        = 33; // Mute Active
    static const uint8_t PAIRING        = 44; // Pairing Mode
    static const uint8_t DISCONNECTED   = 55; // Manually Disconnected
    static const uint8_t CLEARED_PAIRED = 54; // Cleared Paired List
};

// FreeRTOS Task settings
const uint32_t CDC_TASK_STACK_SIZE = 4096;
const uint32_t BT_TASK_STACK_SIZE = 4096;
const uint32_t WEBUI_TASK_STACK_SIZE = 8192;
const UBaseType_t CDC_TASK_PRIORITY = 3;   // Highest priority for real-time SPI
const UBaseType_t BT_TASK_PRIORITY = 2;    // Medium priority for BT commands
const UBaseType_t WEBUI_TASK_PRIORITY = 1; // Lowest priority for Web UI

// ============================================================================
// PIN CONFIGURATION (ESP32)
// ============================================================================
static const uint8_t BT_RX_PIN = 16;  // ESP RX ← BT1036 TX (UART)
static const uint8_t BT_TX_PIN = 17;  // ESP TX → BT1036 RX (UART)

static const uint8_t CDC_SCK_PIN  = 18;  // VSPI CLK → VW Radio
static const uint8_t CDC_MISO_PIN = -1;  // Not used (radio doesn't send SPI data back, needed for SPI initialization)
static const uint8_t CDC_MOSI_PIN = 23;  // VSPI MOSI → VW Radio
static const int8_t  CDC_SS_PIN   = -1;  // Not used (single device)
static const uint8_t CDC_NEC_PIN  =  4;  // VW DataOut ← Radio (button commands)
static const uint8_t STATUS_LED_PIN = 2;   // Blue LED on board

// ============================================================================
// WIFI STATE
// ============================================================================
static bool g_wifiEnabled = true;
static Preferences g_prefs;
static QueueHandle_t g_buttonQueue = NULL; // Button press queue (ISR -> loop)
static SemaphoreHandle_t g_cdcMutex = NULL;      // Mutex for CDC access
static SemaphoreHandle_t g_btMutex = NULL;       // Mutex for BT access

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

// ============================================================================
// DISPLAY MODE STATE MACHINE
// ============================================================================
enum class DisplayMode {
    WAITING_FOR_BT,      // Waiting for connection (TRACK 88)
    NORMAL_PLAYBACK,     // Normal mode - shows track/time from BT
    MUTE_DISPLAY,        // Mute active (TRACK 33)
    PAIRING_MODE,        // Explicit pairing mode (TRACK 44)
    MANUAL_DISCONNECT    // Explicit disconnect (TRACK 55)
};

static DisplayMode g_displayMode = DisplayMode::WAITING_FOR_BT;
static BTConnState g_lastBtState = BTConnState::DISCONNECTED;
static bool g_autoPlaySent = false;                  // Auto-play command already sent
static bool g_isPairingMode = false;                 // true = waiting for NEW device (CD4)
static bool g_isManualDisconnect = false;            // true = manually disconnected (CD5)
static bool g_waitingForAvrcp = false;               // true = connected but waiting for AVRCP ready to play
static uint8_t g_prevTrackBeforeMute = 1;            // To restore track after un-Mute

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static int log_vprintf_null(const char * /*fmt*/, va_list /*args*/) {
    return 0;
}

static void silenceSerialMonitor() {
    Serial.setDebugOutput(false);
    esp_log_level_set("*", ESP_LOG_NONE);
    esp_log_set_vprintf(&log_vprintf_null);
}

static void updateStatusLed() {
    static uint32_t lastToggleMs = 0;
    static bool ledOn = false;

    if (!g_wifiEnabled) {
        ledOn = false;
        digitalWrite(STATUS_LED_PIN, LOW);
        return;
    }

    if (WiFi.status() == WL_CONNECTED) {
        ledOn = true;
        digitalWrite(STATUS_LED_PIN, HIGH);
        return;
    }

    // WiFi enabled, but not connected: slow blink
    uint32_t now = millis();
    if (now - lastToggleMs >= 500) {
        lastToggleMs = now;
        ledOn = !ledOn;
        digitalWrite(STATUS_LED_PIN, ledOn ? HIGH : LOW);
    }
}

/** Toggle WiFi ON/OFF, save to NVS, and reboot */
static void toggleWiFi() {
    g_wifiEnabled = !g_wifiEnabled;
    g_prefs.begin("sys_config", false);
    g_prefs.putBool("wifi_on", g_wifiEnabled);
    g_prefs.end();
    
    String msg = String("[MAIN] WiFi turned ") + (g_wifiEnabled ? "ON" : "OFF") + ". Rebooting to apply...";
    btWebUI_log(msg, LogLevel::INFO);
    
    // Show status on the radio display
    if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
        cdc_setDiscTrack(g_currentDisc, g_wifiEnabled ? DisplayTracks::WIFI_ON : DisplayTracks::WIFI_OFF);
        xSemaphoreGive(g_cdcMutex);
    }
    
    delay(2000); // Allow time for the display to show the status
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
    const bool canSend = bt1036_isHfpAudioActive() || bt1036_isHfpCallInProgress();
    if (canSend) {
        if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            bt1036_setMicMute(g_hfpMuted);
            xSemaphoreGive(g_btMutex);
        }
        
        // --- Mute Display Logic ---
        if (g_hfpMuted) {
            // Enter MUTE display mode (Track 33)
            g_prevTrackBeforeMute = g_currentTrack; // Save current
            g_displayMode = DisplayMode::MUTE_DISPLAY;
            g_currentTrack = DisplayTracks::MUTE_ON;
        } else {
            // Restore previous mode
            g_displayMode = DisplayMode::NORMAL_PLAYBACK;
            g_currentTrack = g_prevTrackBeforeMute;
        }
        
        if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
            cdc_setDiscTrack(g_currentDisc, g_currentTrack);
            xSemaphoreGive(g_cdcMutex);
        }
        // --------------------------

        btWebUI_log(String("[MAIN] HFP mic mute: ") + (g_hfpMuted ? "ON" : "OFF"), LogLevel::INFO);
    } else {
        btWebUI_log(String("[MAIN] HFP mic mute toggled (no call active): ") + (g_hfpMuted ? "ON" : "OFF"), LogLevel::DEBUG);
    }
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
        case CdcButton::UNKNOWN:       return "UNKNOWN";
        default:                       return "???";
    }
}

/**
 * @brief Button press handler, CALLED FROM ISR.
 * Must be as fast as possible. Simply enqueues the button.
 */
static void onCdcButton(CdcButton btn) {
    xQueueSendFromISR(g_buttonQueue, &btn, NULL);
}

/**
 * @brief Main button processing logic. Called from the main loop.
 */
static void handleButtonPress(CdcButton btn) {
    uint32_t now = millis();
    String logMsg;

    // --- Debounce Filter ---
    if (btn == g_lastButton && (now - g_lastButtonTime < DEBOUNCE_MS)) {
        return; 
    }
    g_lastButton = btn;
    g_lastButtonTime = now;

    const char* btnName = getButtonName(btn);

    switch (btn) {
        case CdcButton::NEXT_TRACK:
            if (g_displayMode != DisplayMode::NORMAL_PLAYBACK) {
                g_displayMode = DisplayMode::NORMAL_PLAYBACK;
                g_currentTrack = 1;
            }
            bumpTrackForward();
            if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                cdc_setDiscTrack(g_currentDisc, g_currentTrack);
                xSemaphoreGive(g_cdcMutex);
            }
            if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                bt1036_nextTrack();
                xSemaphoreGive(g_btMutex);
            }
            logMsg = String("[BTN] ") + btnName + " → BT: Next, Track " + String(g_currentTrack);
            break;

        case CdcButton::PREV_TRACK:
            if (g_displayMode != DisplayMode::NORMAL_PLAYBACK) {
                g_displayMode = DisplayMode::NORMAL_PLAYBACK;
                g_currentTrack = 2;
            }
            bumpTrackBackward();
             if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                cdc_setDiscTrack(g_currentDisc, g_currentTrack);
                xSemaphoreGive(g_cdcMutex);
            }
            if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                bt1036_prevTrack();
                xSemaphoreGive(g_btMutex);
            }
            logMsg = String("[BTN] ") + btnName + " → BT: Prev, Track " + String(g_currentTrack);
            break;

        case CdcButton::PLAY_PAUSE:
            g_isPlaying = !g_isPlaying;
            if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                if (g_isPlaying) bt1036_play();
                else bt1036_pause();
                xSemaphoreGive(g_btMutex);
            }
            if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                cdc_setPlayState(g_isPlaying ? CdcPlayState::PLAYING : CdcPlayState::PAUSED);
                xSemaphoreGive(g_cdcMutex);
            }
            logMsg = String("[BTN] ") + btnName + " → BT: " + (g_isPlaying ? "Play" : "Pause");
            break;

        case CdcButton::STOP:
            g_isPlaying = false;
            if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                bt1036_stop();
                xSemaphoreGive(g_btMutex);
            }
            if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                cdc_setPlayState(CdcPlayState::STOPPED);
                xSemaphoreGive(g_cdcMutex);
            }
            logMsg = String("[BTN] ") + btnName + " → BT: Stop";
            break;

        case CdcButton::NEXT_DISC:
        case CdcButton::PREV_DISC:
            logMsg = String("[BTN] ") + btnName + " → (ignored)";
            break;

        case CdcButton::DISC_1: {
            g_isPlaying = !g_isPlaying;
            if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                if (g_isPlaying) bt1036_play();
                else bt1036_pause();
                xSemaphoreGive(g_btMutex);
            }
            if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                cdc_setPlayState(g_isPlaying ? CdcPlayState::PLAYING : CdcPlayState::PAUSED);
                xSemaphoreGive(g_cdcMutex);
            }
            logMsg = String("[BTN] ") + btnName + " → BT: " + (g_isPlaying ? "Play" : "Pause");
            break;
        }

        case CdcButton::DISC_2:
            if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                bt1036_stop();
                xSemaphoreGive(g_btMutex);
            }
            if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                cdc_setPlayState(CdcPlayState::STOPPED);
                xSemaphoreGive(g_cdcMutex);
            }
            logMsg = String("[BTN] ") + btnName + " → BT: Stop";
            break;

        case CdcButton::DISC_3:
            toggleHfpMute(); // Handles display update internally
            logMsg = String("[BTN] ") + btnName + " → Mic Mute: " + (g_hfpMuted ? "ON" : "OFF");
            break;

        case CdcButton::DISC_4:
            if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                bt1036_enterPairingMode(); // AT+CAIR
                xSemaphoreGive(g_btMutex);
            }
            g_displayMode = DisplayMode::PAIRING_MODE;
            g_currentTrack = DisplayTracks::PAIRING; // 44
            if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                cdc_setDiscTrack(g_currentDisc, g_currentTrack);
                xSemaphoreGive(g_cdcMutex);
            }
            logMsg = String("[BTN] ") + btnName + " → BT: Pairing Mode (TRACK " + String(DisplayTracks::PAIRING) + ")";
            break;

        case CdcButton::DISC_5: {
            static uint32_t lastCd5Press = 0;
            if (now - lastCd5Press < 1000) { // Double press < 1 sec
                if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                    bt1036_clearPairedDevices(); // Uses AT+DELPD instead of AT+C
                    g_currentTrack = DisplayTracks::CLEARED_PAIRED; // TRACK 54
                    xSemaphoreGive(g_btMutex);
                }
                
                if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                    cdc_setDiscTrack(g_currentDisc, g_currentTrack);
                    xSemaphoreGive(g_cdcMutex);
                }
                btWebUI_log("[BTN] CD5 (Double) → Clear Paired List (TRACK 54)", LogLevel::INFO);
                lastCd5Press = 0; // Reset
            } else {
                // Single press triggers disconnect logic, but we only SEND commands if connected
                lastCd5Press = now;
                g_isManualDisconnect = true;

                if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                    // Only send disconnect if we think we are connected, to avoid ERR logs
                    BTConnState state = bt1036_getState();
                    if (state != BTConnState::DISCONNECTED) {
                        bt1036_disconnect(); // AT+A2DPDISC
                        bt1036_hfpDisconnect(); // AT+HFPDISC
                    }
                    xSemaphoreGive(g_btMutex);
                }
                
                g_displayMode = DisplayMode::MANUAL_DISCONNECT;
                g_currentTrack = DisplayTracks::DISCONNECTED; // 55
                
                if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                    cdc_setDiscTrack(g_currentDisc, g_currentTrack);
                    xSemaphoreGive(g_cdcMutex);
                }
                String logMsg = String("[BTN] ") + btnName + " → BT: Disconnect (TRACK 55)";
                btWebUI_log(logMsg, LogLevel::INFO);
            }
            break;
        }

        case CdcButton::DISC_6:
            // Display feedback is handled inside toggleWiFi()
            toggleWiFi();
            logMsg = String("[BTN] ") + btnName + " → Toggle WiFi";
            break;
            
        case CdcButton::SCAN_TOGGLE:
            if (bt1036_isHfpCallInProgress() || bt1036_isHfpAudioActive()) {
                if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                    bt1036_hangupCall();
                    xSemaphoreGive(g_btMutex);
                }
            } else {
                btWebUI_log("[BTN] SCAN → HFP hangup ignored (no call)", LogLevel::DEBUG);
            }
            if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                cdc_setScan(true);
                xSemaphoreGive(g_cdcMutex);
            }
            g_scanResetTime = millis() + 500;
            logMsg = String("[BTN] ") + btnName + " → HFP: Hangup";
            break;

        case CdcButton::RANDOM_TOGGLE:
            if (bt1036_isHfpCallIncoming()) {
                if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                    bt1036_answerCall();
                    xSemaphoreGive(g_btMutex);
                }
            } else {
                btWebUI_log("[BTN] RANDOM/MIX → HFP answer ignored (no incoming call)", LogLevel::DEBUG);
            }
            if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                cdc_setRandom(true);
                xSemaphoreGive(g_cdcMutex);
            }
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
// TASKS
// ============================================================================

void cdc_task(void *parameter) {
    btWebUI_log("[TASK] CDC task started", LogLevel::DEBUG);
    for (;;) {
        cdc_loop();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void bt_task(void *parameter) {
    btWebUI_log("[TASK] BT task started", LogLevel::DEBUG);
    for (;;) {
        bt1036_loop();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void webui_task(void *parameter) {
    btWebUI_log("[TASK] WebUI task started", LogLevel::DEBUG);
    for (;;) {
        if (g_wifiEnabled) {
            btWebUI_loop();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    silenceSerialMonitor();

    esp_task_wdt_init(15, true); 
    esp_task_wdt_add(NULL);

    g_prefs.begin("sys_config", true);
    g_wifiEnabled = g_prefs.getBool("wifi_on", true);
    g_prefs.end();

    if (g_wifiEnabled) {
        btWebUI_init();
    }
    
    // Create queue and mutexes BEFORE initializing modules
    g_buttonQueue = xQueueCreate(10, sizeof(CdcButton)); 
    g_cdcMutex = xSemaphoreCreateMutex();
    g_btMutex = xSemaphoreCreateMutex();

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

    // --- Status LED ---
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, g_wifiEnabled ? HIGH : LOW);

    // ESP32-S2 Arduino core does not expose Serial2; Serial1 works on both ESP32 and ESP32-S2
    // because bt1036_init() remaps RX/TX pins in begin().
    bt1036_init(Serial1, BT_RX_PIN, BT_TX_PIN);

    g_currentTrack = DisplayTracks::WAITING_FOR_BT;
    g_displayMode = DisplayMode::WAITING_FOR_BT;
    cdc_setDiscTrack(g_currentDisc, g_currentTrack);
    cdc_setPlayState(CdcPlayState::PLAYING);
    cdc_setRandom(false);
    cdc_setScan(false);
    cdc_init(CDC_SCK_PIN, CDC_MISO_PIN, CDC_MOSI_PIN, CDC_SS_PIN, CDC_NEC_PIN, onCdcButton);

    btWebUI_log("[MAIN] Init complete. Starting tasks...", LogLevel::INFO);
    
    xTaskCreate(cdc_task, "CDCTask", CDC_TASK_STACK_SIZE, NULL, CDC_TASK_PRIORITY, NULL);
    xTaskCreate(bt_task, "BTTask", BT_TASK_STACK_SIZE, NULL, BT_TASK_PRIORITY, NULL);
    xTaskCreate(webui_task, "WebUITask", WEBUI_TASK_STACK_SIZE, NULL, WEBUI_TASK_PRIORITY, NULL);
}

// ============================================================================
// MAIN LOOP (runs as a low-priority background task)
// ============================================================================
void loop() {
    esp_task_wdt_reset();

    updateStatusLed();

    // Check button queue for pending presses
    CdcButton btn;
    if (xQueueReceive(g_buttonQueue, &btn, (TickType_t)0) == pdPASS) {
        handleButtonPress(btn);
    }
    
    // Reset SCAN/MIX indicators on the radio display
    if (g_scanResetTime > 0 && millis() > g_scanResetTime) {
        g_scanResetTime = 0;
        if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
            cdc_setScan(false);
            xSemaphoreGive(g_cdcMutex);
        }
    }
    
    if (g_mixResetTime > 0 && millis() > g_mixResetTime) {
        g_mixResetTime = 0;
        if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
            cdc_setRandom(false);
            cdc_resetModeFF();
            xSemaphoreGive(g_cdcMutex);
        }
    }
    
    // --- Main BT connection state machine ---
    BTConnState currentBtState;
    if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
        currentBtState = bt1036_getState();
        xSemaphoreGive(g_btMutex);
    }
    
    // Event: just connected
    if (g_lastBtState == BTConnState::DISCONNECTED && 
        (currentBtState == BTConnState::CONNECTED_IDLE || 
         currentBtState == BTConnState::PLAYING || 
         currentBtState == BTConnState::PAUSED)) {
        
        if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
            bt1036_setVolume(15);
            xSemaphoreGive(g_btMutex);
        }
        btWebUI_log("[MAIN] Set BT volume to MAX (15)", LogLevel::INFO);

        if (g_isPairingMode) { // New device connected (via Pair)
            // User requested immediate switch to normal playback (Track 1)
            g_displayMode = DisplayMode::NORMAL_PLAYBACK;
            g_currentTrack = 1;
            g_isPairingMode = false;

            if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                cdc_setDiscTrack(g_currentDisc, g_currentTrack);
                cdc_setPlayState(CdcPlayState::PLAYING);
                xSemaphoreGive(g_cdcMutex);
            }
            
            g_autoPlaySent = false;
            btWebUI_log("[MAIN] New device connected! Immediate switch to TRACK 1", LogLevel::INFO);
            
            // Trigger auto-play logic DELAYED (wait for AVRCP)
            if (!g_autoPlaySent) {
                g_waitingForAvrcp = true; 
                btWebUI_log("[MAIN] Connected (Pairing). Waiting for AVRCP ready...", LogLevel::INFO);
            }

        } else { // Auto-reconnect (existing device)
            // If we came from manual disconnect, maybe we should treat it as new connection?
            // But usually auto-reconnect happens on boot or range return.
            g_displayMode = DisplayMode::NORMAL_PLAYBACK;
            g_currentTrack = 1;
            g_isPlaying = true;
            if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                cdc_setDiscTrack(g_currentDisc, g_currentTrack);
                cdc_setPlayState(CdcPlayState::PLAYING);
                xSemaphoreGive(g_cdcMutex);
            }
            
            if (!g_autoPlaySent) {
                g_waitingForAvrcp = true;
                btWebUI_log("[MAIN] Connected (Auto). Waiting for AVRCP ready...", LogLevel::INFO);
            }
        }
    }
    
    // Event: just disconnected
    if (currentBtState == BTConnState::DISCONNECTED && 
        g_lastBtState != BTConnState::DISCONNECTED) {
        g_displayMode = DisplayMode::WAITING_FOR_BT;
        g_currentTrack = DisplayTracks::WAITING_FOR_BT;
        if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
            cdc_setDiscTrack(g_currentDisc, g_currentTrack);
            xSemaphoreGive(g_cdcMutex);
        }
        g_hfpMuted = false; // Reset mute state
        g_autoPlaySent = false;
        btWebUI_log(String("[MAIN] BT Disconnected. Showing TRACK ") + String(DisplayTracks::WAITING_FOR_BT), LogLevel::INFO);
    }
    
    g_lastBtState = currentBtState;

    // --- Check for AVRCP Ready to Trigger Auto-Play ---
    if (g_waitingForAvrcp) {
        bool ready = false;
        if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
             ready = bt1036_isAvrcpReady();
             xSemaphoreGive(g_btMutex);
        }

        if (ready) {
            btWebUI_log("[MAIN] AVRCP is READY. Sending Auto-Play.", LogLevel::INFO);
            g_waitingForAvrcp = false;
            g_autoPlaySent = true;
            
            if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                 bt1036_play();
                 xSemaphoreGive(g_btMutex);
            }
        }
    }
    
    // --- Display mode logic ---
    
    if (g_displayMode == DisplayMode::NORMAL_PLAYBACK) {
        
        // Check for track change
        bool trackChanged = false;
        if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
             trackChanged = bt1036_checkTrackChanged();
             xSemaphoreGive(g_btMutex);
        }
        
        if (trackChanged) {
            bumpTrackForward();
            if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                cdc_setDiscTrack(g_currentDisc, g_currentTrack);
                xSemaphoreGive(g_cdcMutex);
            }
            btWebUI_log("[MAIN] Track changed on phone -> increments track", LogLevel::INFO);
        }

        TrackInfo ti;
        if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
            ti = bt1036_getTrackInfo();
            xSemaphoreGive(g_btMutex);
        }
        if (ti.valid) {  // removed ti.elapsedSec > 0 check so 0:00 displays
            uint8_t mins = ti.elapsedSec / 60;
            uint8_t secs = ti.elapsedSec % 60;
            if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                cdc_setPlayTime(mins, secs);
                xSemaphoreGive(g_cdcMutex);
            }
        }
    }

    // Small delay to yield CPU to other tasks
    vTaskDelay(pdMS_TO_TICKS(10));
}