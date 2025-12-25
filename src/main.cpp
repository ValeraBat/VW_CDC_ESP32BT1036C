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
const uint32_t DOUBLE_PRESS_WINDOW_MS = 500;
const uint32_t MUTEX_WAIT_MS = 100;

// Track numbers used for status display
struct DisplayTracks {
    static const uint8_t WAITING_FOR_BT = 80;
    static const uint8_t JUST_CONNECTED = 10;
    static const uint8_t WIFI_OFF = 90;
    static const uint8_t WIFI_ON = 91;
};

// FreeRTOS Task settings
const uint32_t CDC_TASK_STACK_SIZE = 4096;
const uint32_t BT_TASK_STACK_SIZE = 4096;
const uint32_t WEBUI_TASK_STACK_SIZE = 4096;
const UBaseType_t CDC_TASK_PRIORITY = 3;   // Highest priority for real-time SPI
const UBaseType_t BT_TASK_PRIORITY = 2;    // Medium priority for BT commands
const UBaseType_t WEBUI_TASK_PRIORITY = 1; // Lowest priority for Web UI

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
static QueueHandle_t g_buttonQueue = NULL; // Очередь для нажатий кнопок
static SemaphoreHandle_t g_cdcMutex = NULL;      // Мьютекс для CDC
static SemaphoreHandle_t g_btMutex = NULL;       // Мьютекс для BT

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
    if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
        cdc_setDiscTrack(1, g_wifiEnabled ? DisplayTracks::WIFI_ON : DisplayTracks::WIFI_OFF);
        xSemaphoreGive(g_cdcMutex);
    }
    
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
    if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        bt1036_setMicMute(g_hfpMuted);
        xSemaphoreGive(g_btMutex);
    }
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

/**
 * @brief Обработчик нажатия кнопки, ВЫЗЫВАЕМЫЙ ИЗ ПРЕРЫВАНИЯ (ISR).
 * Должен быть максимально быстрым. Просто кладет кнопку в очередь.
 */
static void onCdcButton(CdcButton btn) {
    xQueueSendFromISR(g_buttonQueue, &btn, NULL);
}

/**
 * @brief ОСНОВНАЯ логика обработки кнопок. Вызывается из главного цикла.
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

    // --- Double Press Logic for CD6 ---
    if (btn == CdcButton::DISC_6) {
        if (now - g_cd6PressTime < DOUBLE_PRESS_WINDOW_MS) {
            g_cd6PressTime = 0; 
            // Рекурсивный вызов заменяем прямым
            handleButtonPress(CdcButton::DISC_6_DOUBLE_PRESS);
            return;
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
            logMsg = String("[BTN ") + btnName + " → BT: Prev, Track " + String(g_currentTrack);
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
            toggleHfpMute();
            logMsg = String("[BTN] ") + btnName + " → Mic Mute: " + (g_hfpMuted ? "ON" : "OFF");
            break;

        case CdcButton::DISC_4:
            if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                bt1036_enterPairingMode();
                xSemaphoreGive(g_btMutex);
            }
            g_displayMode = DisplayMode::WAITING_FOR_BT;
            g_isPairingMode = true;
            g_currentTrack = DisplayTracks::WAITING_FOR_BT;
            if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                cdc_setDiscTrack(g_currentDisc, g_currentTrack);
                xSemaphoreGive(g_cdcMutex);
            }
            logMsg = String("[BTN] ") + btnName + " → BT: Pairing Mode (TRACK " + String(DisplayTracks::WAITING_FOR_BT) + ")";
            break;

        case CdcButton::DISC_5:
            if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                bt1036_disconnect();
                bt1036_hfpDisconnect();
                xSemaphoreGive(g_btMutex);
            }
            g_displayMode = DisplayMode::WAITING_FOR_BT;
            g_currentTrack = DisplayTracks::WAITING_FOR_BT;
            if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                cdc_setDiscTrack(g_currentDisc, g_currentTrack);
                xSemaphoreGive(g_cdcMutex);
            }
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
            if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                bt1036_hangupCall();
                xSemaphoreGive(g_btMutex);
            }
            if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                cdc_setScan(true);
                xSemaphoreGive(g_cdcMutex);
            }
            g_scanResetTime = millis() + 500;
            logMsg = String("[BTN] ") + btnName + " → HFP: Hangup";
            break;

        case CdcButton::RANDOM_TOGGLE:
            if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                bt1036_answerCall();
                xSemaphoreGive(g_btMutex);
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
    esp_task_wdt_init(15, true); 
    esp_task_wdt_add(NULL);

    g_prefs.begin("sys_config", true);
    g_wifiEnabled = g_prefs.getBool("wifi_on", true);
    g_prefs.end();

    if (g_wifiEnabled) {
        btWebUI_init();
    }
    
    // Создаем очередь и мьютексы ДО инициализации модулей
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

    bt1036_init(Serial2, BT_RX_PIN, BT_TX_PIN);

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
// MAIN LOOP (теперь это фоновая задача с низким приоритетом)
// ============================================================================
void loop() {
    esp_task_wdt_reset();

    // Проверяем очередь на наличие нажатых кнопок
    CdcButton btn;
    if (xQueueReceive(g_buttonQueue, &btn, (TickType_t)0) == pdPASS) {
        handleButtonPress(btn);
    }
    
    // Обработка одиночного нажатия CD6 с задержкой
    if (g_cd6PressTime > 0 && (millis() - g_cd6PressTime >= DOUBLE_PRESS_WINDOW_MS)) {
        g_cd6PressTime = 0; 
        
        if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
            bt1036_clearPairedDevices();
            xSemaphoreGive(g_btMutex);
        }
        g_displayMode = DisplayMode::WAITING_FOR_BT;
        g_isPairingMode = true;
        g_currentTrack = DisplayTracks::WAITING_FOR_BT;
        if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
            cdc_setDiscTrack(g_currentDisc, g_currentTrack);
            xSemaphoreGive(g_cdcMutex);
        }
        btWebUI_log("[BTN] CD6 → BT: Clear Paired Devices", LogLevel::INFO);
    }

    // Сброс индикаторов SCAN/MIX на магнитоле
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
    
    // --- Главная машина состояний по статусу BT ---
    BTConnState currentBtState;
    if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
        currentBtState = bt1036_getState();
        xSemaphoreGive(g_btMutex);
    }
    
    // Событие: только что подключились
    if (g_lastBtState == BTConnState::DISCONNECTED && 
        (currentBtState == BTConnState::CONNECTED_IDLE || 
         currentBtState == BTConnState::PLAYING || 
         currentBtState == BTConnState::PAUSED)) {
        
        if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
            bt1036_setVolume(15);
            xSemaphoreGive(g_btMutex);
        }
        btWebUI_log("[MAIN] Set BT volume to MAX (15)", LogLevel::INFO);

        if (g_isPairingMode) { // Подключилось новое устройство
            g_displayMode = DisplayMode::JUST_CONNECTED;
            g_connectedShowTime = millis();
            g_currentTrack = DisplayTracks::JUST_CONNECTED;
            if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                cdc_setDiscTrack(g_currentDisc, g_currentTrack);
                xSemaphoreGive(g_cdcMutex);
            }
            g_autoPlaySent = false;
            btWebUI_log(String("[MAIN] New device connected! Showing TRACK ") + String(DisplayTracks::JUST_CONNECTED) + " for 5 sec", LogLevel::INFO);
        } else { // Авто-реконнект
            g_displayMode = DisplayMode::NORMAL_PLAYBACK;
            g_currentTrack = 1;
            g_isPlaying = true;
            if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                cdc_setDiscTrack(g_currentDisc, g_currentTrack);
                cdc_setPlayState(CdcPlayState::PLAYING);
                xSemaphoreGive(g_cdcMutex);
            }
            
            if (!g_autoPlaySent) {
                g_autoPlaySent = true;
                if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                    bt1036_play();
                    xSemaphoreGive(g_btMutex);
                }
                btWebUI_log("[MAIN] Auto-reconnect! Instant play sent", LogLevel::INFO);
            }
        }
    }
    
    // Событие: только что отключились
    if (currentBtState == BTConnState::DISCONNECTED && 
        g_lastBtState != BTConnState::DISCONNECTED) {
        g_displayMode = DisplayMode::WAITING_FOR_BT;
        g_currentTrack = DisplayTracks::WAITING_FOR_BT;
        if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
            cdc_setDiscTrack(g_currentDisc, g_currentTrack);
            xSemaphoreGive(g_cdcMutex);
        }
        g_autoPlaySent = false;
        btWebUI_log(String("[MAIN] BT Disconnected. Showing TRACK ") + String(DisplayTracks::WAITING_FOR_BT), LogLevel::INFO);
    }
    
    g_lastBtState = currentBtState;
    
    // --- Логика режимов отображения ---
    if (g_displayMode == DisplayMode::JUST_CONNECTED) {
        if (millis() - g_connectedShowTime > 5000) {
            g_displayMode = DisplayMode::NORMAL_PLAYBACK;
            g_currentTrack = 1;
            g_isPlaying = true;
            g_isPairingMode = false;
            if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                cdc_setDiscTrack(g_currentDisc, g_currentTrack);
                xSemaphoreGive(g_cdcMutex);
            }
            btWebUI_log("[MAIN] Switching to normal playback mode (TRACK 1)", LogLevel::INFO);
            
            if (!g_autoPlaySent) {
                g_autoPlaySent = true;
                if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                    bt1036_play();
                    xSemaphoreGive(g_btMutex);
                }
                if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                    cdc_setPlayState(CdcPlayState::PLAYING);
                    xSemaphoreGive(g_cdcMutex);
                }
                btWebUI_log("[MAIN] Auto-play sent", LogLevel::INFO);
            }
        }
    }
    
    if (g_displayMode == DisplayMode::NORMAL_PLAYBACK) {
        TrackInfo ti;
        if (xSemaphoreTake(g_btMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
            ti = bt1036_getTrackInfo();
            xSemaphoreGive(g_btMutex);
        }
        if (ti.valid && ti.elapsedSec > 0) {
            uint8_t mins = ti.elapsedSec / 60;
            uint8_t secs = ti.elapsedSec % 60;
            if (xSemaphoreTake(g_cdcMutex, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE) {
                cdc_setPlayTime(mins, secs);
                xSemaphoreGive(g_cdcMutex);
            }
        }
    }

    // Небольшая задержка, чтобы дать поработать другим задачам
    vTaskDelay(pdMS_TO_TICKS(10));
}