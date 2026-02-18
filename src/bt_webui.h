/**
 * @file bt_webui.h
 * @brief Web UI for VW CDC Bluetooth Emulator
 * 
 * Provides WiFi AP + WebServer + WebSocket interface for:
 *   - Real-time log viewing (BT, CDC, system)
 *   - BT1036 configuration
 *   - WiFi client setup
 *   - OTA firmware updates (ElegantOTA)
 * 
 * Endpoints:
 *   /        - Main control page
 *   /bt      - BT debug logs
 *   /cdc     - CDC debug logs
 *   /logs    - All logs combined
 *   /wifi    - WiFi configuration
 *   /update  - OTA update (ElegantOTA)
 */

#pragma once
#include <Arduino.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include "bt1036_at.h"

extern WebServer webServer;
extern bool g_debugMode;  // Debug mode flag

// Log levels
enum class LogLevel : uint8_t {
    INFO,    // Important events (always visible)
    DEBUG,   // Debug messages (only when g_debugMode is on)
    VERBOSE  // Detailed logs (only when g_debugMode, not stored in ring buffer)
};

void btWebUI_init();
void btWebUI_loop();
void btWebUI_log(const String &line);  // backward compat, = INFO
void btWebUI_log(const String &line, LogLevel level);
void btWebUI_broadcastCdcRaw(const String &line);
void btWebUI_setDebug(bool on);