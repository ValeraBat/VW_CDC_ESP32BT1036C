# VW CDC Bluetooth Emulator

ESP32 firmware that emulates a VW CD Changer (CDC) to integrate a BT1036C Bluetooth module with VW RNS-MFD head units. This version uses a real-time operating system (FreeRTOS) for maximum stability and responsiveness.

## Features

- **A2DP Audio Streaming** - Play music from phone via Bluetooth
- **AVRCP Controls** - Track skip, play/pause from radio buttons
- **HFP Hands-Free** - Answer/hangup calls, mic mute
- **RTOS-based Architecture** - High stability and responsiveness by running CDC, Bluetooth, and Web UI in parallel tasks.
- **Safe OTA Updates** - Peripherals are paused during firmware updates to prevent crashes.
- **Status Display** - BT connection status via track number
- **Web UI** - WiFi configuration, live logs, manual AT command console, and OTA updates.

## Hardware

| Component | Description |
|-----------|-------------|
| ESP-WROVER-KIT | ESP32 development board |
| BT1036C | Bluetooth audio module (A2DP/AVRCP/HFP) |
| VW RNS-MFD | Head unit with CDC port |

### Pinout

| Signal | ESP32 Pin | Description |
|--------|-----------|-------------|
| BT_RX | GPIO16 | ESP RX ← BT1036 TX |
| BT_TX | GPIO17 | ESP TX → BT1036 RX |
| CDC_SCK | GPIO18 | SPI Clock → Radio |
| CDC_MOSI | GPIO23 | SPI Data → Radio |
| CDC_NEC | GPIO4 | Button commands ← Radio |

## Button Mapping

Radio CD buttons are remapped to Bluetooth functions:

| Button | Function |
|--------|----------|
| **CD1** | Play/Pause toggle |
| **CD2** | Stop |
| **CD3** | Mic Mute toggle |
| **CD4** | Enter Pairing Mode |
| **CD5** | Disconnect device |
| **CD6** | Clear paired devices |
| **CD6 (Double Press)** | Toggle WiFi & Reboot |
| **SCAN** | Hangup call |
| **MIX** | Answer call |
| **◀◀ / ▶▶** | Previous/Next track |

## Track Display Status

The track number on radio display indicates BT status:

| Track | Status |
|-------|--------|
| **80** | Waiting for BT connection |
| **10** | Device just connected (5 sec) |
| **90** | WiFi is OFF |
| **91** | WiFi is ON |
| **1+** | Normal playback mode |

## Web Interface

Connect to WiFi AP or use mDNS:

| Mode | Address |
|------|---------|
| **AP Mode** | `192.168.4.1` (SSID: `VW-BT1036`, Pass: `12345678`) |
| **mDNS** | `http://vw-bt.local` |

### Pages

- `/` - Main control panel
- `/bt` - Bluetooth debug logs & **Manual AT Command Console**
- `/cdc` - CDC protocol debug
- `/logs` - All logs combined
- `/wifi` - WiFi configuration
- `/update` - OTA firmware update

## Architecture (FreeRTOS)

The firmware is built on the FreeRTOS real-time operating system to ensure that time-critical operations are never blocked. The logic is split into three main tasks:

1.  **`cdc_task` (High Priority):** Manages all SPI communication with the head unit. Its high priority guarantees that the radio always receives responses in time, preventing "No CD Changer" errors.
2.  **`bt_task` (Medium Priority):** Handles the AT command queue and communication with the BT1036 module.
3.  **`webui_task` (Low Priority):** Serves the web interface and handles WebSocket communication. Its low priority ensures that user interface activity can never interfere with the core functionality.

Button presses from the head unit are safely passed from the ISR (Interrupt Service Routine) to the main logic loop via a thread-safe queue.

## Build & Upload

```bash
# Build
pio run

# Upload via USB
pio run --target upload

# Serial monitor
pio device monitor
```

## Project Structure

```
src/
├── main.cpp        # Entry point, RTOS tasks, button mapping, state machine
├── bt1036_at.cpp/h # BT1036C driver (AT command queue)
├── vw_cdc.cpp/h    # CDC emulator + button decoder
└── bt_webui.cpp/h  # Web UI, WebSocket, OTA
```
## License

MIT License

