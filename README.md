# VW CDC Bluetooth Emulator (BT CONNECTION NOT TESTED YET, BUT COMMUNCATION BETWEEN ESP32 AND HU WORKS FINE)

ESP32 firmware that emulates a VW CD Changer (CDC) to integrate a BT1036C Bluetooth module with VW RNS-MFD head units.

## Features

- **A2DP Audio Streaming** - Play music from phone via Bluetooth
- **AVRCP Controls** - Track skip, play/pause from radio buttons
- **HFP Hands-Free** - Answer/hangup calls, mic mute
- **Auto-Play** - Automatic playback on reconnection
- **Status Display** - BT connection status via track number
- **Web UI** - WiFi configuration, live logs, OTA updates

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
| **SCAN** | Hangup call |
| **MIX** | Answer call |
| **◀◀ / ▶▶** | Previous/Next track |

## Track Display Status

The track number on radio display indicates BT status:

| Track | Status |
|-------|--------|
| **80** | Waiting for BT connection |
| **10** | Device just connected (5 sec) |
| **1+** | Normal playback mode |

## Auto-Play Behavior

- **Known device (auto-reconnect)**: Instant playback
- **New device (after CD4/CD6)**: 5 second delay, then playback

## Web Interface

Connect to WiFi AP or use mDNS:

| Mode | Address |
|------|---------|
| **AP Mode** | `192.168.4.1` (SSID: `VW-BT1036`, Pass: `12345678`) |
| **mDNS** | `http://vw-bt.local` |

### Pages

- `/` - Main control panel
- `/bt` - Bluetooth debug logs
- `/cdc` - CDC protocol debug
- `/logs` - All logs combined
- `/wifi` - WiFi configuration
- `/update` - OTA firmware update

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
├── main.cpp        # Entry point, button mapping, state machine
├── bt1036_at.cpp/h # BT1036C driver (AT command queue)
├── vw_cdc.cpp/h    # CDC emulator + button decoder
└── bt_webui.cpp/h  # Web UI, WebSocket, OTA
```

## Protocol Details

### CDC → Radio (SPI)
- 8-byte packets at 62.5kHz
- Track/time in BCD format
- State machine: IDLE → INIT → LEAD_IN → PLAY

### Radio → CDC (DataOut)
- Pulse-width encoded commands
- 32-bit packets: `[0x53] [0x2C] [cmd] [~cmd]`
- Start pulse: ~4.5ms LOW
- Bit '0': ~560µs LOW, Bit '1': ~1680µs LOW

## Button Codes (VW RNS-MFD)

| Button | Code |
|--------|------|
| CD1 | 0x0C |
| CD2 | 0x8C |
| CD3 | 0x4C |
| CD4 | 0xCC |
| CD5 | 0x2C |
| CD6 | 0xAC |
| NEXT | 0xF8 |
| PREV | 0x78 |
| SCAN | 0xA0 |
| MIX | 0xE0 |

## Factory Setup

Run once via Web UI (Main → System → Factory Setup):
1. Sets device name: `VW_BT1036`
2. Configures COD: `240404` (car audio)
3. Enables profiles: A2DP + AVRCP + HFP
4. Configures HFP: 16kHz, echo cancel, auto-reconnect

**Reboot BT1036 after factory setup to persist changes.**

## License

MIT License
