# VW CDC Bluetooth Emulator - AI Agent Instructions

## Project Overview
ESP32-S2 firmware emulating a VW CDC (CD Changer) to integrate a BT1036C Bluetooth module with a VW RNS-MFD head unit. The system translates NEC IR protocol commands from the head unit into Bluetooth A2DP/AVRCP/HFP controls.

**Hardware**: ESP-WROVER-KIT (ESP32) + BT1036C Bluetooth module + VW RNS-MFD radio via SPI/IR

## Architecture

### Three-Layer Design
1. **CDC Protocol Layer** (`vw_cdc.cpp/h`) - SPI emulation of CDC device + NEC IR decoder
2. **Bluetooth Controller** (`bt1036_at.cpp/h`) - AT command queue for BT1036C module over UART
3. **Web UI** (`bt_webui.cpp/h`) - WiFi AP + WebSocket control interface + OTA updates

### Data Flow
```
VW Radio (NEC IR) → CDC Decoder (ISR) → Button Callbacks → BT Commands (Queued) → BT1036C Module
                                      ↓
                              Web UI (WebSocket broadcast)
```

## Critical Patterns

### 1. NEC IR Protocol Decoding (ISR-based)
- **ISR handler** in `vw_cdc.cpp:nec_isr()` decodes NEC protocol on MISO pin (pin 12)
- **State machine**: WAIT_START_HIGH → WAIT_START_LOW → WAIT_BIT_HIGH → WAIT_BIT_LOW
- **Timing model** (based on STM8 reference implementation):
  - Start bit: 9ms HIGH (8800-9200µs) → 4.5ms LOW (4000-5000µs)
  - Data bit '0': 560µs HIGH (< 800µs)
  - Data bit '1': 1690µs HIGH (1000-2500µs)
  - Bit value determined by **HIGH pulse duration**, not LOW
- Raw pulses logged to ring buffer (`g_rawBuf[64]`) for debugging via WebUI
- Frame validation requires: `addr_lo=0xCA, addr_hi=0x34, cmd XOR cmdInv = 0xFF`
- **Never block in ISR** - only sets `nec_frameReady` flag, processing in `cdc_pollNec()`

### 2. AT Command Queue (BT1036C)
- **Circular queue** in `bt1036_at.cpp` (10 slots) prevents UART overflow
- Commands wait for `OK`/`ERROR` response before sending next (2s timeout)
- State machine updates `BTConnState` from unsolicited messages (`+A2DPSTAT=`, `+PLAYSTAT=`)
- **Critical**: Never send AT commands directly - always use `queuePush()` helper
- Background polling: `AT+A2DPSTAT` and `AT+DEVSTAT` every 3 seconds

### 3. Button Remapping Strategy
See `main.cpp:onCdcButton()` - CD1-CD3 buttons repurposed:
- **CD1** → Play/Pause (not disc select)
- **CD2** → Stop/Disconnect
- **CD3** → HFP mic mute toggle
- **SCAN** → Answer call
- **RANDOM** → Hangup call
- Track buttons map to `AT+FORWARD`/`AT+BACKWARD`

### 4. Web UI Architecture
- **Dual log streams**: General logs (`btWebUI_log()`) + CDC raw NEC data (`btWebUI_broadcastCdcRaw()`)
- Ring buffer (128 entries) replays on WebSocket connect
- Three pages: Main (collapsible controls), WiFi setup, CDC debug (dual terminals)
- HTML embedded as `PROGMEM` strings - edit in-place for UI changes
- OTA via ElegantOTA library on `/update` endpoint

## Build & Debug Workflow

### PlatformIO Commands
```bash
# Build only
pio run

# Upload firmware (USB)
pio run --target upload

# Serial monitor (115200 baud)
pio device monitor

# Clean build
pio run --target clean
```

### Debug Levels
- Set `build_flags = -DCORE_DEBUG_LEVEL=5` in `platformio.ini` for verbose ESP32 logs
- CDC NEC raw pulses visible in Web UI → CDC Debug page (right panel)
- BT1036 AT command trace via WebUI Main page log or Serial console

### Pin Configuration (`main.cpp`)
| Signal | Pin | Notes |
|--------|-----|-------|
| BT_RX  | 16  | ESP RX ← BT1036 TX (Serial2) |
| BT_TX  | 17  | ESP TX → BT1036 RX (Serial2) |
| CDC_SCK | 18 | VSPI CLK to RNS-MFD |
| CDC_MISO | 19 | VSPI MISO (not actively used) |
| CDC_MOSI | 23 | VSPI MOSI → RNS-MFD |
| CDC_NEC | 4 | VW DataOut (NEC IR input) |

### Common Issues
1. **NEC decoding fails**: 
   - Check `RAW:` logs in CDC Debug page
   - Valid start: ~9000µs high, ~4500µs low
   - Data bits: ~560µs high = '0', ~1690µs high = '1'
   - **Critical**: NEC decoder is based on STM8 reference (see `cdc_proto.c` for timing model)
2. **BT commands timeout**: Increase `CMD_TIMEOUT_MS` or check UART wiring (115200 baud, 8N1)
3. **WiFi won't connect**: Use AP mode (`VW-BT1036` / `12345678`), configure via `/wifi` page

## Dependencies & Libraries
- `WebSockets` (links2004) - WebSocket server for live logs
- `ElegantOTA` (v3.1.0) - OTA update UI
- ESP32 Arduino framework - SPI, Preferences (NVS), WiFi, mDNS

## Naming Conventions
- Global state prefixed `g_` (e.g., `g_currentDisc`, `g_rawBuf`)
- Static functions use `snake_case` (e.g., `cdc_sendPackage()`, `bt1036_loop()`)
- ISR functions marked `IRAM_ATTR` for flash-to-RAM optimization
- Enums use `PascalCase` (e.g., `BTConnState::PLAYING`, `CdcButton::NEXT_TRACK`)

## Testing Checklist
When modifying code, verify:
1. **CDC protocol**: Disc/track display updates on radio LCD
2. **NEC decoding**: Button presses logged as `[MAIN] CDC button: X`
3. **BT control**: Play/pause/skip work via phone app
4. **HFP calls**: Answer/hangup buttons functional, mic mute toggles
5. **Web UI**: Logs appear in real-time, controls send commands

## Factory Setup Flow
First-time BT1036C configuration (via Web UI Main → System → "Run Factory Setup"):
1. Sets device name, COD (`240404` = car audio), profiles (A2DP+AVRCP+HFP=168)
2. Configures HFP: 16kHz sample rate, echo cancel, auto-reconnect
3. **Must reboot BT1036** after factory setup to persist EEPROM changes

## File Roles
- `main.cpp` - Entry point, button mapping logic, state coordination
- `vw_cdc.cpp` - SPI CDC emulation + NEC IR decoder (ISR-driven)
- `bt1036_at.cpp` - BT module driver with AT command queue
- `bt_webui.cpp` - WiFi AP, web pages, API handlers, WebSocket logs
- `platformio.ini` - Build config, dependencies, ESP32-S2 board settings
