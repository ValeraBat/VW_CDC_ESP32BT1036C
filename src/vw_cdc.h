/**
 * @file vw_cdc.h
 * @brief VW CDC (CD Changer) Protocol Emulator
 * 
 * Emulates a VW CD changer over SPI bus using vwcdpic protocol.
 * Decodes button commands from VW radio (DataOut pin, pulse-width based protocol).
 * 
 * Protocol details:
 *   - 8-byte SPI packets at 62.5kHz
 *   - Track/time in BCD format
 *   - Button commands via pulse-width encoding on DataOut line
 */

#pragma once
#include <Arduino.h>

// Playback state
enum class CdcPlayState {
    STOPPED,
    PLAYING,
    PAUSED
};

// Buttons/actions from the head unit
enum class CdcButton {
    NEXT_TRACK,
    PREV_TRACK,
    NEXT_DISC,
    PREV_DISC,
    PLAY_PAUSE,
    SCAN_TOGGLE,
    RANDOM_TOGGLE,
    STOP,
    DISC_1,
    DISC_2,
    DISC_3,
    DISC_4,
    DISC_5,
    DISC_6,
    UNKNOWN
};


// CDC status for display
struct CdcStatus {
    uint8_t      disc;      // 1..6
    uint8_t      track;     // 1..99
    CdcPlayState state;
    bool         randomOn;  // shuffle
    bool         scanOn;    // scan mode
};

// Callback: radio button press or command received
typedef void (*CdcButtonCallback)(CdcButton btn);

// Initialize the CDC emulator.
// sck, miso, mosi, ss — SPI bus pins connected to RNS-MFD.
// necPin — separate pin for receiving button commands from the radio.
// buttonCb — callback invoked on radio button presses.
void cdc_init(int sckPin, int misoPin, int mosiPin, int ssPin, int necPin,
              CdcButtonCallback buttonCb = nullptr);

// Call in loop()
void cdc_loop();

// --- API for BT layer / player logic ---

// Set disc and track (1..6, 1..99)
void cdc_setDiscTrack(uint8_t disc, uint8_t track);

// Set playback state
void cdc_setPlayState(CdcPlayState st);

// Enable/disable shuffle (SFL)
void cdc_setRandom(bool on);

// Enable/disable scan
void cdc_setScan(bool on);

// Reset modeByte to 0xFF (neutral value)
void cdc_resetModeFF();

// Set playback time (minutes 0-99, seconds 0-59)
void cdc_setPlayTime(uint8_t minutes, uint8_t seconds);

// Get current status
CdcStatus cdc_getStatus();

// Pause/resume interrupt processing
void cdc_pause(bool pause);
