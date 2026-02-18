/**
 * @file bt1036_at.h
 * @brief BT1036C Bluetooth Module Driver
 * 
 * AT command-based driver for BT1036C module.
 * Supports A2DP (audio streaming), AVRCP (playback control), and HFP (hands-free calls).
 * 
 * Features:
 *   - Command queue with timeout handling
 *   - Automatic status polling (A2DP, DEVSTAT)
 *   - Track info parsing (+TRACKSTAT, +TRACKINFO)
 *   - State change callbacks
 */

#pragma once
#include <Arduino.h>
#include <vector>

enum class BTConnState {
    DISCONNECTED,
    CONNECTING,
    CONNECTED_IDLE,
    PLAYING,
    PAUSED
};

// HFP call state as reported by +HFPSTAT (BT1036 AT set)
// Values are kept numeric to match firmware and allow forwards compatibility.
// 0 Unsupported, 1 Standby, 2 Connecting, 3 Connected, 4 Outgoing call,
// 5 Incoming call, 6 Active call, 7 Active held, 8..10 (3-way states)
typedef uint8_t BtHfpCallState;

// DEVSTAT decoding (bit field)
struct BtDevStat {
    bool powerOn;        // BIT0
    bool brDiscoverable; // BIT1
    bool bleAdvertising; // BIT2
    bool brScanning;     // BIT3
    bool bleScanning;    // BIT4
};

// Structure for storing paired device information
struct PairedDevice {
    String mac;
    String name;
    int    index;
};

// Callback: called when BTConnState changes
typedef void (*BtStateCallback)(BTConnState oldState, BTConnState newState);

// Module initialization (UART + basic AT/VER/ADDR test)
void bt1036_init(HardwareSerial &serial, uint8_t rxPin, uint8_t txPin);
void bt1036_loop();

// ---- Runtime A2DP / AVRCP ----
void bt1036_startScan();      // AT+SCAN=1
void bt1036_connectLast();    // AT+A2DPCONN
void bt1036_disconnect();     // AT+A2DPDISC
void bt1036_enterPairingMode();   // Enter pairing mode (disconnect + scan)
void bt1036_clearPairedDevices(); // Clear all paired devices
void bt1036_playPause();      // AT+PLAYPAUSE
void bt1036_play();           // AT+PLAY
void bt1036_pause();          // AT+PAUSE
void bt1036_stop();           // AT+STOP
void bt1036_nextTrack();      // AT+FORWARD
void bt1036_prevTrack();      // AT+BACKWARD

// ---- Paired device management ----
void bt1036_requestPairedList();
void bt1036_deleteDevice(const String& mac);
const std::vector<PairedDevice>& bt1036_getPairedList();
String bt1036_getConnectedMac();
String bt1036_getConnectedName();
bool bt1036_isPairedListPending();

// Track information (from +TRACKSTAT and +TRACKINFO)
struct TrackInfo {
    uint32_t elapsedSec;    // elapsed time in seconds
    uint32_t totalSec;      // total track time in seconds
    String   title;
    String   artist;
    String   album;
    bool     valid;         // data is up to date
};
TrackInfo bt1036_getTrackInfo();

// Check if track metadata (title/artist) has changed since last check
bool bt1036_checkTrackChanged();

// A2DP/AVRCP status/config
void bt1036_requestA2dpStat();   // AT+A2DPSTAT
void bt1036_requestA2dpInfo();   // AT+A2DPINFO
void bt1036_requestAvrcpStat();  // AT+AVRCPSTAT
void bt1036_setAvrcpCfg(uint8_t cfg); // AT+AVRCPCFG=cfg (bit field)
void bt1036_setVolume(uint8_t volume);      // AT+SPKVOL=vol,vol

// ---- Runtime HFP (calls) ----
// Commands per BT1036C datasheet (HFPxxx)
void bt1036_hfpConnectLast();             // AT+HFPCONN
void bt1036_hfpDisconnect();              // AT+HFPDISC
void bt1036_answerCall();                 // AT+HFPANSW
void bt1036_hangupCall();                 // AT+HFPCHUP
void bt1036_hfpThreeWay(uint8_t mode);    // AT+HFPMCAL=0/1/2
void bt1036_hfpVoiceRecognition(bool on); // AT+HFPVR=0/1
void bt1036_setMicMute(bool muteOn);      // AT+MICMUTE=0/1

// ---- HFP state helpers (from unsolicited +HFPSTAT/+HFPAUDIO and/or AT+HFPSTAT) ----
BtHfpCallState bt1036_getHfpCallState();
bool           bt1036_isHfpCallIncoming();
bool           bt1036_isHfpCallInProgress();
bool           bt1036_isHfpAudioActive();

// ---- Runtime system commands ----
void bt1036_softReboot();                 // AT+REBOOT
void bt1036_setBtEnabled(bool enabled);   // AT+BTEN=0/1
void bt1036_sendRawCommand(const String& cmd); // For manual command input
void bt1036_clearQueue();                 // Clear command queue

// ---- State getters ----
BTConnState bt1036_getState();
BtDevStat   bt1036_getDevStat();
void        bt1036_setStateCallback(BtStateCallback cb);

// ---- Helpers for one-time initial setup (EEPROM) ----
// These can be called from a service CLI, but are not required on every boot.

void bt1036_getName();                                // AT+NAME
void bt1036_setName(const String &name, bool suffix); // AT+NAME=...,0/1

void bt1036_getBLEName();                                      // AT+LENAME
void bt1036_setBLEName(const String &name, bool suffix);       // AT+LENAME=...,0/1

void bt1036_setMicGain(uint8_t gain0_15);                      // AT+MICGAIN=0..15
void bt1036_setSpkVol(uint8_t a2dp0_15, uint8_t hfp0_15);      // AT+SPKVOL=A2DP,HFP
void bt1036_setTxPower(uint8_t level0_15);                     // AT+TXPOWER=0..15

void bt1036_getProfile();                     // AT+PROFILE
void bt1036_setProfile(uint16_t mask);        // AT+PROFILE=mask
void bt1036_getAutoconn();                    // AT+AUTOCONN
void bt1036_setAutoconn(uint16_t mask);       // AT+AUTOCONN=mask

void bt1036_getSsp();                         // AT+SSP
void bt1036_setSsp(uint8_t mode0_3);          // AT+SSP=0..3

void bt1036_getCod();                         // AT+COD
void bt1036_setCod(const String &codHex6);    // AT+COD=XXXXXX (6 hex chars)

void bt1036_getSep();                         // AT+SEP
void bt1036_setSep(uint8_t hexVal);           // AT+SEP=0x01..0xFF (0..255)

// HFP configuration (per datasheet)
void bt1036_requestHfpStat();                 // AT+HFPSTAT
void bt1036_setHfpSampleRate(uint32_t rate);  // AT+HFPSR=0/8000/16000/48000
void bt1036_setHfpConfig(uint8_t cfg);        // AT+HFPCFG=bitfield

// Diagnostics
void bt1036_requestDevStat();                 // AT+DEVSTAT
void bt1036_requestStat();                    // AT+STAT

// Pause/resume background polling
void bt1036_pausePolling(bool pause);

// One-time factory setup for BT1036 module.
// Called manually from CLI/Serial/WebUI once; the module stores settings in its NVM.
void bt1036_runFactorySetup();

// Track if AVRCP commands are safe to send
bool bt1036_isAvrcpReady();

// Check if track metadata has changed since last call
bool bt1036_checkTrackChanged();

