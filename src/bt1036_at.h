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

enum class BTConnState {
    DISCONNECTED,
    CONNECTING,
    CONNECTED_IDLE,
    PLAYING,
    PAUSED
};

// Расшифровка DEVSTAT (битовое поле)
struct BtDevStat {
    bool powerOn;        // BIT0
    bool brDiscoverable; // BIT1
    bool bleAdvertising; // BIT2
    bool brScanning;     // BIT3
    bool bleScanning;    // BIT4
};

// callback: вызывается при смене BTConnState
typedef void (*BtStateCallback)(BTConnState oldState, BTConnState newState);

// Инициализация модуля (только UART + базовый тест AT/VER/ADDR)
void bt1036_init(HardwareSerial &serial, uint8_t rxPin, uint8_t txPin);
void bt1036_loop();

// ---- Runtime A2DP / AVRCP ----
void bt1036_startScan();      // AT+SCAN=1
void bt1036_connectLast();    // AT+A2DPCONN
void bt1036_disconnect();     // AT+A2DPDISC
void bt1036_enterPairingMode();   // Войти в режим сопряжения (disconnect + scan)
void bt1036_clearPairedDevices(); // Очистить список сопряжённых устройств
void bt1036_playPause();      // AT+PLAYPAUSE
void bt1036_play();           // AT+PLAY
void bt1036_pause();          // AT+PAUSE
void bt1036_stop();           // AT+STOP
void bt1036_nextTrack();      // AT+FORWARD
void bt1036_prevTrack();      // AT+BACKWARD

// Информация о треке (из +TRACKSTAT и +TRACKINFO)
struct TrackInfo {
    uint32_t elapsedSec;    // прошедшее время в секундах
    uint32_t totalSec;      // общее время трека в секундах
    String   title;
    String   artist;
    String   album;
    bool     valid;         // данные актуальны
};
TrackInfo bt1036_getTrackInfo();

// Статусы/конфиг A2DP/AVRCP
void bt1036_requestA2dpStat();   // AT+A2DPSTAT
void bt1036_requestA2dpInfo();   // AT+A2DPINFO
void bt1036_requestAvrcpStat();  // AT+AVRCPSTAT
void bt1036_setAvrcpCfg(uint8_t cfg); // AT+AVRCPCFG=cfg (битовое поле)
void bt1036_setVolume(uint8_t volume);      // AT+SPKVOL=vol,vol

// ---- Runtime HFP (звонки) ----
// команды по даташиту BT1036C (HFPxxx)
void bt1036_hfpConnectLast();             // AT+HFPCONN
void bt1036_hfpDisconnect();              // AT+HFPDISC
void bt1036_answerCall();                 // AT+HFPANSW
void bt1036_hangupCall();                 // AT+HFPCHUP
void bt1036_hfpThreeWay(uint8_t mode);    // AT+HFPMCAL=0/1/2
void bt1036_hfpVoiceRecognition(bool on); // AT+HFPVR=0/1
void bt1036_setMicMute(bool muteOn);      // AT+MICMUTE=0/1

// ---- Runtime системные ----
void bt1036_softReboot();                 // AT+REBOOT
void bt1036_setBtEnabled(bool enabled);   // AT+BTEN=0/1
void bt1036_sendRawCommand(const String& cmd); // Для ручного ввода команд

// ---- Геттеры состояния ----
BTConnState bt1036_getState();
BtDevStat   bt1036_getDevStat();
void        bt1036_setStateCallback(BtStateCallback cb);

// ---- Хелперы для ручной первичной настройки (EEPROM) ----
// Их можно дергать из "сервисного" CLI, но не обязательно использовать каждый старт.

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

void bt1036_setI2sConfig(uint8_t cfg);        // AT+I2SCFG=cfg (битовое поле)

// HFP-настройки (по даташиту)
void bt1036_requestHfpStat();                 // AT+HFPSTAT
void bt1036_setHfpSampleRate(uint32_t rate);  // AT+HFPSR=0/8000/16000/48000
void bt1036_setHfpConfig(uint8_t cfg);        // AT+HFPCFG=bitfield

// Диагностика
void bt1036_requestDevStat();                 // AT+DEVSTAT
void bt1036_requestStat();                    // AT+STAT

// Приостановка фонового опроса
void bt1036_pausePolling(bool pause);

// Одноразовая "фабричная" настройка модуля BT1036.
// Вызывается вручную из CLI/Serial/WebUI один раз, дальше модуль хранит всё в своей NVM.
void bt1036_runFactorySetup();

