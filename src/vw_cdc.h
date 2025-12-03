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

// Состояние воспроизведения
enum class CdcPlayState {
    STOPPED,
    PLAYING,
    PAUSED
};

// Кнопки/действия от магнитолы
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


// Состояние для отображения (если понадобится)
struct CdcStatus {
    uint8_t      disc;      // 1..6
    uint8_t      track;     // 1..99
    CdcPlayState state;
    bool         randomOn;  // shuffle
    bool         scanOn;    // scan-режим
};

// callback: магнитола нажала кнопку (или послала команду)
typedef void (*CdcButtonCallback)(CdcButton btn);

// Инициализация CDC-эмулятора.
// sck, miso, mosi, ss — пины SPI шины, подключённой к RNS-MFD.
// necPin — отдельный пин для приема NEC IR сигнала от магнитолы.
// buttonCb — колбэк, который будет вызываться при командах от магнитолы.
void cdc_init(int sckPin, int misoPin, int mosiPin, int ssPin, int necPin,
              CdcButtonCallback buttonCb = nullptr);

// Вызывать в loop()
void cdc_loop();

// --- API для BT-слоя / логики плеера ---

// Установить диск и трек (1..6, 1..99)
void cdc_setDiscTrack(uint8_t disc, uint8_t track);

// Состояние воспроизведения
void cdc_setPlayState(CdcPlayState st);

// Включить/выключить shuffle (SFL)
void cdc_setRandom(bool on);

// Включить/выключить scan
void cdc_setScan(bool on);

// Сбросить modeByte в 0xFF (нейтральное значение)
void cdc_resetModeFF();

// Установить время воспроизведения (minutes 0-99, seconds 0-59)
void cdc_setPlayTime(uint8_t minutes, uint8_t seconds);

// Получить текущее состояние
CdcStatus cdc_getStatus();
