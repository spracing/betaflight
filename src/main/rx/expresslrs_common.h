/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/io_types.h"
#include "drivers/time.h"

#define ELRS_CRC_LEN 256
#define ELRS_CRC14_POLY 0x2E57

#define ELRS_NR_SEQUENCE_ENTRIES 256

#define ELRS_RNG_MAX 0x7FFF
#define ELRS_FREQ_CORRECTION_MAX 200000
#define ELRS_FREQ_CORRECTION_MIN -20000
#define ELRS_FREQ_CORRECTION_INCREMENT 1

#define ELRS_RX_TX_BUFF_SIZE 8

#define ELRS_TELEMETRY_TYPE_LINK 0x01
#define ELRS_MSP_BIND 0x09

#define FREQ_HZ_TO_REG_VAL_900(freq) ((uint32_t)((double)freq/(double)SX127x_FREQ_STEP))
#define FREQ_HZ_TO_REG_VAL_24(freq) ((uint32_t)((double)freq/(double)SX1280_FREQ_STEP))

#define ELRS_RATE_MAX 4
#define ELRS_RATE_DEFAULT 0

#define ELRS_RSSI_LPF_CUTOFF_FREQ_HZ 1
#define ELRS_INTERVAL_S(interval) ((float)interval/(1000*1000))

typedef enum {
    AU433 = 0,
    AU915 = 1,
    EU433 = 2,
    EU868 = 3,
    FCC915 = 4,
    ISM2400 = 5,
} elrs_freq_domain_e;

typedef enum {
    TLM_RATIO_NO_TLM = 0,
    TLM_RATIO_1_128 = 1,
    TLM_RATIO_1_64 = 2,
    TLM_RATIO_1_32 = 3,
    TLM_RATIO_1_16 = 4,
    TLM_RATIO_1_8 = 5,
    TLM_RATIO_1_4 = 6,
    TLM_RATIO_1_2 = 7,
} elrs_tlm_ratio_e;

typedef enum {
    RATE_500HZ = 0,
    RATE_250HZ = 1,
    RATE_200HZ = 2,
    RATE_150HZ = 3,
    RATE_100HZ = 4,
    RATE_50HZ = 5,
    RATE_25HZ = 6,
    RATE_4HZ = 7,
} elrs_rf_rate_e; // Max value of 16 since only 4 bits have been assigned in the sync package.

typedef struct elrs_mod_settings_s {
    uint8_t index;
    elrs_rf_rate_e enumRate; // Max value of 16 since only 4 bits have been assigned in the sync package.
    uint8_t bw;
    uint8_t sf;
    uint8_t cr;
    uint32_t interval;                  // interval in us seconds that corresponds to that frequency
    elrs_tlm_ratio_e tlmInterval; // every X packets is a response TLM packet, should be a power of 2
    uint8_t fhssHopInterval;            // every X packets we hop to a new frequency. Max value of 16 since only 4 bits have been assigned in the sync package.
    uint8_t preambleLen;
    int8_t sensitivity;                 // expected RF sensitivity
} elrs_mod_settings_t;

extern elrs_mod_settings_t air_rate_config[2][ELRS_RATE_MAX];

void generateCrc14Table(void);
uint16_t calcCrc14(uint8_t *data, uint8_t len, uint16_t crc);

uint32_t getInitialFreq(const int32_t freqCorrection);
uint8_t getFHSSNumEntries(void);
uint8_t FHSSgetCurrIndex(void);
void FHSSsetCurrIndex(const uint8_t value);
uint32_t FHSSgetNextFreq(const int32_t freqCorrection);
void FHSSrandomiseFHSSsequence(const uint8_t UID[], const elrs_freq_domain_e dom);
uint8_t tlmRatioEnumToValue(const elrs_tlm_ratio_e enumval);

uint8_t getLQ(const bool addLQ);
void resetLQ(void);

uint16_t convertSwitch1b(const uint16_t val);
uint16_t convertSwitch3b(const uint16_t val);
uint16_t convertSwitch4b(const uint16_t val);
uint16_t convertAnalog(const uint16_t val);

bool elrsRxInit(IO_t resetPin, IO_t busyPin, const elrs_freq_domain_e dom);
void elrsRxConfig(const uint8_t bw, const uint8_t sf, const uint8_t cr, const uint32_t freq, const uint8_t preambleLen, const bool iqInverted);
void elrsRxStartReceiving(void);
bool elrsRxISR(timeUs_t *timeStamp);
void elrsRxTransmitData(const uint8_t *data);
void elrsRxReceiveData(uint8_t *data);
void elrsRxGetRFlinkInfo(int8_t *rssi, int8_t *snr);
void elrsRxSetFrequency(const uint32_t freq);
bool elrsRxGetFrequencyErrorbool(void);
void elrsRxSetPPMoffsetReg(const int32_t offset, const uint32_t freq);