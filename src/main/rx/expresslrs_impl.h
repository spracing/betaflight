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

// expresslrs packet header types
// 00 -> standard 4 channel data packet
// 01 -> switch data packet
// 11 -> tlm packet
// 10 -> sync packet with hop data
typedef enum {
    ELRS_RC_DATA_PACKET=0x00,
    ELRS_MSP_DATA_PACKET=0x01,
    ELRS_SYNC_PACKET=0x02,
    ELRS_TLM_PACKET=0x03,
} elrs_packet_type_e;

typedef enum {
    DIO_UNKNOWN = 0,
    DIO_RX_DONE,
    DIO_TX_DONE
} dioReason_e;

typedef enum {
    LQ_NONE,
    LQ_TRANSMITTING,
    LQ_RECEIVING
} lqMode_e;

typedef struct elrsReceiver_s {

    IO_t resetPin;
    IO_t busyPin;

    int32_t freqOffset;
    uint32_t currentFreq;

    volatile uint8_t nonceRX; // nonce that we THINK we are up to.

    elrs_mod_settings_t *mod_params;

    const uint8_t *UID;

    int8_t rssi;
    int8_t snr;

    uint8_t uplinkLQ;
    lqMode_e lqMode;

    dioReason_e dioReason;

    uint32_t lastValidPacketUs;
    uint16_t missedPackets;

    uint16_t packetHandlingToTockDelayUs;

    uint16_t cycleInterval;
    uint32_t rfModeLastCycled;
    uint8_t rateIndex;

    uint32_t lastConfigCheckTime;
    bool configChanged;

    bool bound;
    bool failsafe;
    bool firstConnection;
    bool synced;
    volatile bool nextChannelRequired;


    elrsRxInitFnPtr init;
    elrsRxConfigFnPtr config;
    elrsRxStartReceivingFnPtr startReceiving;
    elrsRxISRFnPtr rxISR;
    elrsRxTransmitDataFnPtr transmitData;
    elrsRxReceiveDataFnPtr receiveData;
    elrsRxGetRFlinkInfoFnPtr getRFlinkInfo;
    elrsRxSetFrequencyFnPtr setFrequency;
    elrsRxHandleFreqCorrectionFnPtr handleFreqCorrection;
    elrsRxIsBusyFnPtr isBusy;

    timerOvrHandlerRec_t timerUpdateCb;
} elrsReceiver_t;

