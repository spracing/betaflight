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

#include <string.h>
#include "platform.h"

#ifdef USE_RX_EXPRESSLRS

#include "build/debug.h"

#include "common/maths.h"

#include "drivers/io.h"
#include "drivers/rx/rx_spi.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "sensors/battery.h"

#include "config/config.h"
#include "config/feature.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx_spi.h"
#include "pg/rx_spi_expresslrs.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"
#include "rx/rx_spi_common.h"

#include "rx/expresslrs.h"
#include "rx/expresslrs_common.h"

STATIC_UNIT_TESTED elrsReceiver_t receiver;
static const uint8_t BindingUID[6] = {0,1,2,3,4,5}; // Special binding UID values
static uint16_t crcInitializer = 0;

#ifdef USE_RX_RSSI_DBM
static pt1Filter_t rssiFilter;
#endif

static void handleTelemetry(void)
{
    uint8_t packet[8];

    packet[0] = ELRS_TLM_PACKET;
    packet[1] = ELRS_TELEMETRY_TYPE_LINK;
    packet[2] = receiver.rssi;

    uint16_t voltage = getBatteryVoltage();

    packet[3] = (voltage & 0xFF00) >> 8;
    packet[4] = receiver.snr;
    packet[5] = receiver.uplinkLQ;
    packet[6] = (voltage & 0x00FF);

    uint16_t crc = calcCrc14(packet, 7, crcInitializer);
    packet[0] |= (crc >> 6) & 0xFC;
    packet[7] = crc & 0xFF;
    elrsRxTransmitData(packet);
    receiver.sentTelemetry = true;
}

static void handleFreqCorrection(void) 
{
    if (elrsRxGetFrequencyErrorbool()) { //logic is flipped compared to original code
        if (receiver.freqOffset > ELRS_FREQ_CORRECTION_MIN) {
            receiver.freqOffset -= ELRS_FREQ_CORRECTION_INCREMENT;
        } else {
            receiver.freqOffset = 0; //reset because something went wrong
        }
    } else {
        if (receiver.freqOffset < ELRS_FREQ_CORRECTION_MAX) {
            receiver.freqOffset += ELRS_FREQ_CORRECTION_INCREMENT;
        } else {
            receiver.freqOffset = 0; //reset because something went wrong
        }
    }
    elrsRxSetPPMoffsetReg(receiver.freqOffset, receiver.currentFreq); //as above but corrects a different PPM offset based on freq error
}

static void setNextChannel(void)
{
    if ((receiver.mod_params->fhssHopInterval == 0) || !receiver.bound) {
        return;
    }

    if (((receiver.nonceRX + 1) % receiver.mod_params->fhssHopInterval) != 0) {
        if (rxExpressLrsSpiConfig()->domain != ISM2400) {
            handleFreqCorrection(); //corrects for RX freq offset
        }
    } else {
        receiver.currentFreq = FHSSgetNextFreq(receiver.freqOffset);
        elrsRxSetFrequency(receiver.currentFreq);
    }

    if (receiver.mod_params->tlmInterval == TLM_RATIO_NO_TLM || (((receiver.nonceRX + 1) % (tlmRatioEnumToValue(receiver.mod_params->tlmInterval))) != 0)) {
        elrsRxStartReceiving();
    } else {
        handleTelemetry();
    }
}

static void setRFLinkRate(const uint8_t index)
{
    receiver.currentFreq = getInitialFreq(receiver.freqOffset);
    receiver.mod_params = (rxExpressLrsSpiConfig()->domain == ISM2400) ? &air_rate_config[1][index] : &air_rate_config[0][index];
    // Wait for (11/10) 110% of time it takes to cycle through all freqs in FHSS table (in ms)
    receiver.cycleInterval = ((uint32_t)11U * getFHSSNumEntries() * receiver.mod_params->fhssHopInterval * receiver.mod_params->interval) / (10U * 1000U);
    elrsRxConfig(receiver.mod_params->bw, receiver.mod_params->sf, receiver.mod_params->cr, receiver.currentFreq, receiver.mod_params->preambleLen, receiver.UID[5] & 0x01);

#ifdef USE_RX_RSSI_DBM
    pt1FilterInit(&rssiFilter, pt1FilterGain(ELRS_RSSI_LPF_CUTOFF_FREQ_HZ, ELRS_INTERVAL_S(receiver.mod_params->interval)));
#endif
}

/**
 * 10bit uses 10 bits for each analog channel,
 * 1 bit for 8 switches
 * 4 analog channels, 8 switches = 48 bits in total
 */
static void unpackChannelData10bit(uint16_t *rcData, const uint8_t *payload)
{
    rcData[0] = convertAnalog((payload[0] << 3) | ((payload[4] & 0xC0) >> 5));
    rcData[1] = convertAnalog((payload[1] << 3) | ((payload[4] & 0x30) >> 3));
    rcData[2] = convertAnalog((payload[2] << 3) | ((payload[4] & 0x0C) >> 1));
    rcData[3] = convertAnalog((payload[3] << 3) | ((payload[4] & 0x03) << 1));

    rcData[4] = convertSwitch1b(payload[5] & 0x80);
    rcData[5] = convertSwitch1b(payload[5] & 0x40);
    rcData[6] = convertSwitch1b(payload[5] & 0x20);
    rcData[7] = convertSwitch1b(payload[5] & 0x10);
    rcData[8] = convertSwitch1b(payload[5] & 0x08);
    rcData[9] = convertSwitch1b(payload[5] & 0x04);
    rcData[10] = convertSwitch1b(payload[5] & 0x02);
    rcData[11] = convertSwitch1b(payload[5] & 0x01);
}

/**
 * Hybrid switches uses 10 bits for each analog channel,
 * 2 bits for the low latency switch[0]
 * 3 bits for the round-robin switch index and 2 bits for the value
 * 4 analog channels, 1 low latency switch and round robin switch data = 47 bits (1 free)
 */
static void unpackChannelDataHybridSwitches(uint16_t *rcData, const uint8_t *payload)
{
    // The analog channels
    rcData[0] = convertAnalog((payload[0] << 3) | ((payload[4] & 0xC0) >> 5));
    rcData[1] = convertAnalog((payload[1] << 3) | ((payload[4] & 0x30) >> 3));
    rcData[2] = convertAnalog((payload[2] << 3) | ((payload[4] & 0x0C) >> 1));
    rcData[3] = convertAnalog((payload[3] << 3) | ((payload[4] & 0x03) << 1));

    // The low latency switch
    rcData[4] = convertSwitch1b((payload[5] & 0x40) >> 6);

    // The round-robin switch, switchIndex is actually index-1 
    // to leave the low bit open for switch 7 (sent as 0b11x)
    // where x is the high bit of switch 7
    uint8_t switchIndex = (payload[5] & 0x38) >> 3;
    uint16_t switchValue = convertSwitch3b(payload[5] & 0x07);

    switch (switchIndex) {
        case 0:
            rcData[5] = switchValue;
            break;
        case 1:
            rcData[6] = switchValue;
            break;
        case 2:
            rcData[7] = switchValue;
            break;
        case 3:
            rcData[8] = switchValue;
            break;
        case 4:
            rcData[9] = switchValue;
            break;
        case 5:
            rcData[10] = switchValue;
            break;
        case 6:
            FALLTHROUGH;
        case 7:
            rcData[11] = convertSwitch4b(payload[5] & 0x0F);
            break;
        default:
            break;
    }
}

static void initializeReceiver(void)
{
    FHSSrandomiseFHSSsequence(receiver.UID, rxExpressLrsSpiConfig()->domain);
    resetLQ();

    receiver.nonceRX = 0;
    receiver.missedPackets = 0;
    receiver.freqOffset = 0;
    receiver.failsafe = false;
    receiver.sentTelemetry = false;
    receiver.firstConnection = false;
    receiver.rssi = 0;
    receiver.snr = 0;
    receiver.uplinkLQ = 0;
    setRFLinkRate(rxExpressLrsSpiConfig()->rateIndex);

    receiver.rfModeLastCycled = millis();
    receiver.lastValidPacket = micros();
}

static void enterBindingMode(void)
{
    receiver.bound = false;
    receiver.UID = BindingUID;
    crcInitializer = 0;

    receiver.freqOffset = 0;
    receiver.failsafe = false;
    receiver.sentTelemetry = false;

    setRFLinkRate(ELRS_RATE_DEFAULT);
    elrsRxStartReceiving();
}

static void unpackBindPacket(uint8_t *packet)
{
    rxExpressLrsSpiConfigMutable()->UID[2] = packet[3];
    rxExpressLrsSpiConfigMutable()->UID[3] = packet[4];
    rxExpressLrsSpiConfigMutable()->UID[4] = packet[5];
    rxExpressLrsSpiConfigMutable()->UID[5] = packet[6];

    writeEEPROM();

    receiver.UID = rxExpressLrsSpiConfig()->UID;
    crcInitializer = (receiver.UID[4] << 8) | receiver.UID[5];
    receiver.bound = true;

    initializeReceiver();
    elrsRxStartReceiving();
}

static rx_spi_received_e processRFPacket(uint8_t *payload, const uint32_t timeStamp) {

    uint8_t packet[ELRS_RX_TX_BUFF_SIZE];

    elrsRxReceiveData(packet);

    elrs_packet_type_e type = packet[0] & 0x03;
    uint16_t inCRC = (((uint16_t)(packet[0] & 0xFC)) << 6 ) | packet[7];

    packet[0] = type;
    uint16_t calculatedCRC = calcCrc14(packet, 7, crcInitializer);

    if (inCRC != calculatedCRC) {
        return RX_SPI_RECEIVED_NONE;
    }

    uint8_t switchEncModeExpected = rxExpressLrsSpiConfig()->hybridSwitches ? 0x01 : 0x00;

    uint8_t indexIN;
    elrs_tlm_ratio_e tlmRateIn;
    uint8_t switchEncMode;

    receiver.lastValidPacket = timeStamp;
    receiver.nonceRX += 1;
    receiver.missedPackets = 0;
    receiver.failsafe = false;
    receiver.uplinkLQ = getLQ(true);
    elrsRxGetRFlinkInfo(&receiver.rssi, &receiver.snr);
    uint16_t rssiScaled = scaleRange(constrain(receiver.rssi, receiver.mod_params->sensitivity, -50), receiver.mod_params->sensitivity, -50, 0, 1023);
    setRssi(rssiScaled, RSSI_SOURCE_RX_PROTOCOL);
#ifdef USE_RX_RSSI_DBM
    setRssiDbm(pt1FilterApply(&rssiFilter, receiver.rssi), RSSI_SOURCE_RX_PROTOCOL);
#endif
#ifdef USE_RX_LINK_QUALITY_INFO
    setLinkQualityDirect(receiver.uplinkLQ);
    rxSetRfMode((uint8_t)RATE_4HZ - (uint8_t)receiver.mod_params->enumRate);
#endif

    if (!receiver.firstConnection) {
        receiver.firstConnection = true;
        writeEEPROM();
    }

    switch(type) {
        case ELRS_RC_DATA_PACKET:
            memcpy(payload, &packet[1], 6);
            break;
        case ELRS_MSP_DATA_PACKET:
            if (!receiver.bound && packet[1] == 1 && packet[2] == ELRS_MSP_BIND) {
                unpackBindPacket(packet);
            }
            break;
        case ELRS_TLM_PACKET:
            //not implemented
            break;
        case ELRS_SYNC_PACKET:
            indexIN = (packet[3] & 0xC0) >> 6;
            tlmRateIn = (packet[3] & 0x38) >> 3;
            switchEncMode = (packet[3] & 0x06) >> 1;

            if (switchEncModeExpected == switchEncMode && receiver.mod_params->index == indexIN && packet[4] == receiver.UID[3] && packet[5] == receiver.UID[4] && packet[6] == receiver.UID[5]) {
                if (receiver.mod_params->tlmInterval != tlmRateIn) { // change link parameters if required
                    receiver.mod_params->tlmInterval = tlmRateIn;
                }

                if (receiver.nonceRX != packet[2] || FHSSgetCurrIndex() != packet[1]) {
                    FHSSsetCurrIndex(packet[1]);
                    receiver.nonceRX = packet[2];
                }
            }
            break;
        default:
            return RX_SPI_RECEIVED_NONE;
    }

    setNextChannel();

    return RX_SPI_RECEIVED_DATA;
}

bool expressLrsSpiInit(const struct rxSpiConfig_s *rxConfig, struct rxRuntimeState_s *rxRuntimeState, rxSpiExtiConfig_t *extiConfig)
{
    if (!rxSpiExtiConfigured()) {
        return false;
    }

    rxSpiCommonIOInit(rxConfig);
	
	rxRuntimeState->channelCount = 12;
	
	extiConfig->ioConfig = IOCFG_IPD;
    extiConfig->trigger = BETAFLIGHT_EXTI_TRIGGER_RISING;

    if (rxExpressLrsSpiConfig()->resetIoTag) {
        receiver.resetPin = IOGetByTag(rxExpressLrsSpiConfig()->resetIoTag);
	} else {
        receiver.resetPin = IO_NONE;
    }

    if (rxExpressLrsSpiConfig()->busyIoTag) {
        receiver.busyPin = IOGetByTag(rxExpressLrsSpiConfig()->busyIoTag);
    } else {
        receiver.busyPin = IO_NONE;
    }

    if (!elrsRxInit(receiver.resetPin, receiver.busyPin, rxExpressLrsSpiConfig()->domain)) {
        return false;
    }

    if (rssiSource == RSSI_SOURCE_NONE) {
        rssiSource = RSSI_SOURCE_RX_PROTOCOL;
    }

    if (linkQualitySource == LQ_SOURCE_NONE) {
        linkQualitySource = LQ_SOURCE_RX_PROTOCOL_CRSF;
    }

    if (rxExpressLrsSpiConfig()->UID[0] || rxExpressLrsSpiConfig()->UID[1]
        || rxExpressLrsSpiConfig()->UID[2] || rxExpressLrsSpiConfig()->UID[3]
        || rxExpressLrsSpiConfig()->UID[4] || rxExpressLrsSpiConfig()->UID[5]) {
        receiver.bound = true;
        receiver.UID = rxExpressLrsSpiConfig()->UID;
        crcInitializer = (receiver.UID[4] << 8) | receiver.UID[5];
    } else {
        receiver.bound = false;
        receiver.UID = BindingUID;
        crcInitializer = 0;
        rxExpressLrsSpiConfigMutable()->rateIndex = ELRS_RATE_DEFAULT;
    }

    generateCrc14Table();
    initializeReceiver();
    elrsRxStartReceiving();

    return true;
}

static void handleTimeout(void)
{
    if (!receiver.failsafe) {

        const uint32_t time = micros();

        if (receiver.missedPackets > 50) {
            receiver.sentTelemetry = false;
            receiver.failsafe = true;
            receiver.rssi = 0;
            receiver.snr = 0;
            receiver.uplinkLQ = 0;
            receiver.freqOffset = 0;
            setRssiDirect(receiver.rssi, RSSI_SOURCE_RX_PROTOCOL);
#ifdef USE_RX_RSSI_DBM
            setRssiDbmDirect(receiver.rssi, RSSI_SOURCE_RX_PROTOCOL);
#endif
#ifdef USE_RX_LINK_QUALITY_INFO
            setLinkQualityDirect(receiver.uplinkLQ);
#endif
            resetLQ();
            receiver.currentFreq = getInitialFreq(receiver.freqOffset);
            elrsRxSetFrequency(receiver.currentFreq); // in conn lost state we always want to listen on freq index 0
            elrsRxStartReceiving();
        } else if ((time - receiver.lastValidPacket) > receiver.mod_params->interval + (receiver.mod_params->interval >> 3)) {
            if (receiver.sentTelemetry) {
                receiver.sentTelemetry = false;
            } else {
                receiver.missedPackets += 1;
                receiver.uplinkLQ = getLQ(false);
            }
            receiver.lastValidPacket += receiver.mod_params->interval;
            receiver.nonceRX += 1;
            setNextChannel();
        }
    } else if (receiver.bound && !receiver.firstConnection && ((millis() - receiver.rfModeLastCycled) > receiver.cycleInterval)) {
        receiver.rfModeLastCycled += receiver.cycleInterval;
        rxExpressLrsSpiConfigMutable()->rateIndex = (rxExpressLrsSpiConfig()->rateIndex + 1) % ELRS_RATE_MAX;
        setRFLinkRate(rxExpressLrsSpiConfig()->rateIndex);

        elrsRxStartReceiving();
    }
}

void expressLrsSetRcDataFromPayload(uint16_t *rcData, const uint8_t *payload)
{
    if (rcData && payload) {
        rxExpressLrsSpiConfig()->hybridSwitches ? unpackChannelDataHybridSwitches(rcData, payload) : unpackChannelData10bit(rcData, payload);
    }
}

rx_spi_received_e expressLrsDataReceived(uint8_t *payload)
{
    rx_spi_received_e result = RX_SPI_RECEIVED_NONE;
    uint32_t timeStamp;

    if (rxSpiCheckBindRequested(true)) {
        enterBindingMode();
    }

    if (elrsRxISR(&timeStamp)) {
        if (receiver.sentTelemetry) {
            elrsRxStartReceiving();
        } else {
            result = processRFPacket(payload, timeStamp);
        }
    }

    DEBUG_SET(DEBUG_RX_EXPRESSLRS_SPI, 0, receiver.missedPackets);
    DEBUG_SET(DEBUG_RX_EXPRESSLRS_SPI, 1, receiver.rssi);
    DEBUG_SET(DEBUG_RX_EXPRESSLRS_SPI, 2, receiver.snr);
    DEBUG_SET(DEBUG_RX_EXPRESSLRS_SPI, 3, receiver.uplinkLQ);

    handleTimeout();

    receiver.bound ? rxSpiLedBlinkRxLoss(result) : rxSpiLedBlinkBind();

    return result;
}

#endif /* USE_RX_EXPRESSLRS */
