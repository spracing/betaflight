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

/*
 * Based on https://github.com/ExpressLRS/ExpressLRS
 * Thanks to AlessandroAU, original creator of the ExpressLRS project.
 *
 * Authors:
 * Phobos- - Original port.
 * Dominic Clifton/Hydra - Timer-based timeout implementation.
 */

#include <string.h>
#include "platform.h"

#ifdef USE_RX_EXPRESSLRS

#include "build/debug.h"
#include "build/debug_pin.h"

#include "common/maths.h"
#include "common/filter.h"

#include "drivers/io.h"
#include "drivers/rx/rx_spi.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "drivers/rx/rx_sx127x.h"
#include "drivers/rx/rx_sx1280.h"
#include "drivers/rx/expresslrs_driver.h"

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
#include "rx/expresslrs_impl.h"

STATIC_UNIT_TESTED elrsReceiver_t receiver;
static const uint8_t BindingUID[6] = {0,1,2,3,4,5}; // Special binding UID values
static uint16_t crcInitializer = 0;

#ifdef USE_RX_RSSI_DBM
static pt1Filter_t rssiFilter;
#endif

#define PACKET_HANDLING_TO_TOCK_ISR_DELAY_US 250

//
// Event pair recorder
//

typedef enum {
    EPR_FIRST,
    EPR_SECOND,
} eprEvent_e;

#define EPR_EVENT_COUNT 2

typedef struct eprState_s {
    uint32_t eventAtUs[EPR_EVENT_COUNT];
    bool eventRecorded[EPR_EVENT_COUNT];
} eprState_t;

eprState_t eprState = {0};

static void expressLrsEPRRecordEvent(eprEvent_e event, uint32_t currentTimeUs)
{
    eprState.eventAtUs[event] = currentTimeUs;
    eprState.eventRecorded[event] = true;
}

static bool expressLrsEPRHaveBothEvents(void)
{
    bool bothEventsRecorded = eprState.eventRecorded[EPR_SECOND] && eprState.eventRecorded[EPR_FIRST];
    return bothEventsRecorded;
}

static int32_t expressLrsEPRGetResult(void)
{
    if (!expressLrsEPRHaveBothEvents()) {
        return 0;
    }

    return (int32_t)(eprState.eventAtUs[EPR_SECOND] - eprState.eventAtUs[EPR_FIRST]);
}

static void expressLrsEPRReset(void)
{
    memset(&eprState, 0, sizeof(eprState_t));
}


//
// Phase Lock
//


#define EPR_INTERNAL EPR_FIRST
#define EPR_EXTERNAL EPR_SECOND

typedef struct phaseLockState_s {
    simpleLowpassFilter_t offsetFilter;
    simpleLowpassFilter_t offsetDxFilter;

    int32_t rawOffsetUs;
    int32_t previousRawOffsetUs;

    int32_t offsetUs;
    int32_t offsetDeltaUs;
    int32_t previousOffsetUs;
} phaseLockState_t;

phaseLockState_t pl;

static void expressLrsPhaseLockReset(void)
{
    simpleLPFilterInit(&pl.offsetFilter, 2, 5);
    simpleLPFilterInit(&pl.offsetDxFilter, 4, 5);

    expressLrsEPRReset();
}

static void expressLrsUpdatePhaseLock(void)
{
    if (!receiver.synced) {
        return;
    }

    if (expressLrsEPRHaveBothEvents()) {
        pl.rawOffsetUs = expressLrsEPRGetResult();

        pl.offsetUs = simpleLPFilterUpdate(&pl.offsetFilter, pl.rawOffsetUs);
        pl.offsetDeltaUs = simpleLPFilterUpdate(&pl.offsetDxFilter, pl.rawOffsetUs - pl.previousRawOffsetUs);

        pl.previousOffsetUs = pl.offsetUs;
        pl.previousRawOffsetUs = pl.rawOffsetUs;

        if (lqPeriodIsSet()) { // RXtimerState == tim_locked && LQCalc.currentIsSet()
            if (receiver.nonceRX % 8 == 0)
            {
                if (pl.offsetUs > 0)
                {
                    expressLrsTimerIncreaseFrequencyOffset();
                }
                else if (pl.offsetUs < 0)
                {
                    expressLrsTimerDecreaseFrequencyOffset();
                }
            }

            if (receiver.failsafe) // really `!connected`, but no connection state management yet.
            {
                expressLrsUpdatePhaseShift(pl.rawOffsetUs >> 1);
            }
            else
            {
                expressLrsUpdatePhaseShift(pl.offsetUs >> 2);
            }

            expressLrsTimerDebug();

            DEBUG_SET(DEBUG_RX_EXPRESSLRS_PHASELOCK, 0, pl.rawOffsetUs);
            DEBUG_SET(DEBUG_RX_EXPRESSLRS_PHASELOCK, 1, pl.offsetUs);
        }
    }

    expressLrsEPRReset();
}

static void transmitTelemetry(void)
{
    uint8_t packet[8];

    packet[0] = ELRS_TLM_PACKET;
    packet[1] = ELRS_TELEMETRY_TYPE_LINK;
    packet[2] = receiver.rssi;
    packet[3] = 0;
    packet[4] = receiver.snr;
    packet[5] = receiver.uplinkLQ;
    packet[6] = 0;

    uint16_t crc = calcCrc14(packet, 7, crcInitializer);
    packet[0] |= (crc >> 6) & 0xFC;
    packet[7] = crc & 0xFF;

    DEBUG_HI(1);
    receiver.dioReason = DIO_TX_DONE;
    receiver.lqMode = LQ_TRANSMITTING;
    receiver.transmitData(packet, ELRS_RX_TX_BUFF_SIZE);
}

static void startReceiving(void)
{
    DEBUG_LO(1);
    receiver.dioReason = DIO_RX_DONE;
    receiver.lqMode = LQ_RECEIVING;
    receiver.startReceiving();
}

static void setNextChannelOrSendTelemetry(void)
{
    if ((receiver.mod_params->fhssHopInterval == 0) || !receiver.bound) {
        return;
    }

    static uint8_t lastNonceRX = 0;

    if (receiver.nonceRX == lastNonceRX) {
        // already done, either because of packet reception or because of tock.
        return;
    }

    lastNonceRX = receiver.nonceRX;

    if (((receiver.nonceRX + 1) % receiver.mod_params->fhssHopInterval) != 0) {
        receiver.handleFreqCorrection(receiver.freqOffset, receiver.currentFreq); //corrects for RX freq offset
    } else {
        receiver.currentFreq = FHSSgetNextFreq(receiver.freqOffset);
        receiver.setFrequency(receiver.currentFreq);
    }

    if (receiver.mod_params->tlmInterval == TLM_RATIO_NO_TLM || (((receiver.nonceRX + 1) % (tlmRatioEnumToValue(receiver.mod_params->tlmInterval))) != 0)) {
        startReceiving();
    } else {
        transmitTelemetry();
    }

}

void expressLrsOnTimerTickISR(void)
{
    expressLrsUpdatePhaseLock();
    receiver.nonceRX += 1;
    receiver.missedPackets += 1;


    receiver.uplinkLQ = lqGet();

#ifdef USE_RX_LINK_QUALITY_INFO
    setLinkQualityDirect(receiver.uplinkLQ);
#endif

    bool shouldStartNewLQPeriod = receiver.lqMode == LQ_RECEIVING;
    if (shouldStartNewLQPeriod) {
        lqNewPeriod();
    }

    if (receiver.lqMode == LQ_TRANSMITTING) {
        // If we just transmitted, the next LQ period should be receiving on the next tick.
        // However, late processing of the DIO TX_DONE IRQ means that it is possible to miss packets and
        // these missed packets must still be counted.
        receiver.lqMode = LQ_RECEIVING;
    }
}

void expressLrsOnTimerTockISR(void)
{
    uint32_t currentTimeUs = micros();

    expressLrsEPRRecordEvent(EPR_INTERNAL, currentTimeUs);

    receiver.nextChannelRequired = true;
}

static void reconfigureRF(void)
{
    receiver.config(receiver.mod_params->bw, receiver.mod_params->sf, receiver.mod_params->cr, receiver.currentFreq, receiver.mod_params->preambleLen, receiver.UID[5] & 0x01);
}

static void setRFLinkRate(const uint8_t index)
{
#if defined(USE_RX_SX1280) && defined(USE_RX_SX127X)
    receiver.mod_params = (rxExpressLrsSpiConfig()->domain == ISM2400) ? &air_rate_config[1][index] : &air_rate_config[0][index];
#else
    receiver.mod_params = &air_rate_config[0][index];
#endif
    receiver.currentFreq = getInitialFreq(receiver.freqOffset);
    // Wait for (11/10) 110% of time it takes to cycle through all freqs in FHSS table (in ms)
    receiver.cycleIntervalMs = ((uint32_t)11U * getFHSSNumEntries() * receiver.mod_params->fhssHopInterval * receiver.mod_params->interval) / (10U * 1000U);

    reconfigureRF();

    expressLrsUpdateTimerInterval(receiver.mod_params->interval);

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
    lqReset();

    receiver.nonceRX = 0;
    receiver.missedPackets = 0;
    receiver.freqOffset = 0;
    receiver.failsafe = false;
    receiver.firstConnection = false;
    receiver.configChanged = false;
    receiver.rssi = 0;
    receiver.snr = 0;
    receiver.uplinkLQ = 0;
    receiver.rateIndex = rxExpressLrsSpiConfig()->rateIndex;
    receiver.packetHandlingToTockDelayUs = PACKET_HANDLING_TO_TOCK_ISR_DELAY_US;
    setRFLinkRate(receiver.rateIndex);

    receiver.rfModeCycledAtMs = millis();
    receiver.configCheckedAtMs = receiver.rfModeCycledAtMs;
    receiver.validPacketReceivedAtUs = micros();
}

static void enterBindingMode(void)
{
    receiver.bound = false;
    receiver.UID = BindingUID;
    crcInitializer = 0;

    receiver.freqOffset = 0;
    receiver.failsafe = false;

    setRFLinkRate(ELRS_RATE_DEFAULT);
    startReceiving();
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
    startReceiving();
}

static rx_spi_received_e processRFPacket(uint8_t *payload, const uint32_t isrTimeStampUs)
{
    UNUSED(isrTimeStampUs);

    uint8_t packet[ELRS_RX_TX_BUFF_SIZE];

    receiver.receiveData(packet, ELRS_RX_TX_BUFF_SIZE);

    uint32_t timeStampUs = micros();

    elrs_packet_type_e type = packet[0] & 0x03;
    uint16_t inCRC = (((uint16_t)(packet[0] & 0xFC)) << 6 ) | packet[7];

    packet[0] = type;
    uint16_t calculatedCRC = calcCrc14(packet, 7, crcInitializer);

    if (inCRC != calculatedCRC) {
        return RX_SPI_RECEIVED_NONE;
    }

    uint8_t indexIN;
    elrs_tlm_ratio_e tlmRateIn;
    uint8_t switchEncMode;


    expressLrsEPRRecordEvent(EPR_EXTERNAL, timeStampUs + receiver.packetHandlingToTockDelayUs);

    receiver.validPacketReceivedAtUs = timeStampUs;
    receiver.missedPackets = 0;
    receiver.failsafe = false;
    lqIncrease();
    receiver.getRFlinkInfo(&receiver.rssi, &receiver.snr);
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
        if (receiver.rateIndex != rxExpressLrsSpiConfig()->rateIndex) {
            rxExpressLrsSpiConfigMutable()->rateIndex = receiver.rateIndex;
            receiver.configChanged = true;
        }
    }

    bool shouldStartTimer = false;

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
            receiver.synced = true;
            indexIN = (packet[3] & 0xC0) >> 6;
            tlmRateIn = (packet[3] & 0x38) >> 3;
            switchEncMode = (packet[3] & 0x06) >> 1;

            if (packet[4] == receiver.UID[3] && packet[5] == receiver.UID[4] && packet[6] == receiver.UID[5]) {
                if (switchEncMode != rxExpressLrsSpiConfig()->hybridSwitches) {
                    rxExpressLrsSpiConfigMutable()->hybridSwitches = switchEncMode;
                    receiver.configChanged = true;
                }

                if (receiver.mod_params->index == indexIN ) {
                    if (receiver.mod_params->tlmInterval != tlmRateIn) { // change link parameters if required
                        receiver.mod_params->tlmInterval = tlmRateIn;
                    }

                    if (receiver.nonceRX != packet[2] || FHSSgetCurrIndex() != packet[1]) {
                        FHSSsetCurrIndex(packet[1]);
                        receiver.nonceRX = packet[2];

                        if (!expressLrsTimerIsRunning()) {
                            shouldStartTimer = true;
                        }
                    }
                }
            }
            break;
        default:
            return RX_SPI_RECEIVED_NONE;
    }

    if (shouldStartTimer) {
        expressLrsTimerResume();
    }

    receiver.nextChannelRequired = true;

    return RX_SPI_RECEIVED_DATA;
}

#ifdef USE_RX_SX1280
static inline void configureReceiverForSX1280(void)
{
    receiver.init = (elrsRxInitFnPtr) sx1280Init;
    receiver.config = (elrsRxConfigFnPtr) sx1280Config;
    receiver.startReceiving = (elrsRxStartReceivingFnPtr) sx1280StartReceiving;
    receiver.rxISR = (elrsRxISRFnPtr) sx1280ISR;
    receiver.transmitData = (elrsRxTransmitDataFnPtr) sx1280TransmitData;
    receiver.receiveData = (elrsRxReceiveDataFnPtr) sx1280ReceiveData;
    receiver.getRFlinkInfo = (elrsRxGetRFlinkInfoFnPtr) sx1280GetLastPacketStats;
    receiver.setFrequency = (elrsRxSetFrequencyFnPtr) sx1280SetFrequencyReg;
    receiver.handleFreqCorrection = (elrsRxHandleFreqCorrectionFnPtr) sx1280AdjustFrequency;
    receiver.isBusy = (elrsRxIsBusyFnPtr) sx1280IsBusy;
}
#endif

#ifdef USE_RX_SX127X

bool neverBusy(void) {
    return false;
}

static inline void configureReceiverForSX127x(void)
{
    receiver.init = (elrsRxInitFnPtr) sx127xInit;
    receiver.config = (elrsRxConfigFnPtr) sx127xConfig;
    receiver.startReceiving = (elrsRxStartReceivingFnPtr) sx127xStartReceiving;
    receiver.rxISR = (elrsRxISRFnPtr) sx127xISR;
    receiver.transmitData = (elrsRxTransmitDataFnPtr) sx127xTransmitData;
    receiver.receiveData = (elrsRxReceiveDataFnPtr) sx127xReceiveData;
    receiver.getRFlinkInfo = (elrsRxGetRFlinkInfoFnPtr) sx127xGetLastPacketStats;
    receiver.setFrequency = (elrsRxSetFrequencyFnPtr) sx127xSetFrequencyReg;
    receiver.handleFreqCorrection = (elrsRxHandleFreqCorrectionFnPtr) sx127xAdjustFrequency;
    receiver.isBusy = neverBusy;
}
#endif

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

    switch (rxExpressLrsSpiConfig()->domain) {
#ifdef USE_RX_SX127X
        case AU433:
            FALLTHROUGH;
        case AU915:
            FALLTHROUGH;
        case EU433:
            FALLTHROUGH;
        case EU868:
            FALLTHROUGH;
        case FCC915:
            configureReceiverForSX127x();
            break;
#endif
#ifdef USE_RX_SX1280
        case ISM2400:
            configureReceiverForSX1280();
            break;
#endif
        default:
            return false;
    }

    if (!receiver.init(receiver.resetPin, receiver.busyPin)) {
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

    expressLrsPhaseLockReset();

    expressLrsInitialiseTimer(RX_EXPRESSLRS_TIMER_INSTANCE, &receiver.timerUpdateCb);
    expressLrsTimerStop();

    generateCrc14Table();
    initializeReceiver();

    // Timer IRQs must only be enabled after the receiver is configured otherwise race conditions occur.
    expressLrsTimerEnableIRQs();

    startReceiving();

    return true;
}



static void handleTimeout(void)
{
    if (!receiver.failsafe) {

        if ((micros() - receiver.validPacketReceivedAtUs) > (receiver.mod_params->failsafeIntervalUs)) {
            // FAILSAFE!
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
            lqReset();

            expressLrsPhaseLockReset();
            expressLrsTimerStop();

            receiver.synced = false;
            receiver.failsafe = true;

            // in connection lost state we want to listen on the frequency that sync packets are expected to appear on.
            receiver.currentFreq = getInitialFreq(receiver.freqOffset);

            reconfigureRF();

            startReceiving();
        }
    } else if (receiver.bound && !receiver.firstConnection && ((millis() - receiver.rfModeCycledAtMs) > receiver.cycleIntervalMs)) {
        receiver.rfModeCycledAtMs += receiver.cycleIntervalMs;
        receiver.rateIndex = (receiver.rateIndex + 1) % ELRS_RATE_MAX;
        setRFLinkRate(receiver.rateIndex);

        expressLrsPhaseLockReset();

        startReceiving();
    }
}

static void handleConfigUpdate(void)
{
    const uint32_t time = millis();

    if ((time - receiver.configCheckedAtMs) > ELRS_CONFIG_CHECK_MS) {
        receiver.configCheckedAtMs = time;
        if (receiver.configChanged) {
            writeEEPROM();
            receiver.configChanged = false;
        }
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
    uint32_t isrTimeStampUs;

    if (rxSpiCheckBindRequested(true)) {
        enterBindingMode();
    }

    if (receiver.rxISR(&isrTimeStampUs)) {

        // It's important to note that the DIO reason, and LQ mode are tracked separately as the task can run late.
        // When a task runs late the reason for the DIO might not match the current LQ mode.  i.e. if telemetry is sent
        // then the task is late the LQ mode should be LQ_RECEIVING but the reason for the DIO will be DIO_TX_DONE.

        // Note: it is possible to read the IRQ state from the receiver to find out the cause of the EXTI (DIO) trigger
        // (e.g. see SX1280_RADIO_GET_IRQSTATUS), but instead we maintain state to avoid having to read from the device via SPI.

        if (receiver.dioReason == DIO_TX_DONE) {
            startReceiving();
        } else {
            result = processRFPacket(payload, isrTimeStampUs);
        }
    }

    if (receiver.nextChannelRequired) {
        receiver.nextChannelRequired = false;
        if (receiver.synced) {
            setNextChannelOrSendTelemetry();
        }
    }

    DEBUG_SET(DEBUG_RX_EXPRESSLRS_SPI, 0, receiver.missedPackets);
    DEBUG_SET(DEBUG_RX_EXPRESSLRS_SPI, 1, receiver.rssi);
    DEBUG_SET(DEBUG_RX_EXPRESSLRS_SPI, 2, receiver.snr);
    DEBUG_SET(DEBUG_RX_EXPRESSLRS_SPI, 3, receiver.uplinkLQ);

    handleTimeout();
    handleConfigUpdate();

    receiver.bound ? rxSpiLedBlinkRxLoss(result) : rxSpiLedBlinkBind();

    return result;
}

#endif /* USE_RX_EXPRESSLRS */
