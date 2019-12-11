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

#include "platform.h"

#ifdef USE_TARGET_CONFIG

#include "common/axis.h"
#include "common/maths.h"

#include "flight/pid.h"
#include "flight/servos.h"
#include "fc/rc_modes.h"

#include "pg/sdcard.h"
#include "pg/motor.h"

#include "config_helper.h"

#if defined(SPRACINGAIRBIT)

static targetSerialPortFunction_t targetSerialPortFunction[] = {
    { SERIAL_PORT_USART1, FUNCTION_RX_SERIAL | FUNCTION_TELEMETRY_LTM},
};

typedef struct targetServoParam_t {
    uint8_t servoIndex;
    int16_t min;
    int16_t max;
    int16_t middle;
    int8_t rate;
    int8_t forwardFromChannel;
} targetServoParam_t;

static targetServoParam_t targetServoParams[] = {
    { 0, 1000, 2000, 1500, 100, 6 },
    { 1, 1000, 2000, 1500, 100, 5 },
};

typedef struct targetActivationCondition_s {
    uint8_t index;
    boxId_e modeId;
    uint8_t auxChannelIndex;
    uint16_t startValue;
    uint16_t endValue;
    modeLogic_e modeLogic;
    boxId_e linkedTo;
} targetModeActivationCondition_t;

static targetModeActivationCondition_t targetModeActivationConditions[] = {
    {0, 0, 0, 1650, 2100, 0, 0},
    {1, 1, 0, 900, 2100, 0, 0},
};

#endif

#if defined(SPRACINGF3MQ) || defined(SPRACINGAIRBIT)
#ifdef BRUSHED_MOTORS_PWM_RATE
#undef BRUSHED_MOTORS_PWM_RATE
#endif

#define BRUSHED_MOTORS_PWM_RATE 32000           // 32kHz
#endif

void targetConfiguration(void)
{
#if defined(SPRACINGF3MQ) || defined(SPRACINGAIRBIT)

    motorConfigMutable()->dev.motorPwmRate = BRUSHED_MOTORS_PWM_RATE;

    for (uint8_t pidProfileIndex = 0; pidProfileIndex < PID_PROFILE_COUNT; pidProfileIndex++) {
        pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);

        pidProfile->pid[FD_ROLL].P = 90;
        pidProfile->pid[FD_ROLL].I = 44;
        pidProfile->pid[FD_ROLL].D = 60;
        pidProfile->pid[FD_PITCH].P = 90;
        pidProfile->pid[FD_PITCH].I = 44;
        pidProfile->pid[FD_PITCH].D = 60;
    }
#else
    // Temporary workaround: Disable SDCard DMA by default since it causes error with DSHOT, brushed targets can use DMA.
    sdcardConfigMutable()->useDma = false;
#endif

#ifdef SPRACINGAIRBIT
    targetSerialPortFunctionConfig(targetSerialPortFunction, ARRAYLEN(targetSerialPortFunction));

    serialPortConfig_t *ltmUART = serialFindPortConfiguration(SERIAL_PORT_USART1);
    if (ltmUART) {
        ltmUART->telemetry_baudrateIndex = BAUD_115200;
    }

    for (uint32_t i = 0; i < ARRAYLEN(targetServoParams); i++) {
        targetServoParam_t *params = &targetServoParams[i];

        uint32_t servoIndex = params->servoIndex;
        servoParam_t *servo = servoParamsMutable(servoIndex);
        servo->min = params->min;
        servo->max = params->max;
        servo->middle = params->middle;
        servo->rate = params->rate;
        servo->forwardFromChannel = params->forwardFromChannel;
    }

    for (uint32_t i = 0; i < ARRAYLEN(targetModeActivationConditions); i++) {
        targetModeActivationCondition_t *tmac = &targetModeActivationConditions[i];

        uint32_t tmacIndex = tmac->index;
        modeActivationCondition_t *mac = modeActivationConditionsMutable(tmacIndex);
        mac->modeId = tmac->modeId;
        mac->auxChannelIndex = tmac->auxChannelIndex;
        mac->range.startStep = CHANNEL_VALUE_TO_STEP(tmac->startValue);
        mac->range.endStep = CHANNEL_VALUE_TO_STEP(tmac->endValue);
        mac->modeLogic = tmac->modeLogic;
        mac->linkedTo = tmac->linkedTo;
    }

    servoConfigMutable()->channelForwardingStartChannel = 5;

#endif
}
#endif
