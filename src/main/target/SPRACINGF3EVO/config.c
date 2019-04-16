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

#include "flight/mixer.h"
#include "flight/pid.h"

#include "pg/sdcard.h"

#include "config_helper.h"

#if defined(SPRACINGAIRBIT)

static targetSerialPortFunction_t targetSerialPortFunction[] = {
    { SERIAL_PORT_USART1, FUNCTION_RX_SERIAL },
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
#endif
}
#endif
