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
 * Dominic Clifton/Hydra - Timer-based timeout implementation.
 */

#include <string.h>
#include "platform.h"

#ifdef USE_RX_EXPRESSLRS

#include "build/debug_pin.h"

#include "drivers/timer.h"

#include "drivers/nvic.h"

#include "rx/expresslrs_common.h"
#include "rx/expresslrs_impl.h"

#define TIMER_INTERVAL_US_DEFAULT 20000

static void expressLrsOnTimerUpdate(timerOvrHandlerRec_t *cbRec, captureCompare_t capture)
{
    UNUSED(cbRec);
    UNUSED(capture);

    static bool pinState = true;
    pinState = !pinState;
    if (pinState) {
        DEBUG_HI(0);
    } else {
        DEBUG_LO(0);
    }

    elrsReceiver_t *self = container_of(cbRec, elrsReceiver_t, timerUpdateCb);
    UNUSED(self);
}

void expressLrsInitialiseTimer(elrsReceiver_t *receiver)
{
    receiver->timer = RX_EXPRESSLRS_TIMER_INSTANCE;

    configTimeBase(receiver->timer, TIMER_INTERVAL_US_DEFAULT, MHZ_TO_HZ(1));

    timerChOvrHandlerInit(&receiver->timerUpdateCb, expressLrsOnTimerUpdate);

    timerConfigUpdateCallback(receiver->timer, &receiver->timerUpdateCb);

    uint8_t irq = timerInputIrq(receiver->timer);

    // Use the NVIC TIMER priority for now
    HAL_NVIC_SetPriority(irq, NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER), NVIC_PRIORITY_SUB(NVIC_PRIO_TIMER));
    HAL_NVIC_EnableIRQ(irq);

    LL_TIM_EnableCounter(receiver->timer);
}

#endif
