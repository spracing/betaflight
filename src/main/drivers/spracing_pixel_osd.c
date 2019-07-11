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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SPRACING_PIXEL_OSD


// Pins 8-15 of GPIOE reserved for OSD use when in GPIO OUTPUT MODE (Upper 8 bits of GPIO port)
// Pins 8-15 on GPIOE *can* be used for other functions, just not GPIO OUTPUT.
#define SPRACING_PIXEL_OSD_BLACK_PIN                    PE13
#define SPRACING_PIXEL_OSD_WHITE_PIN                    PE14
#define SPRACING_PIXEL_OSD_RESERVED_PIN                 PE15

#define SPRACING_PIXEL_OSD_SYNC_IN_PIN                  PE11 // COMP2_INP
#define SPRACING_PIXEL_OSD_SYNC_OUT_A_PIN               PE12 // TIM1_CH3N
#define SPRACING_PIXEL_OSD_SYNC_OUT_B_PIN               PA8  // TIM1_CH1 / MCO

#define SPRACING_PIXEL_OSD_VIDEO_THRESHOLD_DEBUG_PIN    PA5
#define SPRACING_PIXEL_OSD_PIXEL_DEBUG_1_PIN            PE5  // TIM15_CH1 - For DMA updates
#define SPRACING_PIXEL_OSD_PIXEL_DEBUG_2_PIN            PE6  // TIM15_CH2 - Spare


#include "build/debug.h"

#include "pg/spracing_pixel_osd.h"
#include "pg/vcd.h"

#include "drivers/io.h"
//#include "drivers/light_led.h"
//#include "drivers/nvic.h"
//#include "drivers/time.h"

typedef struct spracingPixelOSDIO_s {
    IO_t blackPin;
    IO_t whitePin;
    IO_t syncInPin;
} spracingPixelOSDIO_t;

static spracingPixelOSDIO_t spracingPixelOSDIO = {
    .blackPin           = IO_NONE,
    .whitePin           = IO_NONE,
    .syncInPin          = IO_NONE,
};

#define IO_PIXEL_BLACK_CFG      IO_CONFIG(GPIO_MODE_OUTPUT_OD, GPIO_SPEED_FREQ_MEDIUM,  GPIO_NOPULL)
#define IO_PIXEL_WHITE_CFG      IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_MEDIUM,  GPIO_NOPULL)

#define IO_VIDEO_SYNC_IN_CFG       IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_LOW,     GPIO_NOPULL)

bool spracingPixelOSDInit(const struct spracingPixelOSDConfig_s *spracingPixelOSDConfig, const struct vcdProfile_s *vcdProfile)
{
    spracingPixelOSDIO.blackPin = IOGetByTag(IO_TAG(SPRACING_PIXEL_OSD_BLACK_PIN));
    IOHi(spracingPixelOSDIO.blackPin);
    IOInit(spracingPixelOSDIO.blackPin, OWNER_OSD, 0);
    IOConfigGPIO(spracingPixelOSDIO.blackPin, IO_PIXEL_BLACK_CFG);

    spracingPixelOSDIO.whitePin = IOGetByTag(IO_TAG(SPRACING_PIXEL_OSD_WHITE_PIN));
    IOLo(spracingPixelOSDIO.whitePin);
    IOInit(spracingPixelOSDIO.whitePin, OWNER_OSD, 0);
    IOConfigGPIO(spracingPixelOSDIO.whitePin, IO_PIXEL_WHITE_CFG);

    spracingPixelOSDIO.syncInPin = IOGetByTag(IO_TAG(SPRACING_PIXEL_OSD_SYNC_IN_PIN));
    IOLo(spracingPixelOSDIO.syncInPin);
    IOInit(spracingPixelOSDIO.syncInPin, OWNER_OSD, 0);
    IOConfigGPIO(spracingPixelOSDIO.syncInPin, IO_VIDEO_SYNC_IN_CFG);

    return false;
}

#endif // USE_SPRACING_PIXEL_OSD
