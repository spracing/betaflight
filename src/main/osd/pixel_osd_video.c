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
 * Author: Dominic Clifton
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_PIXEL_OSD

#if 1
#define DISPLAY_FRAME_COUNTER
#endif

#include "build/build_config.h"

#include "common/printf.h"

#include "drivers/spracing_pixel_osd.h"

#include "pixel_osd_video.h"

FAST_CODE bool taskPixelOSDVideoCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs)
{
    return (fillLineNow || frameFlag);
}

FAST_CODE void taskPixelOSDVideo(timeUs_t currentTimeUs)
{

    if (fillLineNow) {
        fillLineNow = false;
        pixelBuffer_fillFromFrameBuffer(fillPixelBuffer, 0, fillLineIndex);
        //pixelBuffer_update();
    }

    if (frameFlag) {
        frameFlag = false;

#ifdef DISPLAY_FRAME_COUNTER
        uint8_t *frameBuffer = frameBuffer_getBuffer(0);
        uint16_t frameCounter = frameBuffer_getCounter();

        static uint8_t frameCountBuffer[7];
        tfp_sprintf((char *)frameCountBuffer, "F:%04x", frameCounter);
        frameBuffer_slowWriteString(frameBuffer, (320 - (12 * 6)) / 2, (288 - 18) / 2, frameCountBuffer, 6);
#endif
    }
}

#endif
