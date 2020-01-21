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

#pragma once

#include "common/time.h"
#include "drivers/display.h"


//
// General
//

typedef enum {
    MODE_UNKNOWN = 0,
    MODE_PAL,
    MODE_NTSC
} videoMode_t;

extern volatile videoMode_t detectedVideoMode; // unstable value
extern volatile bool cameraConnected;

struct vcdProfile_s;
struct spracingPixelOSDConfig_s;
bool spracingPixelOSDInit(const struct spracingPixelOSDConfig_s *spracingPixelOSDConfig, const struct vcdProfile_s *vcdProfile);
bool spracingPixelOSDIsInitialised(void);
void spracingPixelOSDDrawDebugOverlay(void);
bool spracingPixelOSDShouldProcessNow(timeUs_t currentTimeUs);
void spracingPixelOSDProcess(timeUs_t currentTimeUs);

//
// Layer
//
bool spracingPixelOSDLayerSupported(displayPortLayer_e layer);
bool spracingPixelOSDLayerSelect(displayPortLayer_e layer);
bool spracingPixelOSDLayerCopy(displayPortLayer_e destLayer, displayPortLayer_e sourceLayer);

