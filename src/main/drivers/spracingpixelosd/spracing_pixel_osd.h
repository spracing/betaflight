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
