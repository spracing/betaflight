/*
 * Author: Dominic Clifton - Sync generation, Sync Detection, Video Overlay and first-cut of working OSD system.
 */

#pragma once

#ifdef BETAFLIGHT
#include "drivers/display.h"
#endif

//
// General
//

extern volatile bool cameraConnected;

//
// PUBLIC API
//

#define PIXELOSD_FLAG_INITIALISED       (1 << 0)
#define PIXELOSD_FLAG_ERROR             (1 << 1)
#define PIXELOSD_FLAG_SERVICE_REQUIRED  (1 << 2)
#define PIXELOSD_FLAG_FRAME_START       (1 << 3)

#define PIXELOSD_CF_VIDEO_SYSTEM_PAL    (1 << 0)
#define PIXELOSD_CF_VIDEO_SYSTEM_NTSC   (0 << 1)

typedef struct pixelOSDState_s {
    uint32_t flags;
} pixelOSDState_t;

typedef struct pixelOSDHostAPI_s {
  uint32_t (*micros)(void);
} pixelOSDHostAPI_t;

typedef struct pixelOSDDefaultConfig_s {
  uint32_t flags;
} pixelOSDDefaultConfig_t;

typedef struct pixelOSDAPIVTable_s {
  void (*init)(const pixelOSDHostAPI_t *hostAPI, const pixelOSDDefaultConfig_t *defaultConfig);
  pixelOSDState_t *(*getState)(void);
  void (*refreshState)(uint32_t currentTimeUs);
  void (*service)(uint32_t currentTimeUs);
  void (*renderDebugOverlay)(void);
} pixelOSDAPIVTable_t;

typedef struct pixelOSDClientAPI_s {
    const uint32_t osdVersion;
    const uint32_t apiVersion;
    const pixelOSDAPIVTable_t *vTable;
} pixelOSDClientAPI_t;

const pixelOSDClientAPI_t *spracingPixelOSDGetAPI(void);

#ifdef BETAFLIGHT
//
// Layer
//
bool spracingPixelOSDLayerSupported(displayPortLayer_e layer);
bool spracingPixelOSDLayerSelect(displayPortLayer_e layer);
bool spracingPixelOSDLayerCopy(displayPortLayer_e destLayer, displayPortLayer_e sourceLayer);
#endif
