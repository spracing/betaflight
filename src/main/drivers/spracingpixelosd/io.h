/*
 * Author: Dominic Clifton - Sync generation, Sync Detection, Video Overlay and first-cut of working OSD system.
 */

#pragma once

#include "configuration.h"

void spracingPixelOSD_initIO(void);

//
// OSD Pin Debugging
//
void pixelDebug1Set(bool state);
void pixelDebug2Set(bool state);
void pixelDebug1Low(void);
void pixelDebug2Low(void);
void pixelDebug1High(void);
void pixelDebug2High(void);
void pixelDebug1Toggle(void);
void pixelDebug2Toggle(void);

