/*
 * Author: Dominic Clifton - Sync generation, Sync Detection, Video Overlay and first-cut of working OSD system.
 */

#pragma once

#include "videosystem.h"

void syncInit(void);
void configureSyncGeneration(videoSystem_t videoSystem);

void syncStartDMA(void);
void syncStopDMA(void);

void syncStartPWM(void);
void syncStopPWM(void);
