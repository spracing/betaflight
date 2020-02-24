/*
 * Author: Dominic Clifton - Sync generation, Sync Detection, Video Overlay and first-cut of working OSD system.
 */

#pragma once

typedef enum {
    VIDEO_SYSTEM_UNKNOWN = 0,
    VIDEO_SYSTEM_PAL,
    VIDEO_SYSTEM_NTSC
} videoSystem_t;

extern volatile videoSystem_t detectedVideoSystem; // unstable value
