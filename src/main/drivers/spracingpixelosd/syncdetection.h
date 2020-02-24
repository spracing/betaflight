/*
 * Author: Dominic Clifton - Sync generation, Sync Detection, Video Overlay and first-cut of working OSD system.
 */

#pragma once

typedef struct syncDetectionState_s {
    uint32_t minimumLevelForLineThreshold;
    uint32_t minimumLevelForValidFrameMv;
    uint32_t maximumLevelForValidFrameMv;
    uint32_t minMaxDifference;
    uint32_t syncThresholdMv;
    uint32_t lineCounterAtStart;

    timeUs_t syncStartedAt;
    timeUs_t syncCompletedAt;
    timeUs_t syncDuration;

} syncDetectionState_t;

extern syncDetectionState_t syncDetectionState;

void syncDetection_reset(void);

void spracingPixelOSDRefreshState(timeUs_t currentTimeUs);
void spracingPixelOSDService(timeUs_t currentTimeUs);
