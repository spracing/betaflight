/*
 * Author: Dominic Clifton - Sync generation, Sync Detection, Video Overlay and first-cut of working OSD system.
 */

#include <stdbool.h>
#include <string.h>

#include "platform.h"

#ifdef BETAFLIGHT
#include "build/debug.h"

#include "common/time.h"
#endif

#include "configuration.h"
#include "videotiming.h"
#include "videoframe.h"
#include "spracing_pixel_osd_impl.h"
#include "spracing_pixel_osd.h"

#include "syncdetection.h"

typedef enum {
    OUTPUT_DISABLED = 0,
    SEARCHING_FOR_LINE_MIN_LEVEL,
    SEARCHING_FOR_FRAME_MIN_LEVEL,
    SEARCHING_FOR_LINE_MAX_LEVEL,
    GENERATING_VIDEO,
} pixelOsdVideoState_t;

pixelOsdVideoState_t pixelOsdVideoState = SEARCHING_FOR_LINE_MIN_LEVEL;

#ifdef DEBUG_OSD_EVENTS
typedef struct eventLogItem_s {
    timeUs_t us;
    pixelOsdVideoState_t state;
} eventLogItem_t;

eventLogItem_t eventLog[256] = {0};
unsigned int eventLogIndex = 0;

void logEvent(timeUs_t us, pixelOsdVideoState_t state)
{
    eventLogIndex++;
    if (eventLogIndex >= ARRAYLEN(eventLog)) {
        eventLogIndex = 0;
    }

    eventLogItem_t *item = &eventLog[eventLogIndex];
    item->state = state;
    item->us = us;
}
#else
#define logEvent(us, state) {}
#endif

#define MAXIMIM_LINE_LEVEL_THRESHOLD_MV 2000
#define MAXIMIM_FRAME_LEVEL_THRESHOLD_MV (MAXIMIM_LINE_LEVEL_THRESHOLD_MV + 1000)
#define MAXIMIM_FRAME_LEVEL_DIFFERENCE_MV 400 // was 300
#define MAXIMIM_LINE_LEVEL_DIFFERENCE_MV 300


extern pixelOSDState_t spracingPixelOSDState;
extern volatile bool vSyncFlag;
extern volatile bool fieldSyncFlag;

uint32_t pulseErrorsPerSecond = 0;
uint32_t framesPerSecond = 0;

syncDetectionState_t syncDetectionState = { 0 };
static uint32_t nextEventAt = 0;
static timeUs_t previousServiceAt = 0;

void syncDetection_reset(void)
{
    memset(&syncDetectionState, 0x00, sizeof(syncDetectionState));
}

#define DELAY_60_HZ (1000000 / 60)

void spracingPixelOSDRefreshState(timeUs_t currentTimeUs)
{
    timeDelta_t serviceDeltaUs = currentTimeUs - previousServiceAt;

    bool deltaTooLong = serviceDeltaUs > DELAY_60_HZ; // FIXME use current video mode, (50/60hz)
    bool handleEventNow = cmp32(currentTimeUs, nextEventAt) > 0;

    spracingPixelOSDState.flags = 0;

    if (deltaTooLong || handleEventNow) {
      spracingPixelOSDState.flags |= PIXELOSD_FLAG_SERVICE_REQUIRED;
    }

    if (vSyncFlag) {
      spracingPixelOSDState.flags |= PIXELOSD_FLAG_VSYNC;

      vSyncFlag = false;
    }

    if (fieldSyncFlag) {
      spracingPixelOSDState.flags |= PIXELOSD_FLAG_FIELD_SYNC;

      fieldSyncFlag = false;
    }

}

void spracingPixelOSDService(timeUs_t currentTimeUs)
{

    const uint16_t requiredLines = 100; // ~64us * 100 = 6.4ms

    const uint32_t lineCounterDelayUs = (videoTimings->lineNs / 1000) * (requiredLines);
    const uint32_t minimumFrameDelayUs = (videoTimings->lineNs / 1000) * (videoTimings->lineCount + 10);

    DEBUG_SET(DEBUG_SPRACING_PIXEL_OSD, 0, frameState.validFrameCounter);
    DEBUG_SET(DEBUG_SPRACING_PIXEL_OSD, 0, frameState.totalPulseErrors);

    static uint8_t syncDetectionFailureCount = 0;

    switch(pixelOsdVideoState) {
        case OUTPUT_DISABLED:
        {
            if (nextEventAt == 0) {
                // state transition
                nextEventAt = currentTimeUs + (1000000/4); // 1/4 of a second
                syncDetectionFailureCount++;

                if (syncDetectionFailureCount > 1) {


                    spracingPixelOSDPause();


                    if (cameraConnected) {
                        cameraConnected = false;
                    } else {
                        cameraConnected = true;
                    }

                    spracingPixelOSDRestart();

                    syncDetectionFailureCount = 0;

                    pixelOsdVideoState = SEARCHING_FOR_LINE_MIN_LEVEL;
                    nextEventAt = 0;
                }
            }

            bool handleEventNow = cmp32(currentTimeUs, nextEventAt) > 0;
            if (handleEventNow) {
                logEvent(currentTimeUs, pixelOsdVideoState);

                pixelOsdVideoState = SEARCHING_FOR_LINE_MIN_LEVEL;
                nextEventAt = 0;
            }
            break;
        }
        case SEARCHING_FOR_LINE_MIN_LEVEL:
        {
            if (nextEventAt == 0) {
                // state transition
                syncDetectionState.syncStartedAt = currentTimeUs;
                syncDetectionState.syncCompletedAt = 0;
                syncDetectionState.syncDuration = 0;


                syncDetectionState.minimumLevelForLineThreshold = 0;
                syncDetectionState.minimumLevelForValidFrameMv = 0;
                syncDetectionState.maximumLevelForValidFrameMv = 0;

                syncDetectionState.lineCounterAtStart = frameState.lineCounter;

                // always reset comparator target.
                setComparatorTargetMv(syncDetectionState.minimumLevelForLineThreshold);

                disableComparatorBlanking();

                nextEventAt = currentTimeUs + lineCounterDelayUs;
            }

            bool handleEventNow = cmp32(currentTimeUs, nextEventAt) > 0;

            if (handleEventNow) {
                logEvent(currentTimeUs, pixelOsdVideoState);

                uint32_t linesSinceStart = frameState.lineCounter - syncDetectionState.lineCounterAtStart;

                // FIXME this algorithm is dependent on the task being scheduled exactly.  Ideally we should calculate the time delta and base the
                // value below on the amount of lines that could have actually been seen, and not `requiredLines`
                bool lineThesholdAchieved = linesSinceStart >= requiredLines / 4;

                if (!lineThesholdAchieved) {
                    if (syncDetectionState.minimumLevelForLineThreshold < MAXIMIM_LINE_LEVEL_THRESHOLD_MV) {
                        syncDetectionState.minimumLevelForLineThreshold += 5;
                        setComparatorTargetMv(syncDetectionState.minimumLevelForLineThreshold);

                        nextEventAt = currentTimeUs + lineCounterDelayUs;
                        syncDetectionState.lineCounterAtStart = frameState.lineCounter;
                    } else {
                        pixelOsdVideoState = OUTPUT_DISABLED;
                        nextEventAt = 0;
                    }
                } else {
                    pixelOsdVideoState = SEARCHING_FOR_FRAME_MIN_LEVEL;
                    nextEventAt = 0;
                }

            }
            break;
        }
        case SEARCHING_FOR_FRAME_MIN_LEVEL:
        {
            if (nextEventAt == 0) {
                // state transition
                syncDetectionState.minimumLevelForValidFrameMv = syncDetectionState.minimumLevelForLineThreshold;
                nextEventAt = currentTimeUs + minimumFrameDelayUs;
            }

            bool handleEventNow = cmp32(currentTimeUs, nextEventAt) > 0;

            if (handleEventNow) {
                logEvent(currentTimeUs, pixelOsdVideoState);

                if (frameState.validFrameCounter == 0) {

                    uint32_t minMaxDifference = syncDetectionState.minimumLevelForValidFrameMv - syncDetectionState.minimumLevelForLineThreshold;

                    if (syncDetectionState.minimumLevelForValidFrameMv < MAXIMIM_FRAME_LEVEL_THRESHOLD_MV && minMaxDifference < MAXIMIM_LINE_LEVEL_DIFFERENCE_MV) {
                        syncDetectionState.minimumLevelForValidFrameMv += 5;
                        setComparatorTargetMv(syncDetectionState.minimumLevelForValidFrameMv);
                        nextEventAt = currentTimeUs + minimumFrameDelayUs;
                    } else {
                        pixelOsdVideoState = OUTPUT_DISABLED;
                        nextEventAt = 0;
                    }

                } else {
                    pixelOsdVideoState = SEARCHING_FOR_LINE_MAX_LEVEL;
                    nextEventAt = 0;
                }
            }
            break;
        }
        case SEARCHING_FOR_LINE_MAX_LEVEL:
        {
            static uint32_t lineCounterAtStart;

            if (nextEventAt == 0) {
                // state transition
                syncDetectionState.maximumLevelForValidFrameMv = syncDetectionState.minimumLevelForValidFrameMv + 5;
                setComparatorTargetMv(syncDetectionState.maximumLevelForValidFrameMv);

                nextEventAt = currentTimeUs + lineCounterDelayUs;
                lineCounterAtStart = frameState.lineCounter; // store the line counter as late as possible, so that less ISR's occur between now and the next event
            }

            bool handleEventNow = cmp32(currentTimeUs, nextEventAt) > 0;

            if (handleEventNow) {

                // calculate lines seen NOW, since the lineCounter will change via the ISR.
                int32_t linesSeen = frameState.lineCounter - lineCounterAtStart;

                logEvent(currentTimeUs, pixelOsdVideoState);

#if 0 // XXX
                int32_t timeDeltaUs = cmp32(currentTimeUs, nextEventAt);
                int32_t maximumLines = timeDeltaUs / (videoTimings->lineNs / 1000);
                // in practice, if this method isn't scheduled exactly on time, this doesn't work; linesSeen is frequently more than maximumLines.
                // suspect this code would need to move into the ISR itself for accuracy if the exact amount of lines seen is needed.
#endif

                bool levelOk = false;

                int8_t possibleSyncPulseCount = detectedVideoSystem == VIDEO_SYSTEM_NTSC ? VIDEO_NTSC_TOTAL_HSYNC_PULSES : VIDEO_PAL_TOTAL_HSYNC_PULSES;

                if (linesSeen > (requiredLines - possibleSyncPulseCount)) {
                    // still getting valid frames, increase targetMv
                  syncDetectionState.minMaxDifference = syncDetectionState.maximumLevelForValidFrameMv - syncDetectionState.minimumLevelForValidFrameMv;

                    if (syncDetectionState.minimumLevelForValidFrameMv < MAXIMIM_FRAME_LEVEL_THRESHOLD_MV && syncDetectionState.minMaxDifference < MAXIMIM_FRAME_LEVEL_DIFFERENCE_MV) {
                        syncDetectionState.maximumLevelForValidFrameMv += 10;
                        setComparatorTargetMv(syncDetectionState.maximumLevelForValidFrameMv);

                        // start again using current frame counter.
                        lineCounterAtStart = frameState.lineCounter;
                        nextEventAt = currentTimeUs + lineCounterDelayUs;
                    } else {
                        // use the current level, since we reached the upper threshold and we are still getting valid frames.
                        levelOk = true;
                    }
                } else {
                    // no valid frame received, use the previous level
                    syncDetectionState.maximumLevelForValidFrameMv -= 5;
                    levelOk = true;
                }

                if (levelOk) {
                    syncDetectionState.minMaxDifference = syncDetectionState.maximumLevelForValidFrameMv - syncDetectionState.minimumLevelForValidFrameMv;
                    syncDetectionState.syncThresholdMv = syncDetectionState.minimumLevelForValidFrameMv + (0.4 * syncDetectionState.minMaxDifference);

                    setComparatorTargetMv(syncDetectionState.syncThresholdMv);

                    setVideoSourceVoltageMv(syncDetectionState.syncThresholdMv + 1700);

                    pixelOsdVideoState = GENERATING_VIDEO;

                    syncDetectionState.syncCompletedAt = currentTimeUs;
                    syncDetectionState.syncDuration = syncDetectionState.syncCompletedAt - syncDetectionState.syncStartedAt;
                    nextEventAt = 0;
                }
            }

            break;
        }
        case GENERATING_VIDEO:
        {
            static uint32_t lastTimeUs;
            static uint32_t lastTotalPulseErrors;
            static uint32_t lastValidFrameCounter;
            static bool errorDetectionEnabled;

            if (nextEventAt == 0) {
                // state transition
                lastTimeUs = currentTimeUs;
                lastTotalPulseErrors = frameState.totalPulseErrors;
                lastValidFrameCounter = frameState.validFrameCounter;
                errorDetectionEnabled = false;
                syncDetectionFailureCount = 0;

                nextEventAt = currentTimeUs + minimumFrameDelayUs;

            }
            bool handleEventNow = cmp32(currentTimeUs, nextEventAt) > 0;

            if (handleEventNow) {
                logEvent(currentTimeUs, pixelOsdVideoState);

                if (errorDetectionEnabled) {

                    // need to have had the `last*` variables initialised before it's possible to notice any change.

                    uint32_t recentPulseErrors = frameState.totalPulseErrors - lastTotalPulseErrors;
                    int32_t timeDeltaUs = cmp32(currentTimeUs, lastTimeUs);
                    pulseErrorsPerSecond = recentPulseErrors * 1000000 / timeDeltaUs;


                    int32_t recentFrames = frameState.validFrameCounter - lastValidFrameCounter;
                    framesPerSecond = recentFrames * 1000000 / timeDeltaUs;

                    //DEBUG_SET(DEBUG_SPRACING_PIXEL_OSD, 2, lastTotalPulseErrors);
                    DEBUG_SET(DEBUG_SPRACING_PIXEL_OSD, 2, pulseErrorsPerSecond);
                    DEBUG_SET(DEBUG_SPRACING_PIXEL_OSD, 3, framesPerSecond);

                    // TODO, if too many pulseErrorsPerSecond then reset sync levels.
                    // TODO, if frame counter stops counting then reset sync levels.
                    // TODO, if time since reset sync levels is large, and no valid frame received then camera is probably
                    // disconnected, enable internal sync generation instead.

                    bool tooManyPulseErrors = pulseErrorsPerSecond > 1000;

                    if (tooManyPulseErrors || (framesPerSecond == 0)) {

                        // probably the errors are caused by having camera sync interfering with generated sync or the camera was powered off

                        spracingPixelOSDPause();


                        if (cameraConnected) {
                            cameraConnected = false;
                        } else {
                            cameraConnected = true;
                        }

                        spracingPixelOSDRestart();

                        pixelOsdVideoState = SEARCHING_FOR_LINE_MIN_LEVEL;
                        nextEventAt = 0;
                        break;
                    }
                } else {
                    errorDetectionEnabled = true;
                }

                lastTimeUs = currentTimeUs;
                lastTotalPulseErrors = frameState.totalPulseErrors;
                lastValidFrameCounter = frameState.validFrameCounter;

                nextEventAt = currentTimeUs + 1000000; // one second
            }
            break;
        }
    }

    previousServiceAt = currentTimeUs;
}
