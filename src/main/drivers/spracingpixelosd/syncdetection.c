#include <stdbool.h>
#include <string.h>

#include "platform.h"

#include "build/debug.h"

#include "common/time.h"

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
    SEARCHING_FOR_FRAME_MAX_LEVEL,
    GENERATING_VIDEO,
} pixelOsdState_t;

pixelOsdState_t pixelOsdState = SEARCHING_FOR_LINE_MIN_LEVEL;

#ifdef DEBUG_OSD_EVENTS
typedef struct eventLogItem_s {
    timeUs_t us;
    pixelOsdState_t state;
} eventLogItem_t;

eventLogItem_t eventLog[256] = {0};
unsigned int eventLogIndex = 0;

void logEvent(timeUs_t us, pixelOsdState_t state)
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


uint32_t pulseErrorsPerSecond = 0;
uint32_t framesPerSecond = 0;

syncDetectionState_t syncDetectionState = { 0 };
static uint32_t nextEventAt = 0;

void syncDetection_reset(void)
{
    memset(&syncDetectionState, 0x00, sizeof(syncDetectionState));
}

bool spracingPixelOSDShouldProcessNow(timeUs_t currentTimeUs)
{
    bool handleEventNow = cmp32(currentTimeUs, nextEventAt) > 0;
    return handleEventNow;
}

void spracingPixelOSDProcess(timeUs_t currentTimeUs)
{
    const uint16_t requiredLines = 100; // ~64us * 100 = 6.4ms

    const uint32_t lineCounterDelayUs = (VIDEO_LINE_LEN) * (requiredLines);
    const uint32_t minimumFrameDelayUs = (VIDEO_LINE_LEN) * (PAL_LINES + 10);

    DEBUG_SET(DEBUG_SPRACING_PIXEL_OSD, 0, frameState.validFrameCounter);
    DEBUG_SET(DEBUG_SPRACING_PIXEL_OSD, 0, frameState.totalPulseErrors);

    static uint8_t syncDetectionFailureCount = 0;

    switch(pixelOsdState) {
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

                    pixelOsdState = SEARCHING_FOR_LINE_MIN_LEVEL;
                    nextEventAt = 0;
                }
            }

            bool handleEventNow = cmp32(currentTimeUs, nextEventAt) > 0;
            if (handleEventNow) {
                logEvent(currentTimeUs, pixelOsdState);

                pixelOsdState = SEARCHING_FOR_LINE_MIN_LEVEL;
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
                logEvent(currentTimeUs, pixelOsdState);

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
                        pixelOsdState = OUTPUT_DISABLED;
                        nextEventAt = 0;
                    }
                } else {
                    pixelOsdState = SEARCHING_FOR_FRAME_MIN_LEVEL;
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
                logEvent(currentTimeUs, pixelOsdState);

                if (frameState.validFrameCounter == 0) {

                    uint32_t minMaxDifference = syncDetectionState.minimumLevelForValidFrameMv - syncDetectionState.minimumLevelForLineThreshold;

                    if (syncDetectionState.minimumLevelForValidFrameMv < MAXIMIM_FRAME_LEVEL_THRESHOLD_MV && minMaxDifference < MAXIMIM_LINE_LEVEL_DIFFERENCE_MV) {
                        syncDetectionState.minimumLevelForValidFrameMv += 5;
                        setComparatorTargetMv(syncDetectionState.minimumLevelForValidFrameMv);
                        nextEventAt = currentTimeUs + minimumFrameDelayUs;
                    } else {
                        pixelOsdState = OUTPUT_DISABLED;
                        nextEventAt = 0;
                    }

                } else {
                    pixelOsdState = SEARCHING_FOR_FRAME_MAX_LEVEL;
                    nextEventAt = 0;
                }
            }
            break;
        }
        case SEARCHING_FOR_FRAME_MAX_LEVEL:
        {
            static uint32_t validFrameCounterAtStart;

            if (nextEventAt == 0) {
                // state transition
                validFrameCounterAtStart = frameState.validFrameCounter;
                syncDetectionState.maximumLevelForValidFrameMv = syncDetectionState.minimumLevelForValidFrameMv + 5;
                setComparatorTargetMv(syncDetectionState.maximumLevelForValidFrameMv);
                nextEventAt = currentTimeUs + minimumFrameDelayUs;
            }
            bool handleEventNow = cmp32(currentTimeUs, nextEventAt) > 0;

            if (handleEventNow) {
                logEvent(currentTimeUs, pixelOsdState);

                int32_t framesSinceStart = frameState.validFrameCounter - validFrameCounterAtStart;

                bool levelOk = false;

                if (framesSinceStart > 0) {
                    // still getting valid frames, increase targetMv
                    uint32_t minMaxDifference = syncDetectionState.maximumLevelForValidFrameMv - syncDetectionState.minimumLevelForValidFrameMv;

                    if (syncDetectionState.minimumLevelForValidFrameMv < MAXIMIM_FRAME_LEVEL_THRESHOLD_MV && minMaxDifference < MAXIMIM_FRAME_LEVEL_DIFFERENCE_MV) {
                        syncDetectionState.maximumLevelForValidFrameMv += 5;
                        setComparatorTargetMv(syncDetectionState.maximumLevelForValidFrameMv);

                        // start again using current frame counter.
                        validFrameCounterAtStart = frameState.validFrameCounter;
                        nextEventAt = currentTimeUs + minimumFrameDelayUs;
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

                    pixelOsdState = GENERATING_VIDEO;

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
                logEvent(currentTimeUs, pixelOsdState);

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

                        pixelOsdState = SEARCHING_FOR_LINE_MIN_LEVEL;
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

#if 0
    bool osdFrameTimeoutFlag = osdFrameTimeoutAt > 0 && cmp32(currentTimeUs, osdFrameTimeoutAt) > 0;

    // There should be no pulse errors when the picture is generated by the FC.
    // The sync input will be messed up if a camera is connected at the same time as sync is generated.
    bool internalSyncInterference = (!cameraConnected && frameState.pulseErrors > PAL_LINES * 2); // state should have settled after the initial frame.

    //
    // If the comparator threshold is wrong then pulse errors will occur.
    //
    bool externalSyncInterference = (cameraConnected && frameState.pulseErrors > 100);
    UNUSED(externalSyncInterference);

    if (osdFrameTimeoutFlag || internalSyncInterference) {
        if (cameraConnected) {
            cameraConnected = false;
        } else {
            syncStopDMA();
            syncStopPWM();

            // probably the frame timeout caused by having camera sync interfering with generated sync..
            cameraConnected = true;
        }

        pixelStopDMA();
        disableComparatorBlanking();


        spracingPixelOSDSyncTimerInit();
        //spracingPixelOSDSyncTriggerReset();
        setComparatorTargetMv(0);
        syncInit();

        memset(&frameState, 0x00, sizeof(frameState));
        memset(&fieldState, 0x00, sizeof(fieldState));

        osdFrameTimeoutAt = currentTimeUs + (1000000 / 1) * 1; // 1 second
    }

#endif
}
