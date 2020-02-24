/*
 * Author: Dominic Clifton - Sync generation, Sync Detection, Video Overlay and first-cut of working OSD system.
 */

#pragma once

typedef enum frameStatus_e {
    WAITING_FOR_FIRST_FIELD = 0,
    COUNTING_PRE_EQUALIZING_PULSES,
    COUNTING_SYNCRONIZING_PULSES,
    COUNTING_POST_EQUALIZING_PULSES,
    COUNTING_HSYNC_PULSES,
} frameStatus_t;

typedef struct frameState_s {
    uint32_t frameStartCounter;
    uint32_t validFrameCounter;
    uint32_t lineCounter;
    uint16_t pulseErrors;
    uint16_t totalPulseErrors;
    frameStatus_t status;
    uint32_t vsyncAt;
} frameState_t;

void setComparatorTargetMv(uint32_t newTargetMv);
void disableComparatorBlanking(void);
void recalculateBlankingTimings(const videoTimings_t *vt);

void videoFrame_reset(void);

extern frameState_t frameState;
extern uint32_t osdFrameTimeoutAt;
