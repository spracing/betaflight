/*
 * Author: Dominic Clifton - Sync generation, Sync Detection, Video Overlay and first-cut of working OSD system.
 */

#include <stdbool.h>
#include <string.h>

#include "platform.h"

#include "drivers/io.h"
#include "drivers/time.h"

#include "configuration.h"
#include "io.h"
#include "videotiming.h"
#include "pixeltiming.h"
#include "pixelbuffer.h"

#include "spracing_pixel_osd.h"
#include "spracing_pixel_osd_impl.h"

#include "videoframe.h"

typedef enum fieldType_e {
    FIELD_EVEN = 0,
    FIELD_ODD = 1,

    FIELD_FIRST = 0,
    FIELD_SECOND = 1
} fieldType_t;

typedef enum fieldPhase_e {
    FIELD_UNKNOWN = 0,
    FIELD_PRE_EQUALIZING,
    FIELD_SYNCRONIZING,
    FIELD_POST_EQUALIZING,
    FIELD_HSYNC,
} fieldPhase_t;

typedef struct fieldState_s {
    uint8_t preEqualizingPulses;
    uint8_t syncronizingPulses;
    uint8_t postEqualizingPulses;
    fieldType_t type;
    fieldPhase_t phase;
    uint16_t lineNumber;

    uint16_t highestFieldLineNumber;
} fieldState_t;


volatile int16_t frameFallingEdgeIndex = 0;
volatile uint8_t fallingEdgesRemainingBeforeNewField = 0;

uint16_t firstVisibleLine;
uint16_t lastVisibleLine;
bool nextLineIsVisible;
volatile uint16_t visibleLineIndex;

volatile bool frameStartFlag = false;
volatile uint16_t fillLineIndex = 0;

uint32_t osdFrameTimeoutAt = 0; // XXX currently unused

#ifdef DEBUG_PULSE_STATISTICS
uint16_t syncPulseRisingStatisticIndex = 0;
uint16_t syncPulseRisingStatistics[PAL_LINES] __attribute__((used));
uint16_t syncPulseFallingStatisticIndex = 0;
uint16_t syncPulseFallingStatistics[PAL_LINES] __attribute__((used));
#endif

typedef struct {
#ifdef DEBUG_COMP_TRIGGER
    uint32_t triggerLowCount;
    uint32_t triggerHighCount;
#endif
    bool fallingEdge;
} compState_t;

compState_t compState = { 0 };

static volatile uint16_t pulseLength __attribute__((used)) = 0;
static volatile uint16_t previousPulseLength __attribute__((used)) = 0;

fieldState_t fieldState = { 0 };
frameState_t frameState = { 0 };


uint32_t targetMv = 0;

//
// Sync blanking
//

void disableComparatorBlanking(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_5, htim1.Init.Period + 1); // never reached.
    LL_TIM_OC_SetMode((&htim1)->Instance, LL_TIM_CHANNEL_CH5, LL_TIM_OCMODE_FORCED_ACTIVE);
    //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_5, htim1.Init.Period + 1);
#ifdef DEBUG_BLANKING
    LL_TIM_OC_SetMode((&htim1)->Instance, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_FORCED_ACTIVE);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, htim1.Init.Period + 1); // never reached.
    //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, htim1.Init.Period + 1);
#endif
}

static inline void setBlankingPeriod(uint32_t clocks) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_5, clocks);
#ifdef DEBUG_BLANKING
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, clocks);
#endif
}

//
// Sync level
//

void setComparatorTargetMv(uint32_t newTargetMv)
{
    // High-saturation colors in the picture data should not cause false comparator triggers.
    //
    //      color burst          /----\             /-----\.
    //                |    -----/      \----   ----/       \---
    //                v   |                | |                 |
    //  --        --\/\/\--                | |                 --
    //    | SYNC |                         |_|   <-- false comparator trigger needs to be avoided
    //    |______|
    //    ^      ^
    //    |      |
    //    |      Rising edge of Sync
    //    Falling edge of Sync
    //
    // Comparator threshold MV should be as low as possible to detect sync voltages without triggering
    // on low voltages causes by color bust or high-saturation colors.

    // IMPORTANT: The voltage keeps drifting the longer the camera has been on (rises over time)
    // TODO: auto-correct targetMv based on sync length (shorter = nearer lower level, longer = nearer high level)

    targetMv = newTargetMv;

    // TODO get measured VREF via ADC and use instead of VIDEO_DAC_VCC here?
    uint32_t dacComparatorRaw = (targetMv * 0x0FFF) / (VIDEO_DAC_VCC * 1000);

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dacComparatorRaw);
}

//
// Sync handling
//

void videoFrame_reset(void)
{
    memset(&frameState, 0x00, sizeof(frameState));
    memset(&fieldState, 0x00, sizeof(fieldState));
}

static inline void pulseError(void) {
#ifdef DEBUG_PULSE_ERRORS
    pixelDebug2Toggle();
#endif
    frameState.pulseErrors++;
    frameState.totalPulseErrors++;
#ifdef DEBUG_PULSE_ERRORS
    pixelDebug2Toggle();
#endif
}

#define USE_HAL_COMP_CALLBACK

#ifdef USE_HAL_COMP_CALLBACK
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{
#else
void RAW_COMP_TriggerCallback(void)
{
    COMP_HandleTypeDef *hcomp = &hcomp2;
    uint32_t exti_line = COMP_GET_EXTI_LINE(hcomp->Instance);
    __HAL_COMP_EXTI_CLEAR_FLAG(exti_line);

#endif
    static uint16_t previousCompare = 0;
    uint16_t currentCompare = (&htim2)->Instance->CCR4;
    pulseLength = currentCompare - previousCompare;

    compState.fallingEdge = (HAL_COMP_GetOutputLevel(hcomp) == COMP_OUTPUT_LEVEL_HIGH);
    if (compState.fallingEdge) {
#ifdef DEBUG_COMP_TRIGGER
        compState.triggerLowCount++;
        pixelDebug2Low();
#endif

        // VIDEO_SYNC_VSYNC_MIN ((uint32_t)((((64.000 / 2.0) - 4.700) - (((64.000 / 2.0) - 4.700) - (4.700))/2.0) * (80000000 / 1000000)))
        // VIDEO_SYNC_VSYNC_MAX ((uint32_t)((((64.000 / 2.0) - 4.700) + (((64.000 / 2.0) - 2.000) - ((64.000 / 2.0) - 4.700))/2.0) * (80000000 / 1000000)))
        if (pulseLength > VIDEO_SYNC_VSYNC_MIN && pulseLength < VIDEO_SYNC_VSYNC_MAX) {
            if (previousPulseLength > VIDEO_SYNC_HSYNC_MIN) {

                // depending on the video mode a half frame pulse occurs at different times
                // use the detected mode to figure out if this is the first or second field.

                switch (detectedVideoMode) {
                case MODE_PAL:
                    fieldState.type = FIELD_SECOND; // pulse occurs at end of second field
                    break;
                case MODE_NTSC:
                    fieldState.type = FIELD_FIRST;  // pulse occurs at end of first field
                    break;
                default:
                    frameState.status = WAITING_FOR_FIRST_FIELD;
                    break;
                }

#ifdef DEBUG_LAST_HALF_LINE
                pixelDebug2Low();
                pixelDebug2High();
#endif
            }
        }

        frameFallingEdgeIndex++;

#ifdef DEBUG_PULSE_STATISTICS
        syncPulseFallingStatistics[syncPulseFallingStatisticIndex] = pulseLength;
        syncPulseFallingStatisticIndex++;
        if (syncPulseFallingStatisticIndex >= sizeof(syncPulseFallingStatistics) / sizeof(syncPulseFallingStatistics[0])) {
            syncPulseFallingStatisticIndex = 0;
        }
#endif


    } else {
#ifdef DEBUG_COMP_TRIGGER
        compState.triggerHighCount++;
        pixelDebug2High();
#endif

        disableComparatorBlanking();

        //
        // check pulse lengths in order from shortest to longest.
        //
        if (pulseLength < VIDEO_SYNC_SHORT_MIN) {
            pulseError();
        } else if (pulseLength < VIDEO_SYNC_SHORT_MAX) {
#ifdef DEBUG_SHORT_PULSE
            pixelDebug2Low();
#endif

            if (frameState.status == WAITING_FOR_FIRST_FIELD) {
                frameState.status = COUNTING_PRE_EQUALIZING_PULSES;

                fieldState.phase = FIELD_PRE_EQUALIZING;
                fieldState.preEqualizingPulses = 1; // this one

            } else if (frameState.status == COUNTING_PRE_EQUALIZING_PULSES) {
                fieldState.preEqualizingPulses++;
            } else if (frameState.status == COUNTING_POST_EQUALIZING_PULSES) {
                fieldState.postEqualizingPulses++;
            } else if (frameState.status == COUNTING_SYNCRONIZING_PULSES) {
                frameState.status = COUNTING_POST_EQUALIZING_PULSES;

                fieldState.phase = FIELD_POST_EQUALIZING;
                fieldState.postEqualizingPulses = 1; // this one

                if (fieldState.syncronizingPulses == 5) { // PAL
                    detectedVideoMode = MODE_PAL;
                    firstVisibleLine = 15;
                    lastVisibleLine = (PAL_VISIBLE_LINES - 1);
                } else if (fieldState.syncronizingPulses == 6) { // NTSC
                    detectedVideoMode = MODE_NTSC;
                    firstVisibleLine = 15;
                    lastVisibleLine = (NTSC_VISIBLE_LINES - 1);
                }

                if (fieldState.type == FIELD_ODD) {
                    firstVisibleLine--;
                }
                lastVisibleLine += firstVisibleLine;

            } else if (frameState.status == COUNTING_HSYNC_PULSES) {
                frameState.status = COUNTING_PRE_EQUALIZING_PULSES;

                if (fieldState.type == FIELD_SECOND) {
                    if (frameState.pulseErrors == 0) {
                        frameState.validFrameCounter++;
                    }
                    frameState.pulseErrors = 0;
                }

                fieldState.phase = FIELD_PRE_EQUALIZING;
                fieldState.preEqualizingPulses = 1; // this one
            } else {
                pulseError();
            }

#ifdef DEBUG_SHORT_PULSE
            pixelDebug2High();
#endif

        } else if (pulseLength < VIDEO_SYNC_HSYNC_MAX) {

            //
            // Important start DMA *NOW* - then deal with remaining state.
            //
            bool thisLineIsVisible = nextLineIsVisible;
            if (thisLineIsVisible /* && pixelOsdState == GENERATING_VIDEO*/) {
                // DMA configured for next line (below) or by transfer-complete handler.
                pixelStartDMA();

                visibleLineIndex++;

                //
                // blank the comparator if the line is visible and later when the DMA completes, disable blanking.
                //

                // Now = rising pulse of HSYNC.

                LL_TIM_OC_SetMode((&htim1)->Instance, LL_TIM_CHANNEL_CH5, LL_TIM_OCMODE_FORCED_INACTIVE);
                LL_TIM_OC_SetMode((&htim1)->Instance, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_FORCED_INACTIVE);

                setBlankingPeriod(((&htim1)->Instance->CNT + _US_TO_CLOCKS(VIDEO_LINE_LEN - VIDEO_FRONT_PORCH - VIDEO_SYNC_HSYNC - VIDEO_COMPARATOR_TO_IRQ_OFFSET)) % (&htim1)->Instance->ARR);

                LL_TIM_OC_SetMode((&htim1)->Instance, LL_TIM_CHANNEL_CH5, TIM_OCMODE_ACTIVE);
                LL_TIM_OC_SetMode((&htim1)->Instance, LL_TIM_CHANNEL_CH3, TIM_OCMODE_ACTIVE);

            }

            switch(frameState.status) {
            case COUNTING_POST_EQUALIZING_PULSES:
                frameState.status = COUNTING_HSYNC_PULSES;

                //
                // HSync detected
                //
                fieldState.phase = FIELD_HSYNC;

            FALLTHROUGH;

            case COUNTING_HSYNC_PULSES:

                fieldState.lineNumber++;
                frameState.lineCounter++;

                // prepare for next line
                nextLineIsVisible = fieldState.lineNumber >= firstVisibleLine && fieldState.lineNumber <= lastVisibleLine;

                if (nextLineIsVisible) {
                    uint8_t *previousOutputPixelBuffer = outputPixelBuffer;

                    outputPixelBuffer = fillPixelBuffer;

                    fillLineIndex = visibleLineIndex;

                    fillPixelBuffer = previousOutputPixelBuffer;
                    if (!thisLineIsVisible) {
                        // if this line is visible the transfer-complete handler would configure the DMA
                        pixelConfigureDMAForNextField();
                    }

                    pixelBuffer_fillFromFrameBuffer(fillPixelBuffer, 0, fillLineIndex);
                }

                if (fieldState.lineNumber > fieldState.highestFieldLineNumber) {
                    fieldState.highestFieldLineNumber = fieldState.lineNumber;
                }
            break;
            default:
                pulseError();
            }

        } else if (pulseLength >= VIDEO_SYNC_LO_BROAD_MIN && pulseLength <= VIDEO_SYNC_LO_BROAD_MAX) {

            if (frameState.status == COUNTING_PRE_EQUALIZING_PULSES) {
                frameState.status = COUNTING_SYNCRONIZING_PULSES;

                fieldState.syncronizingPulses = 0; // incremented below for this one

                fieldState.phase = FIELD_SYNCRONIZING;

                if (fieldState.type == FIELD_SECOND) {
                    fieldState.type = FIELD_FIRST;

                    frameState.vsyncAt = microsISR();
                    frameState.frameStartCounter++;
                    frameStartFlag = true;

                    osdFrameTimeoutAt = frameState.vsyncAt + (1000000 / 50); // 50 FPS

                } else {
                    fieldState.type = FIELD_SECOND;
                }

                fieldState.lineNumber = 0;
                visibleLineIndex = 0;
                nextLineIsVisible = false;

#ifdef DEBUG_FIELD_START
                pixelDebug2Low();
#endif
            }

            if (frameState.status == COUNTING_POST_EQUALIZING_PULSES) {
                fieldState.type = FIELD_SECOND;
#ifdef DEBUG_FIELD_START
                pixelDebug2High();
#endif
            }

            if (frameState.status == COUNTING_SYNCRONIZING_PULSES) {
                fieldState.syncronizingPulses++;
            }

#ifdef DEBUG_FIRST_SYNC_PULSE
            bool firstSyncPulse = (fieldState.syncronizingPulses == 0);
            if (firstSyncPulse) {
                pixelDebug2Low();
            } else if (fieldState.syncronizingPulses == 1) {
                pixelDebug2High();
            }
#endif
        } else {
            pulseError();
        }


#ifdef DEBUG_PULSE_STATISTICS
        syncPulseRisingStatistics[syncPulseRisingStatisticIndex] = pulseLength;
        syncPulseRisingStatisticIndex++;
        if (syncPulseRisingStatisticIndex >= sizeof(syncPulseRisingStatistics) / sizeof(syncPulseRisingStatistics[0])) {
            syncPulseRisingStatisticIndex = 0;
        }
#endif
    }

    previousPulseLength = pulseLength;
    previousCompare = currentCompare;
}

void COMP1_IRQHandler(void)
{
#ifdef USE_HAL_COMP_CALLBACK
  HAL_COMP_IRQHandler(&hcomp2);
#else
  RAW_COMP_TriggerCallback();
#endif
}
