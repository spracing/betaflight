#include <stdbool.h>

#include "platform.h"

#include "configuration.h"
#include "glue.h"
#include "videotiming.h"
#include "spracing_pixel_osd.h"
#include "spracing_pixel_osd_impl.h"

#include "syncgeneration.h"


//
// Sync Generation
//

bool syncDMAActive = false;

DMA_HandleTypeDef *hSyncOutDMA;

typedef struct syncBufferItem_s {
    // the order and size of these structure is fixed.  the data is transferred by DMA to the timer peripheral, starting with the ARR register
    uint16_t arrValue;
    uint16_t repetitions; // timers only support 8 bit ARR.
    uint16_t cc1Value;
#ifdef USE_TIM1_CH3_FOR_SYNC
    uint16_t cc2Value;
    uint16_t cc3Value;
#endif
} syncBufferItem_t;

#ifdef USE_TIM1_CH3_FOR_SYNC
#define SYNC_CC(cc3Value) 0, 0, cc3Value
#define SYNC_BUST_TRANSFER_COUNT TIM_DMABURSTLENGTH_5TRANSFERS
#else
#define SYNC_CC(cc1Value) cc1Value
#define SYNC_BUST_TRANSFER_COUNT TIM_DMABURSTLENGTH_3TRANSFERS
#endif

#define HALF_LINE(period, repetitions, pulse) _US_TO_CLOCKS(period / 2) - 1, (repetitions) - 1, SYNC_CC(_US_TO_CLOCKS(pulse) - 1)
#define FULL_LINE(period, repetitions, pulse) _US_TO_CLOCKS(period) - 1, (repetitions) - 1, SYNC_CC(_US_TO_CLOCKS(pulse) - 1)

const syncBufferItem_t palSyncItems[] = {
        { HALF_LINE(VIDEO_LINE_LEN,  5,   VIDEO_SYNC_LO_BROAD)},  // start of first field
        { HALF_LINE(VIDEO_LINE_LEN,  5,   VIDEO_SYNC_SHORT)},
        { FULL_LINE(VIDEO_LINE_LEN,  153, VIDEO_SYNC_HSYNC)},     // start of picture data
        { FULL_LINE(VIDEO_LINE_LEN,  152, VIDEO_SYNC_HSYNC)},
        { HALF_LINE(VIDEO_LINE_LEN,  5,   VIDEO_SYNC_SHORT)},
        { HALF_LINE(VIDEO_LINE_LEN,  5,   VIDEO_SYNC_LO_BROAD)},  // start of second field
        { HALF_LINE(VIDEO_LINE_LEN,  4,   VIDEO_SYNC_SHORT)},
        { FULL_LINE(VIDEO_LINE_LEN,  1,   VIDEO_SYNC_SHORT)},     // second half of a line
        { FULL_LINE(VIDEO_LINE_LEN,  152, VIDEO_SYNC_HSYNC)},     // start of picture data
        { FULL_LINE(VIDEO_LINE_LEN,  152, VIDEO_SYNC_HSYNC)},
        { HALF_LINE(VIDEO_LINE_LEN,  1,   VIDEO_SYNC_HSYNC)},     // first half of a line/frame
        { HALF_LINE(VIDEO_LINE_LEN,  5,   VIDEO_SYNC_SHORT)},
        // 625 lines (2.5+2.5+153+152+2.5+2.5+2+1+152+152+.5+2.5)
};

#undef SYNC_CC
#undef HALF_LINE
#undef FULL_LINE

void syncStartPWM(void)
{
    if (HAL_TIM_PWM_Start(&htim1, SYNC_TIMER_CHANNEL) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_TIMEx_PWMN_Start(&htim1, SYNC_TIMER_CHANNEL) != HAL_OK)
    {
      Error_Handler();
    }
}

void syncStopPWM(void)
{
    if (HAL_TIM_PWM_Stop(&htim1, SYNC_TIMER_CHANNEL) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_TIMEx_PWMN_Stop(&htim1, SYNC_TIMER_CHANNEL) != HAL_OK)
    {
      Error_Handler();
    }

    //HAL_GPIO_WritePin(SYNC_OUT_GPIO_Port, SYNC_OUT_Pin, GPIO_PIN_RESET); // XXX - In GPIO AF TIM mode this has no effect, PWM IDLE state is what matters.
}

void syncStartDMA(void)
{
    HAL_TIM_DMABurst_MultiWriteStart(
        &htim1,
        TIM_DMABASE_ARR,
        TIM_DMA_UPDATE,
        (uint32_t *)palSyncItems,
        SYNC_BUST_TRANSFER_COUNT,
        sizeof(palSyncItems) / 2 // 2 because each item is uint16_t, not uint32_t?
    );

    syncDMAActive = true;
}

void syncStopDMA(void)
{
    HAL_TIM_DMABurst_WriteStop(&htim1, TIM_DMA_UPDATE);
}

void syncInit(void)
{
    hSyncOutDMA = htim1.hdma[TIM_DMA_ID_UPDATE];

    if (!cameraConnected) {
        syncStartPWM();
        syncStartDMA();
    }
}
