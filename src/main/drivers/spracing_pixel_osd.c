/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Author: Dominic Clifton
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SPRACING_PIXEL_OSD

#include "build/debug.h"

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/nvic.h"
#include "drivers/io.h"
#include "drivers/timer.h"
//#include "drivers/light_led.h"
//#include "drivers/time.h"

#include "pg/spracing_pixel_osd.h"
#include "pg/vcd.h"

#include "drivers/spracing_pixel_osd.h"

// Pins 8-15 of GPIOE reserved for OSD use when in GPIO OUTPUT MODE (Upper 8 bits of GPIO port)
// Pins 8-15 on GPIOE *can* be used for other functions, just not GPIO OUTPUT.
#define SPRACING_PIXEL_OSD_BLACK_PIN                    PE13
#define SPRACING_PIXEL_OSD_WHITE_PIN                    PE14
#define SPRACING_PIXEL_OSD_RESERVED_PIN                 PE15

#define SPRACING_PIXEL_OSD_SYNC_IN_PIN                  PE11 // COMP2_INP
#define SPRACING_PIXEL_OSD_SYNC_OUT_A_PIN               PE12 // TIM1_CH3N
#define SPRACING_PIXEL_OSD_SYNC_OUT_B_PIN               PA8  // TIM1_CH1 / MCO

#define SPRACING_PIXEL_OSD_VIDEO_THRESHOLD_DEBUG_PIN    PA5
#define SPRACING_PIXEL_OSD_PIXEL_DEBUG_1_PIN            PE5  // TIM15_CH1 - For DMA updates
#define SPRACING_PIXEL_OSD_PIXEL_DEBUG_2_PIN            PE6  // TIM15_CH2 - Spare


//
// Video Format
//

#define PAL_LINES 625
#define NTSC_LINES 525

//
// Output Specification
//

#define PAL_VISIBLE_LINES 288  // MAX7456 (16 rows * 18 character height)
#define NTSC_VISIBLE_LINES 234 // MAX7456 (13 rows * 18 character height)

// 48us * 80 = 3840 clocks.  3840 clocks / 640 = 6 clocks per pixel.
// resolution scale of 2 = 12 clocks per pixel = 320 pixels.

#define HORIZONTAL_RESOLUTION 640
#define RESOLUTION_SCALE 2
#define OVERLAY_LENGTH 48.000 // us


//
// Timing
//

#define TIMER_BUS_CLOCK   200000000
#define TIMER_CLOCK  100000000

#define TIMER_CLOCKS_PER_US                      (TIMER_CLOCK / 1000000)
#define _US_TO_CLOCKS(__us)                      ((uint32_t)((__us) * TIMER_CLOCKS_PER_US))


#ifdef USE_NTSC
#define VIDEO_LINE_LEN            63.556  // us
#define VIDEO_SYNC_SHORT           2.000  // us
#define VIDEO_SYNC_HSYNC           4.700  // us
#else
#define VIDEO_LINE_LEN            64.000  // us
#define VIDEO_SYNC_SHORT           2.000  // us
#define VIDEO_SYNC_HSYNC           4.700  // us
#endif
#define VIDEO_FIELD_ODD            1
#define VIDEO_FIELD_EVEN           (1-VIDEO_FIELD_ODD)
#define VIDEO_FIRST_FIELD          VIDEO_FIELD_ODD     // ODD (NTSC)
#define VIDEO_SECOND_FIELD         (1-(VIDEO_FIRST_FIELD))

// timing for high level (from shortest to longest period)
// (1) [HI] BROAD SYNC: t ~ VIDEO_SYNC_HSYNC
// (2) [HI] VSYNC+DATA: t ~ (VIDEO_LINE_LEN / 2) - VIDEO_SYNC_HSYNC
// (3) [HI] SHORT SYNC: t ~ (VIDEO_LINE_LEN / 2) - VIDEO_SYNC_SHORT
// (4) [HI] VIDEO DATA: t ~ VIDEO_LINE_LEN - VIDEO_SYNC_HSYNC
//
#define VIDEO_SYNC_HI_BROAD     (VIDEO_SYNC_HSYNC)
#define VIDEO_SYNC_HI_VSYNC     ((VIDEO_LINE_LEN / 2.0) - VIDEO_SYNC_HSYNC)
#define VIDEO_SYNC_HI_SHORT     ((VIDEO_LINE_LEN / 2.0) - VIDEO_SYNC_SHORT)
#define VIDEO_SYNC_HI_DATA      (VIDEO_LINE_LEN)
//
// -> valid vsync = .... (1)---[xxx(2)xxx]---(3)------(4)
//
#define VIDEO_SYNC_VSYNC_MIN        _US_TO_CLOCKS(VIDEO_SYNC_HI_VSYNC - (VIDEO_SYNC_HI_VSYNC - VIDEO_SYNC_HI_BROAD)/2.0)
#define VIDEO_SYNC_VSYNC_MAX        _US_TO_CLOCKS(VIDEO_SYNC_HI_VSYNC + (VIDEO_SYNC_HI_SHORT - VIDEO_SYNC_HI_VSYNC)/2.0)

// timing for low level (from shortest to longest period)
// (1) [LO] SHORT SYNC: t ~ 2.0us
// (2) [LO] HSYNC     : t ~ 4.7us
// (3) [LO] BROAD     : t ~ (VIDEO_LINE_LEN / 2) - VIDEO_SYNC_HSYNC
//
//
// short sync =  (1)xxx]---(2)------(3)
//
#define VIDEO_SYNC_SHORT_MIN    _US_TO_CLOCKS(0)
#define VIDEO_SYNC_SHORT_MAX    _US_TO_CLOCKS(VIDEO_SYNC_SHORT +  (VIDEO_SYNC_HSYNC - VIDEO_SYNC_SHORT)/2.0)
//
// hsync      =  (1)---[xxx(2)xxx]---(3)
//
#define VIDEO_SYNC_HSYNC_MIN    _US_TO_CLOCKS(VIDEO_SYNC_HSYNC - (VIDEO_SYNC_HSYNC - VIDEO_SYNC_SHORT)/2.0)
#define VIDEO_SYNC_HSYNC_MAX    _US_TO_CLOCKS(VIDEO_SYNC_HSYNC + (VIDEO_SYNC_LO_BROAD - VIDEO_SYNC_HSYNC)/2.0)
//
// broad      = (1)------(2)---[xxx(3)]
//
#define VIDEO_SYNC_LO_BROAD       (VIDEO_LINE_LEN / 2.0) - VIDEO_SYNC_HSYNC
#define VIDEO_SYNC_LO_BROAD_MIN   _US_TO_CLOCKS(VIDEO_SYNC_LO_BROAD - VIDEO_SYNC_HSYNC/2.0)
#define VIDEO_SYNC_LO_BROAD_MAX   _US_TO_CLOCKS(VIDEO_SYNC_LO_BROAD + VIDEO_SYNC_HSYNC/2.0)

//
// Pixel Generation
//

#define BLACK_SINK_Pin GPIO_PIN_13
#define BLACK_SINK_GPIO_Port GPIOE
#define WHITE_SOURCE_Pin GPIO_PIN_14
#define WHITE_SOURCE_GPIO_Port GPIOE

#define PIXEL_DEBUG_1_Pin GPIO_PIN_5
#define PIXEL_DEBUG_1_GPIO_Port GPIOE
#define PIXEL_DEBUG_2_Pin GPIO_PIN_6
#define PIXEL_DEBUG_2_GPIO_Port GPIOE

#define PIXEL_COUNT (HORIZONTAL_RESOLUTION / RESOLUTION_SCALE)
#define PIXEL_BUFFER_SIZE PIXEL_COUNT + 1 // one more pixel which must always be transparent to reset output level during sync

#define USE_PIXEL_OUT_GPIOE

// NOTE: for optimal CPU usage the current design requires that the pixel black and white GPIO bits are adjacent.
// BLACK bit must be before WHITE bit.
#ifdef USE_PIXEL_OUT_GPIOE
#define PIXEL_ODR       GPIOE->ODR
#define PIXEL_ODR_OFFSET 8 // 0 = PE0-PE7, 8 = PE8-PE15

#define PIXEL_BLACK_BIT 5 // PE13
#define PIXEL_WHITE_BIT 6 // PE14
#endif

#ifdef USE_PIXEL_OUT_GPIOC
#define PIXEL_ODR       GPIOC->ODR
#define PIXEL_ODR_OFFSET 8

#define PIXEL_BLACK_BIT 6 // PC14
#define PIXEL_WHITE_BIT 7 // PC15
#endif

#ifdef USE_PIXEL_OUT_GPIOB
#define PIXEL_ODR       GPIOB->ODR
#define PIXEL_ODR_OFFSET 0
//#define PIXEL_BLACK_BIT 6 // PB6
//#define PIXEL_WHITE_BIT 7 // PB7
#define PIXEL_BLACK_BIT 0 // PB0
#define PIXEL_WHITE_BIT 1 // PB1
#endif

#define BITS_PER_PIXEL 2 // the current implementation only supports 2.

#define PIXEL_ADDRESS   ((uint32_t)&(PIXEL_ODR) + (PIXEL_ODR_OFFSET / 8)) // +1 for upper 8 bits

uint8_t pixelBufferA[PIXEL_BUFFER_SIZE];
uint8_t pixelBufferB[PIXEL_BUFFER_SIZE];

uint8_t *fillPixelBuffer = NULL;
uint8_t *outputPixelBuffer = NULL;

#define PIXEL_WHITE_ON 1
#define PIXEL_WHITE_OFF 0
// black is inverted (open drain)
#define PIXEL_BLACK_ON 0
#define PIXEL_BLACK_OFF 1

#define PIXEL_WHITE       ((PIXEL_WHITE_ON  << PIXEL_WHITE_BIT) | (PIXEL_BLACK_OFF << PIXEL_BLACK_BIT))
#define PIXEL_BLACK       ((PIXEL_WHITE_OFF << PIXEL_WHITE_BIT) | (PIXEL_BLACK_ON  << PIXEL_BLACK_BIT))
#define PIXEL_GREY        ((PIXEL_WHITE_ON  << PIXEL_WHITE_BIT) | (PIXEL_BLACK_ON  << PIXEL_BLACK_BIT))
#define PIXEL_TRANSPARENT ((PIXEL_WHITE_OFF << PIXEL_WHITE_BIT) | (PIXEL_BLACK_OFF << PIXEL_BLACK_BIT))

//
// Frame
//

#define FRAME_PIXEL_WHITE       ((PIXEL_WHITE_ON  << 1) | (PIXEL_BLACK_OFF << 0))
#define FRAME_PIXEL_BLACK       ((PIXEL_WHITE_OFF << 1) | (PIXEL_BLACK_ON  << 0))
#define FRAME_PIXEL_GREY        ((PIXEL_WHITE_ON  << 1) | (PIXEL_BLACK_ON  << 0))
#define FRAME_PIXEL_TRANSPARENT ((PIXEL_WHITE_OFF << 1) | (PIXEL_BLACK_OFF << 0))

#define FRAME_PIXEL_MASK        ((1 << 1) | (1 << 0))

#define BLOCK_TRANSPARENT ((FRAME_PIXEL_TRANSPARENT << 6) | (FRAME_PIXEL_TRANSPARENT << 4) | (FRAME_PIXEL_TRANSPARENT << 2) | (FRAME_PIXEL_TRANSPARENT << 0))

#define PIXELS_PER_BYTE (8 / BITS_PER_PIXEL)

#define BITS_PER_BYTE 8

#define FRAME_BUFFER_LINE_SIZE ((PIXEL_COUNT / BITS_PER_BYTE) * BITS_PER_PIXEL)
#define FRAME_BUFFER_SIZE (FRAME_BUFFER_LINE_SIZE * PAL_VISIBLE_LINES)

uint8_t frameBuffers[2][FRAME_BUFFER_SIZE];

//
// Sync Detection/Timing
//

COMP_HandleTypeDef hcomp2;
DAC_HandleTypeDef hdac1;

#define VIDEO_DAC_VCC 3.3

volatile videoMode_t detectedVideoMode = MODE_UNKNOWN;

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

typedef enum frameStatus_e {
    WAITING_FOR_FIRST_FIELD = 0,
    COUNTING_PRE_EQUALIZING_PULSES,
    COUNTING_SYNCRONIZING_PULSES,
    COUNTING_POST_EQUALIZING_PULSES,
    COUNTING_HSYNC_PULSES,
} frameStatus_t;

typedef struct fieldState_s {
    uint8_t preEqualizingPulses;
    uint8_t syncronizingPulses;
    uint8_t postEqualizingPulses;
    fieldType_t type;
    fieldPhase_t phase;
    uint16_t lineNumber;

    uint16_t highestFieldLineNumber;
} fieldState_t;

typedef struct frameState_s {
    uint32_t counter;
    uint16_t lineNumber;
    uint16_t pulseErrors;
    frameStatus_t status;
} frameState_t;


volatile int16_t frameFallingEdgeIndex = 0;
volatile uint8_t fallingEdgesRemainingBeforeNewField = 0;

uint16_t firstVisibleLine;
uint16_t lastVisibleLine;
bool nextLineIsVisible;
volatile uint16_t visibleLineIndex;

volatile bool fillLineNow = false;
volatile bool frameFlag = false;
volatile uint16_t fillLineIndex = 0;

//
// State
//

volatile bool cameraConnected = true;

typedef struct spracingPixelOSDIO_s {
    IO_t blackPin;
    IO_t whitePin;
    IO_t syncInPin;
} spracingPixelOSDIO_t;

static spracingPixelOSDIO_t spracingPixelOSDIO = {
    .blackPin           = IO_NONE,
    .whitePin           = IO_NONE,
    .syncInPin          = IO_NONE,
};

#define IO_PIXEL_BLACK_CFG      IO_CONFIG(GPIO_MODE_OUTPUT_OD, GPIO_SPEED_FREQ_MEDIUM,  GPIO_NOPULL)
#define IO_PIXEL_WHITE_CFG      IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_MEDIUM,  GPIO_NOPULL)

#define IO_VIDEO_SYNC_IN_CFG       IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_LOW,     GPIO_NOPULL)

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim15;
DMA_HandleTypeDef hdma_tim1_up;
DMA_HandleTypeDef hdma_tim15_ch1;

typedef DMA_Stream_TypeDef dmaStream_t;

//
// Sync Generation
//

bool syncDMAActive = false;
DMA_HandleTypeDef *hSyncOutDMA;

void syncInit(void);
void syncStartDMA(void);
void syncStopDMA(void);
void syncStartPWM(void);
void syncStopPWM(void);

//
//
//

static void Error_Handler()
{
    do {} while (true);
}


void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */

      __HAL_RCC_GPIOE_CLK_ENABLE();
      /**TIM1 GPIO Configuration
      PE12     ------> TIM1_CH3N
      */
      GPIO_InitStruct.Pin = GPIO_PIN_12;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
      HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }
  else if(htim->Instance==TIM15)
  {
  /* USER CODE BEGIN TIM15_MspPostInit 0 */

  /* USER CODE END TIM15_MspPostInit 0 */

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**TIM15 GPIO Configuration
    PE5     ------> TIM15_CH1
    PE6     ------> TIM15_CH2
    */
    GPIO_InitStruct.Pin = PIXEL_DEBUG_1_Pin|PIXEL_DEBUG_2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM15;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM15_MspPostInit 1 */

  /* USER CODE END TIM15_MspPostInit 1 */
  }

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* TIM1 DMA Init */
    /* TIM1_UP Init */
    hdma_tim1_up.Instance = DMA2_Stream6;
    hdma_tim1_up.Init.Request = DMA_REQUEST_TIM1_UP;
    hdma_tim1_up.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim1_up.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim1_up.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim1_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim1_up.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim1_up.Init.Mode = DMA_CIRCULAR;
    hdma_tim1_up.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tim1_up.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_tim1_up) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_UPDATE],hdma_tim1_up);

    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
  else if(htim_base->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }

  else if(htim_base->Instance==TIM15)
  {
  /* USER CODE BEGIN TIM15_MspInit 0 */

  /* USER CODE END TIM15_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM15_CLK_ENABLE();

    /* TIM15 DMA Init */
    /* TIM15_CH1_UP_TRIG_COM Init */
    hdma_tim15_ch1.Instance = DMA2_Stream7;
    hdma_tim15_ch1.Init.Request = DMA_REQUEST_TIM15_CH1;
    hdma_tim15_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim15_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim15_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim15_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tim15_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_tim15_ch1.Init.Mode = DMA_NORMAL;
    hdma_tim15_ch1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_tim15_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_tim15_ch1) != HAL_OK)
    {
      Error_Handler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one channel to perform all the requested DMAs. */
    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_CC1],hdma_tim15_ch1);
//    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_UPDATE],hdma_tim15_ch1);
//    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_TRIGGER],hdma_tim15_ch1);
//    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_COMMUTATION],hdma_tim15_ch1);

  /* USER CODE BEGIN TIM15_MspInit 1 */

  /* USER CODE END TIM15_MspInit 1 */
  }
}

void SYNC_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    //    HAL_DMA_IRQHandler(TimHandle.hdma[descriptor->userParam]);
    HAL_DMA_IRQHandler(&hdma_tim1_up);
}

void PIXEL_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    //    HAL_DMA_IRQHandler(TimHandle.hdma[descriptor->userParam]);
    HAL_DMA_IRQHandler(&hdma_tim15_ch1);
}

static void MX_DMA_Init(void)
{
#if 0
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
#endif


  //
  // Sync Generation DMA
  //

  ioTag_t syncIoTag = timerioTagGetByUsage(TIM_USE_VIDEO_SYNC, 0);
  const timerHardware_t *syncTimerHardware = timerGetByTag(syncIoTag);

#if defined(USE_DMA_SPEC)
    const dmaChannelSpec_t *syncDmaSpec = dmaGetChannelSpecByTimer(syncTimerHardware);

    if (!syncDmaSpec) {
        return;
    }

    dmaStream_t *syncDmaRef = syncDmaSpec->ref;
    uint32_t syncDmaChannel = syncDmaSpec->channel;
#else
    dmaStream_t *syncDmaRef = syncTimerHardware->dmaRef;
    uint32_t syncDmaChannel = syncTimerHardware->dmaChannel;
#endif

  uint16_t syncTimerChannel = syncTimerHardware->channel;
  uint16_t syncDmaIndex = timerDmaIndex(syncTimerChannel);

  dmaInit(dmaGetIdentifier(syncDmaRef), OWNER_OSD, 0);
  dmaSetHandler(dmaGetIdentifier(syncDmaRef), SYNC_DMA_IRQHandler, NVIC_PRIO_VIDEO_DMA, syncDmaIndex);

  //
  // Pixel Generation DMA
  //

  ioTag_t pixelIoTag = timerioTagGetByUsage(TIM_USE_VIDEO_PIXEL, 0);
  const timerHardware_t *pixelTimerHardware = timerGetByTag(pixelIoTag);

#if defined(USE_DMA_SPEC)
    const dmaChannelSpec_t *pixelDmaSpec = dmaGetChannelSpecByTimer(pixelTimerHardware);

    if (!pixelDmaSpec) {
        return;
    }

    dmaStream_t *pixelDmaRef = pixelDmaSpec->ref;
    uint32_t pixelDmaChannel = pixelDmaSpec->channel;
#else
    dmaStream_t *pixelDmaRef = pixelTimerHardware->dmaRef;
    uint32_t pixelDmaChannel = pixelTimerHardware->dmaChannel;
#endif

  uint16_t pixelTimerChannel = pixelTimerHardware->channel;
  uint16_t pixelDmaIndex = timerDmaIndex(pixelTimerChannel);

  dmaInit(dmaGetIdentifier(pixelDmaRef), OWNER_OSD, 0);
  dmaSetHandler(dmaGetIdentifier(pixelDmaRef), PIXEL_DMA_IRQHandler, NVIC_PRIO_VIDEO_DMA, pixelDmaIndex);

}

static void spracingPixelOSDSyncTimerInit(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = (TIMER_BUS_CLOCK / TIMER_CLOCK) - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = _US_TO_CLOCKS(VIDEO_LINE_LEN) - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
/*
  // IMPORTANT: Start the timer BEFORE enabling slave mode.
  if (HAL_TIM_Base_Start(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  */

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  if (cameraConnected) {
      // COMP2 trigger resets timer (i.e. when camera connected)
      sSlaveConfig.SlaveMode = TIM_SLAVEMODE_COMBINED_RESETTRIGGER;
      sSlaveConfig.InputTrigger = TIM_TS_ETRF;
      sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_INVERTED;
      sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
      sSlaveConfig.TriggerFilter = 0;
      if (HAL_TIM_SlaveConfigSynchronization(&htim1, &sSlaveConfig) != HAL_OK)
      {
        Error_Handler();
      }
  }

  // Channel 4 used to gate TIM15, TRGO2 unused, but can use it to trigger ADC of black level.
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIMEx_RemapConfig(&htim1, TIM_TIM1_ETR_COMP2_OUT) != HAL_OK)
  {
    Error_Handler();
  }

  // Channel 3 for SYNC output
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = _US_TO_CLOCKS(4.7);
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  // offset can be between 0 and 4, 4 = (VIDEO_LINE_LEN(~64) - hsync(4.7) - back porch(5.8) - front porch (1.5)) - OVERLAY_LENGTH (48)
  sConfigOC.Pulse = _US_TO_CLOCKS((4.7 + 5.8) + (1.8)); // start of picture data + offset
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = (TIMER_BUS_CLOCK / TIMER_CLOCK) - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIMEx_TISelection(&htim2, TIM_TIM2_TI4_COMP2_OUT, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/*
48us = 48,000ns
48,000ns / resolution (320) = 150ns

1000 / 150ns = 6.6666666666666666666666666666667

80 / 6.6666666666666666666666666666667 = 12

100 / 6.6666666666666666666666666666667 = 15

have to divide result by 2 for 50 percent duty cycle

12/2 = 6
15/2 =

 *
 */
static void spracingPixelOsdPixelTimerInit(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  __HAL_RCC_TIM15_CLK_ENABLE();

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = (TIMER_BUS_CLOCK / TIMER_CLOCK) - 1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = (7.5 * RESOLUTION_SCALE) - 1;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim15, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = ((7.5/2) * RESOLUTION_SCALE);
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
{
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
}


//
// Sync Generation
//

typedef struct syncBufferItem_s {
    // the order and size of these structure is fixed.  the data is transferred by DMA to the timer peripheral, starting with the ARR register
    uint16_t arrValue;
    uint16_t repetitions; // timers only support 8 bit ARR.
    uint16_t cc1Value;
    uint16_t cc2Value;
    uint16_t cc3Value;
} syncBufferItem_t;

#define HALF_LINE(period, repetitions, pulse) _US_TO_CLOCKS(period / 2) - 1, (repetitions) - 1, 0, 0, _US_TO_CLOCKS(pulse) - 1
#define FULL_LINE(period, repetitions, pulse) _US_TO_CLOCKS(period) - 1, (repetitions) - 1, 0, 0, _US_TO_CLOCKS(pulse) - 1

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

void syncInit(void)
{
    hSyncOutDMA = htim1.hdma[TIM_DMA_ID_UPDATE];

    if (!cameraConnected) {
        syncStartPWM();
        syncStartDMA();
    }
}

void syncStartPWM(void)
{
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
    {
      Error_Handler();
    }
}

void syncStopPWM(void)
{
    if (HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3) != HAL_OK)
    {
      Error_Handler();
    }
}

void syncStartDMA(void)
{
    HAL_TIM_DMABurst_MultiWriteStart(
        &htim1,
        TIM_DMABASE_ARR,
        TIM_DMA_UPDATE,
        (uint32_t *)palSyncItems,
        TIM_DMABURSTLENGTH_5TRANSFERS,
        sizeof(palSyncItems) / 2 // 2 because each item is uint16_t, not uint32_t?
    );

    syncDMAActive = true;
}

void syncStopDMA(void)
{
    HAL_TIM_DMABurst_WriteStop(&htim1, TIM_DMA_UPDATE);
}

//
// Pixel Generation
//

void pixelBuffer_createTestPattern1(uint8_t *destinationPixelBuffer, uint8_t bands)
{
    uint8_t pattern = 0;
    uint8_t patterns = 8;
    uint8_t bandWidth = PIXEL_COUNT / bands;
    for (int i = 0; i < PIXEL_COUNT; i++) {
        uint8_t band = i / bandWidth;

        pattern = band % patterns;

        uint8_t pixelValue = 0x00;

        if (pattern == 0) {
            pixelValue = PIXEL_BLACK;
            if (i & 1) {
                pixelValue = PIXEL_WHITE;
            }
        } else if (pattern == 1) {
            pixelValue = PIXEL_WHITE;
        } else if (pattern == 2) {
            pixelValue = PIXEL_TRANSPARENT;
            if (i & 1) {
                pixelValue = PIXEL_BLACK;
            }
        } else if (pattern == 3) {
            pixelValue = PIXEL_TRANSPARENT;
        } else if (pattern == 4) {
            pixelValue = PIXEL_GREY;
        } else if (pattern == 5) {
            pixelValue = PIXEL_TRANSPARENT;
            if (i & 1) {
                pixelValue = PIXEL_WHITE;
            }
        } else if (pattern == 6) {
            pixelValue = PIXEL_WHITE;
            if (i & 1) {
                pixelValue = PIXEL_BLACK;
            }
        } else if (pattern == 7){
            pixelValue = PIXEL_BLACK;
        }

        destinationPixelBuffer[i] = pixelValue;
    }
    destinationPixelBuffer[PIXEL_COUNT] = PIXEL_TRANSPARENT; // IMPORTANT!  The white source/black sink must be disabled before the SYNC signal, otherwise we change the sync voltage level.
}


DMA_HandleTypeDef *hPixelOutDMA;
bool pixelDMAActive = false;

void pixelOutputDisable(void)
{
    bool blackOn = cameraConnected ? false : true;
    //bool blackOn = true;

    HAL_GPIO_WritePin(BLACK_SINK_GPIO_Port, BLACK_SINK_Pin, blackOn ? GPIO_PIN_RESET : GPIO_PIN_SET);

    // turn off black/white
    HAL_GPIO_WritePin(WHITE_SOURCE_GPIO_Port, WHITE_SOURCE_Pin, GPIO_PIN_RESET);
}

void pixelConfigureDMAForNextField(void)
{
#ifdef DEBUG_PIXEL_BUFFER
    HAL_GPIO_WritePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin, fillPixelBuffer == pixelBufferA ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif

    if (HAL_DMA_Start_IT(hPixelOutDMA, (uint32_t)fillPixelBuffer, PIXEL_ADDRESS, PIXEL_BUFFER_SIZE) != HAL_OK)
    {
      //Error_Handler(); // FIXME the first time this is called it fails.
    }

}

void pixelXferCpltCallback(struct __DMA_HandleTypeDef * hdma)
{

#ifdef DEBUG_PIXEL_DMA
    HAL_GPIO_TogglePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin);
#endif

#ifdef STOP_START_PIXEL_TIMER
    (&htim15)->Instance->CR1 &= ~(TIM_CR1_CEN); // stop
    (&htim15)->Instance->SMCR &= ~TIM_SMCR_SMS; // disable slave mode
#endif

    pixelOutputDisable();

    __HAL_TIM_DISABLE_DMA(&htim15, TIM_DMA_CC1);

    pixelConfigureDMAForNextField();

    pixelDMAActive = false;
}

static inline void pixelStartDMA(void);

void pixelInit(void)
{
    hPixelOutDMA = htim15.hdma[TIM_DMA_ID_CC1];
    hPixelOutDMA->XferCpltCallback = pixelXferCpltCallback;

    HAL_TIM_Base_Start(&htim15);
    if (HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1) != HAL_OK)
    {
      // PWM Generation Error
      Error_Handler();
    }

    // Pre-configure DMA, DMA is started in COMP IRQ Handler
    outputPixelBuffer = pixelBufferA;
    fillPixelBuffer = pixelBufferB;
    if (HAL_DMA_Start_IT(hPixelOutDMA, (uint32_t)outputPixelBuffer, PIXEL_ADDRESS, PIXEL_BUFFER_SIZE) != HAL_OK)
    {
      Error_Handler();
    }

    // OC4REF used to gate TIM15
    if (HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
    {
      Error_Handler();
    }
}

static inline void pixelStartDMA(void)
{
#ifdef DEBUG_PIXEL_DMA
    HAL_GPIO_WritePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin, GPIO_PIN_SET);
#endif

    (&htim15)->Instance->CNT = 0;
#ifdef STOP_START_PIXEL_TIMER
    (&htim15)->Instance->SMCR |= TIM_SLAVEMODE_GATED;
    (&htim15)->Instance->CR1 |= (TIM_CR1_CEN);
#endif

    __HAL_TIM_ENABLE_DMA(&htim15, TIM_DMA_CC1);

    pixelDMAActive = true;
}

//
// Sync Detection/Timing
//

void HAL_COMP_MspInit(COMP_HandleTypeDef* hcomp)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hcomp->Instance==COMP2)
  {
  /* USER CODE BEGIN COMP2_MspInit 0 */

  /* USER CODE END COMP2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_COMP12_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**COMP2 GPIO Configuration
    PE11     ------> COMP2_INP
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* COMP2 interrupt Init */
    HAL_NVIC_SetPriority(COMP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(COMP_IRQn);
  /* USER CODE BEGIN COMP2_MspInit 1 */

  /* USER CODE END COMP2_MspInit 1 */
  }

}

static void MX_COMP2_Init(void)
{
  hcomp2.Instance = COMP2;
  hcomp2.Init.InvertingInput = COMP_INPUT_MINUS_DAC1_CH2;
  hcomp2.Init.NonInvertingInput = COMP_INPUT_PLUS_IO2; // COMP2 PE11
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp2.Init.Hysteresis = COMP_HYSTERESIS_LOW; // LOW = fewer transistions near threshold, HIGH = more transitions near threshold.
  hcomp2.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp2.Init.Mode = COMP_POWERMODE_HIGHSPEED;
  hcomp2.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING_FALLING;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
* @brief DAC MSP Initialization
* This function configures the hardware resources used in this example
* @param hdac: DAC handle pointer
* @retval None
*/
void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hdac->Instance==DAC1)
  {
  /* USER CODE BEGIN DAC1_MspInit 0 */

  /* USER CODE END DAC1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_DAC12_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**DAC1 GPIO Configuration
    PA5     ------> DAC1_OUT2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN DAC1_MspInit 1 */

  /* USER CODE END DAC1_MspInit 1 */
  }

}

static void MX_DAC1_Init(void)
{
  DAC_ChannelConfTypeDef sConfig = {0};

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}


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

static inline void pulseError(void) {
#ifdef DEBUG_PULSE_ERRORS
    HAL_GPIO_TogglePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin);
#endif
    frameState.pulseErrors++;
#ifdef DEBUG_PULSE_ERRORS
    HAL_GPIO_TogglePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin);
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

    compState.fallingEdge = (HAL_COMP_GetOutputLevel(hcomp) == COMP_OUTPUT_LEVEL_LOW);
    if (compState.fallingEdge) {
#ifdef DEBUG_COMP_TRIGGER
        compState.triggerLowCount++;
        HAL_GPIO_WritePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin, GPIO_PIN_RESET);
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
                HAL_GPIO_WritePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin, GPIO_PIN_RESET);
#endif
#ifdef DEBUG_LAST_HALF_LINE
                HAL_GPIO_WritePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin, GPIO_PIN_SET);
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
        HAL_GPIO_WritePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin, GPIO_PIN_SET);
#endif

        //
        // check pulse lengths in order from shortest to longest.
        //
        if (pulseLength < VIDEO_SYNC_SHORT_MAX) {
#ifdef DEBUG_SHORT_PULSE
            HAL_GPIO_WritePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin, GPIO_PIN_RESET);
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

                fieldState.phase = FIELD_PRE_EQUALIZING;
                fieldState.preEqualizingPulses = 1; // this one
            } else {
                pulseError();
            }

#ifdef DEBUG_SHORT_PULSE
            HAL_GPIO_WritePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin, GPIO_PIN_SET);
#endif

        } else if (pulseLength < VIDEO_SYNC_HSYNC_MAX) {

            //
            // Important start DMA *NOW* - then deal with remaining state.
            //
            bool thisLineIsVisible = nextLineIsVisible;
            if (thisLineIsVisible) {
                // DMA configured for next line (below) or by transfer-complete handler.
                pixelStartDMA();

                visibleLineIndex++;
            }

            switch(frameState.status) {
            case COUNTING_POST_EQUALIZING_PULSES:
                frameState.status = COUNTING_HSYNC_PULSES;

                fieldState.phase = FIELD_HSYNC;

            FALLTHROUGH;

            case COUNTING_HSYNC_PULSES:

                fieldState.lineNumber++;
                frameState.lineNumber++;

                // prepare for next line
                nextLineIsVisible = fieldState.lineNumber >= firstVisibleLine && fieldState.lineNumber <= lastVisibleLine;

                if (nextLineIsVisible) {
                    uint8_t *previousOutputPixelBuffer = outputPixelBuffer;

                    outputPixelBuffer = fillPixelBuffer;

                    fillLineNow = true;
                    fillLineIndex = visibleLineIndex;

                    fillPixelBuffer = previousOutputPixelBuffer;
                    if (!thisLineIsVisible) {
                        // if this line is visible the transfer-complete handler would configure the DMA
                        pixelConfigureDMAForNextField();
                    }
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

                    frameState.counter++;
                    frameFlag = true;
                } else {
                    fieldState.type = FIELD_SECOND;
                }

                fieldState.lineNumber = 0;
                visibleLineIndex = 0;
                nextLineIsVisible = false;

                fieldState.phase = FIELD_SYNCRONIZING;
#ifdef DEBUG_FIELD_START
                HAL_GPIO_WritePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin, GPIO_PIN_RESET);
#endif
            }

            if (frameState.status == COUNTING_POST_EQUALIZING_PULSES) {
                fieldState.type = FIELD_SECOND;
#ifdef DEBUG_FIELD_START
                HAL_GPIO_WritePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin, GPIO_PIN_SET);
#endif
            }

            if (frameState.status == COUNTING_SYNCRONIZING_PULSES) {
                fieldState.syncronizingPulses++;
            }

#ifdef DEBUG_FIRST_SYNC_PULSE
            bool firstSyncPulse = (fieldState.syncronizingPulses == 0);
            if (firstSyncPulse) {
                HAL_GPIO_WritePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin, GPIO_PIN_SET);
            } else if (fieldState.syncronizingPulses == 1) {
                HAL_GPIO_WritePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin, GPIO_PIN_RESET);
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

//
// Frame
//


void frameBuffer_erase(uint8_t *frameBuffer)
{
    memset(frameBuffer, BLOCK_TRANSPARENT, FRAME_BUFFER_SIZE);
}

void pixelBuffer_fillFromFrameBuffer(uint8_t *destinationPixelBuffer, uint8_t frameBufferIndex, uint16_t lineIndex)
{
#ifdef DEBUG_OUT_GPIO_Port
    HAL_GPIO_TogglePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin);
#endif
    uint8_t *frameBuffer = frameBuffers[frameBufferIndex];
    uint8_t *frameBufferLine = frameBuffer + (FRAME_BUFFER_LINE_SIZE * lineIndex);
    uint8_t *pixel = destinationPixelBuffer;
    for (int i = 0; i < FRAME_BUFFER_LINE_SIZE; i++) {
        uint8_t pixelBlock = *(frameBufferLine + i);

        uint8_t mask = (1 << 7) | ( 1 << 6); // only for BITS_PER_PIXEL == 2
        *pixel++ = (pixelBlock & mask) >> (BITS_PER_PIXEL * 3) << PIXEL_BLACK_BIT;

        mask = mask >> BITS_PER_PIXEL;
        *pixel++ = (pixelBlock & mask) >> (BITS_PER_PIXEL * 2) << PIXEL_BLACK_BIT;

        mask = mask >> BITS_PER_PIXEL;
        *pixel++ = (pixelBlock & mask) >> (BITS_PER_PIXEL * 1) << PIXEL_BLACK_BIT;

        mask = mask >> BITS_PER_PIXEL;
        *pixel++ = (pixelBlock & mask) >> (BITS_PER_PIXEL * 0) << PIXEL_BLACK_BIT;
    }

    destinationPixelBuffer[PIXEL_COUNT] = PIXEL_TRANSPARENT; // IMPORTANT!  The white source/black sink must be disabled before the SYNC signal, otherwise we change the sync voltage level.
#ifdef DEBUG_OUT_GPIO_Port
    HAL_GPIO_TogglePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin);
#endif
}

// unoptimized, avoid over-use.
void frameBuffer_setPixel(uint8_t *frameBuffer, uint16_t x, uint16_t y, uint8_t mode)
{
    uint8_t *lineBuffer = frameBuffer + (y * FRAME_BUFFER_LINE_SIZE);

    uint8_t pixelOffsetInBlock = (PIXELS_PER_BYTE - 1) - (x % PIXELS_PER_BYTE);

    uint8_t pixelBitOffset = BITS_PER_PIXEL * pixelOffsetInBlock;

    uint8_t mask = ~(FRAME_PIXEL_MASK << pixelBitOffset);

    uint8_t before = lineBuffer[x / PIXELS_PER_BYTE];
    uint8_t withMaskCleared = before & mask;
    lineBuffer[x / PIXELS_PER_BYTE] = withMaskCleared |
            (mode << pixelBitOffset);
}

void frameBuffer_createTestPattern1(uint8_t *frameBuffer)
{
    for (int lineIndex = 0; lineIndex < PAL_VISIBLE_LINES; lineIndex++) {
        uint8_t *lineBuffer = frameBuffer + (lineIndex * FRAME_BUFFER_LINE_SIZE);

        if (lineIndex & 0x8) {
            continue; // empty vertical band.
        }

        for (int i = 0; i < PIXEL_COUNT / PIXELS_PER_BYTE; i ++) {

            lineBuffer[i] =
                    (FRAME_PIXEL_WHITE << (BITS_PER_PIXEL * 3)) |
                    (FRAME_PIXEL_GREY << (BITS_PER_PIXEL * 2)) |
                    (FRAME_PIXEL_BLACK << (BITS_PER_PIXEL * 1)) |
                    (FRAME_PIXEL_TRANSPARENT << (BITS_PER_PIXEL * 0));
        }
    }
}

void frameBuffer_createTestPattern2(uint8_t *frameBuffer)
{
    for (int lineIndex = 0; lineIndex < PAL_VISIBLE_LINES; lineIndex++) {
        int x;

        x = lineIndex;
        frameBuffer_setPixel(frameBuffer, x, lineIndex, FRAME_PIXEL_BLACK);
        frameBuffer_setPixel(frameBuffer, x+1, lineIndex, FRAME_PIXEL_WHITE);
        frameBuffer_setPixel(frameBuffer, x+2, lineIndex, FRAME_PIXEL_BLACK);

        x = PIXEL_COUNT - 1 - lineIndex;
        frameBuffer_setPixel(frameBuffer, x, lineIndex, FRAME_PIXEL_BLACK);
        frameBuffer_setPixel(frameBuffer, x-1, lineIndex, FRAME_PIXEL_WHITE);
        frameBuffer_setPixel(frameBuffer, x-2, lineIndex, FRAME_PIXEL_BLACK);

    }
}

bool spracingPixelOSDInit(const struct spracingPixelOSDConfig_s *spracingPixelOSDConfig, const struct vcdProfile_s *vcdProfile)
{
    UNUSED(spracingPixelOSDConfig);
    UNUSED(vcdProfile);

    spracingPixelOSDIO.blackPin = IOGetByTag(IO_TAG(SPRACING_PIXEL_OSD_BLACK_PIN));
    IOHi(spracingPixelOSDIO.blackPin);
    IOInit(spracingPixelOSDIO.blackPin, OWNER_OSD, 0);
    IOConfigGPIO(spracingPixelOSDIO.blackPin, IO_PIXEL_BLACK_CFG);

    spracingPixelOSDIO.whitePin = IOGetByTag(IO_TAG(SPRACING_PIXEL_OSD_WHITE_PIN));
    IOLo(spracingPixelOSDIO.whitePin);
    IOInit(spracingPixelOSDIO.whitePin, OWNER_OSD, 0);
    IOConfigGPIO(spracingPixelOSDIO.whitePin, IO_PIXEL_WHITE_CFG);

    spracingPixelOSDIO.syncInPin = IOGetByTag(IO_TAG(SPRACING_PIXEL_OSD_SYNC_IN_PIN));
    IOLo(spracingPixelOSDIO.syncInPin);
    IOInit(spracingPixelOSDIO.syncInPin, OWNER_OSD, 0);
    IOConfigGPIO(spracingPixelOSDIO.syncInPin, IO_VIDEO_SYNC_IN_CFG);

    //
    // Frame
    //

    frameBuffer_erase(frameBuffers[0]);
    frameBuffer_erase(frameBuffers[1]);

    //frameBuffer_createTestPattern1(frameBuffers[0]);
    //frameBuffer_createTestPattern1(frameBuffers[1]);

    frameBuffer_createTestPattern2(frameBuffers[0]);

    //
    // Sync detection
    //

    MX_COMP2_Init();
    MX_DAC1_Init();

    MX_TIM2_Init();


    // DAC CH2 - Comparator reference

    //uint32_t targetMv = 735; // Sony Board Camera, positive sync voltage
    uint32_t targetMv = 820; // Sony Camera, positive sync voltage
    //uint32_t targetMv = 725; // Pixim Seawolf, negative sync voltage
    int32_t offsetMv = 0;//-28; // scope shows 328mv when targetMv is 300mv

    if (!cameraConnected) {
        targetMv = 700;
    }

    // IMPORTANT: The voltage keeps drifting the longer the camera has been on (rises over time)
    // TODO: auto-correct targetMv based on sync length (shorter = nearer lower level, longer = nearer high level)

    // TODO get measured VREF via ADC and use instead of VIDEO_DAC_VCC here?

    uint32_t dacComparatorRaw = ((targetMv + offsetMv) * 0x0FFF) / (VIDEO_DAC_VCC * 1000);

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dacComparatorRaw);

    HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

    if (HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_4) != HAL_OK)
    {
      Error_Handler();
    }

    //
    // Sync generation
    //

    MX_DMA_Init();

    spracingPixelOSDSyncTimerInit(); // TIM1

    syncInit();

    //
    // Pixel generation
    //

    spracingPixelOsdPixelTimerInit(); // TIM15

    pixelBuffer_createTestPattern1(pixelBufferA, 16);
    pixelBuffer_createTestPattern1(pixelBufferB, 8);

    pixelInit(); // Requires that TIM1 is initialised.

    //
    // Sync detection (enable)
    //

    // Don't enable comparator until sync generation is started
    // TODO - Maybe this is ok now?

    if(HAL_COMP_Start_IT(&hcomp2) != HAL_OK)
    {
      Error_Handler();
    }

    return false;
}

#endif // USE_SPRACING_PIXEL_OSD
