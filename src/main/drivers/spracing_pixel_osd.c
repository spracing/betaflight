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
// State
//

volatile bool cameraConnected = false;

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

static void MX_TIM1_Init(void)
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
      // COMP1 trigger resets timer (i.e. when camera connected)
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

  if (HAL_TIMEx_RemapConfig(&htim1, TIM_TIM1_ETR_COMP1_OUT) != HAL_OK)
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
  htim2.Init.Prescaler = 0;
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
  if (HAL_TIMEx_RemapConfig(&htim2, TIM_TIM2_TI4_COMP2_OUT) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

static void spracingPixelOsdPixelTimerInit(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  __HAL_RCC_TIM15_CLK_ENABLE();

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = (6 * RESOLUTION_SCALE) - 1;
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
  sConfigOC.Pulse = (3 * RESOLUTION_SCALE);
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
    HAL_TIM_DMABurst_WriteStart(
        &htim1,
        TIM_DMABASE_ARR,
        TIM_DMA_UPDATE,
        (uint32_t *)palSyncItems,
        sizeof(palSyncItems) / 2, // 2 because each item is uint16_t, not uint32_t?
        TIM_DMABURSTLENGTH_5TRANSFERS
    );

    syncDMAActive = true;
}

void syncStopDMA(void)
{
    HAL_TIM_DMABurst_WriteStop(&htim1, TIM_DMA_UPDATE);
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

    MX_DMA_Init();

    spracingPixelOsdPixelTimerInit(); // TIM15

    MX_TIM1_Init();
    MX_TIM2_Init();

    //
    // Sync generation
    //

    syncInit();


    return false;
}

#endif // USE_SPRACING_PIXEL_OSD
