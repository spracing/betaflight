/*
 * Author: Dominic Clifton - Sync generation, Sync Detection, Video Overlay and first-cut of working OSD system.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SPRACING_PIXEL_OSD

#include "build/debug.h"

#include "common/maths.h"
#include "common/printf.h"

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/nvic.h"
#include "drivers/io.h"
#include "drivers/timer.h"
#include "drivers/time.h"

#include "drivers/display.h"

#include "pg/spracing_pixel_osd.h"
#include "pg/vcd.h"

#include "osd/font_max7456_12x18.h"

#include "configuration.h"
#include "framebuffer.h"
#include "pixelbuffer.h"
#include "pixeltiming.h"
#include "videotiming.h"
#include "videoframe.h"
#include "syncdetection.h"
#include "syncgeneration.h"
#include "io.h"
#include "glue.h"

#include "drivers/spracingpixelosd/spracing_pixel_osd.h"

// All 8 pins of the OSD GPIO port are reserved for OSD use if any are using GPIO OUTPUT MODE
// The 8 pins on the OSD GPIO port *can* be used for other functions, just not GPIO OUTPUT, e.g. mixing QUADSPI_BK2 and 4 GPIO pins on GPIOE on the H750 is fine.
// Note: using the BSRR register instead of ODR could also be implemented for greater IO flexibility.


//
// Output Specification
//

// 48us * 80mhz = 3840 clocks.  3840 clocks / 720 = 5.33 clocks per pixel.
// resolution scale of 2 = 10.77 clocks per pixel = 360 pixels.
// 5 * 720 = 3600 clocks / 80 = 45us.
// 6 * 720 = 4320 clocks / 80 = 54us == Too long!

// 48us * 100mhz = 4800 clocks.  4800 clocks / 720 = 6.6 clocks per pixel.
// resolution scale of 2 = 13.33 clocks per pixel = 360 pixels.
// 7 * 720 = 5040 clocks / 100 = 50.04us.

#define CLOCKS_PER_PIXEL 6.6

#define OVERLAY_LENGTH ((CLOCKS_PER_PIXEL * HORIZONTAL_RESOLUTION) / TIMER_CLOCKS_PER_US) // us

//
// State
//

volatile bool cameraConnected = true;
volatile videoMode_t detectedVideoMode = MODE_UNKNOWN;

//
// Sync Detection/Timing
//

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim15;
DMA_HandleTypeDef hdma_tim1_up;
DMA_HandleTypeDef hdma_tim15_ch1;
COMP_HandleTypeDef hcomp2;
DAC_HandleTypeDef hdac1;

//
// Init
//

static void avoidMCO1SyncClash(void)
{
#ifdef STM32H7
    if ((SYNC_OUT_Pin == GPIO_PIN_8) && (SYNC_OUT_GPIO_Port == GPIOA)) {
        // See MCO1_GPIO_PORT & MCO1_PIN in stm32h7xx_hal_rcc.c

        // PA8 is MCO by default, which interferes with the comparator trigger
        GPIO_InitTypeDef GPIO_InitStruct = {0};

        HAL_GPIO_WritePin(SYNC_OUT_GPIO_Port, SYNC_OUT_Pin, GPIO_PIN_RESET);

        GPIO_InitStruct.Pin = SYNC_OUT_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(SYNC_OUT_GPIO_Port, &GPIO_InitStruct);
    }
#endif
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */

      __HAL_RCC_GPIOE_CLK_ENABLE();
#if defined(DEBUG_GATING) || defined(DEBUG_BLANKING)
      __HAL_RCC_GPIOB_CLK_ENABLE();
#endif

      /**TIM1 GPIO Configuration
      PE12     ------> TIM1_CH3N
      OR
      PA8      ------> TIM1_CH1
      */
      GPIO_InitStruct.Pin = SYNC_OUT_Pin;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
      HAL_GPIO_Init(SYNC_OUT_GPIO_Port, &GPIO_InitStruct);

#ifdef DEBUG_GATING
      GPIO_InitStruct.Pin = GATING_DEBUG_Pin;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
      HAL_GPIO_Init(GATING_DEBUG_GPIO_Port, &GPIO_InitStruct);
#endif

#ifdef DEBUG_BLANKING
      GPIO_InitStruct.Pin = BLANKING_DEBUG_Pin;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
      HAL_GPIO_Init(BLANKING_DEBUG_GPIO_Port, &GPIO_InitStruct);
#endif

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
    GPIO_InitStruct.Pin = PIXEL_DEBUG_1_Pin;
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
    UNUSED(descriptor);
    //    HAL_DMA_IRQHandler(TimHandle.hdma[descriptor->userParam]);
    HAL_DMA_IRQHandler(&hdma_tim1_up);
}

void PIXEL_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    UNUSED(descriptor);
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

    const dmaResource_t *syncDmaRef = syncDmaSpec->ref;
    uint32_t syncDmaChannel = syncDmaSpec->channel;
#else
    const dmaResource_t *syncDmaRef = syncTimerHardware->dmaRef;
    uint32_t syncDmaChannel = syncTimerHardware->dmaChannel;
#endif

    UNUSED(syncDmaChannel);

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

    const dmaResource_t *pixelDmaRef = pixelDmaSpec->ref;
    uint32_t pixelDmaChannel = pixelDmaSpec->channel;
#else
    const dmaResource_t *pixelDmaRef = pixelTimerHardware->dmaRef;
    uint32_t pixelDmaChannel = pixelTimerHardware->dmaChannel;
#endif

    UNUSED(pixelDmaChannel);

    uint16_t pixelTimerChannel = pixelTimerHardware->channel;
    uint16_t pixelDmaIndex = timerDmaIndex(pixelTimerChannel);

    dmaInit(dmaGetIdentifier(pixelDmaRef), OWNER_OSD, 0);
    dmaSetHandler(dmaGetIdentifier(pixelDmaRef), PIXEL_DMA_IRQHandler, NVIC_PRIO_VIDEO_DMA, pixelDmaIndex);
}

static void spracingPixelOSDSyncTriggerReset(void)
{

    TIM_SlaveConfigTypeDef sSlaveConfig = {0};

    if (cameraConnected) {
        // COMP2 trigger resets timer when camera connected
        sSlaveConfig.SlaveMode = TIM_SLAVEMODE_COMBINED_RESETTRIGGER;
        sSlaveConfig.InputTrigger = TIM_TS_ETRF;
    } else {
        sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
        sSlaveConfig.InputTrigger = TIM_TS_ITR0;
    }

    sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
    sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
    sSlaveConfig.TriggerFilter = 0;
    if (HAL_TIM_SlaveConfigSynchronization(&htim1, &sSlaveConfig) != HAL_OK)
    {
      Error_Handler();
    }
}

static void spracingPixelOSDSyncTimerInit(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
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

  spracingPixelOSDSyncTriggerReset();

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

  // SYNC output channel
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = _US_TO_CLOCKS(4.7);
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, SYNC_TIMER_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  // offset can be between 0 and 4, 4 = (VIDEO_LINE_LEN(~64) - hsync(4.7) - back porch(5.8) - front porch (1.5)) - OVERLAY_LENGTH
  sConfigOC.Pulse = _US_TO_CLOCKS((4.7 + 5.8) + (2.0)); // start of picture data + offset
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

#ifdef DEBUG_GATING
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
#endif

  // Channel 5 used to blank comparator for duration of visible portion of line
  sConfigOC.OCMode = TIM_OCMODE_FORCED_ACTIVE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_5) != HAL_OK)
  {
    Error_Handler();
  }

#ifdef DEBUG_BLANKING
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
#endif



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
  htim15.Init.Period = (CLOCKS_PER_PIXEL * RESOLUTION_SCALE) - 1;
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
  sConfigOC.Pulse = ((CLOCKS_PER_PIXEL / 2) * RESOLUTION_SCALE);
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
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_INVERTED;
  hcomp2.Init.Hysteresis = COMP_HYSTERESIS_LOW; // LOW = more transitions near threshold, HIGH = fewer transitions near threshold,  H7: LOW=10mV, MEDIUM=20mV, HIGH=30mV
  //hcomp2.Init.BlankingSrce = COMP_BLANKINGSRC_NONE; // during pixel output this this will be COMP_BLANKINGSRC_TIM1_OC5.
  hcomp2.Init.BlankingSrce = COMP_BLANKINGSRC_TIM1_OC5;
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
    PA4     ------> DAC1_OUT1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
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
  /** DAC channel OUT1/2 config
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
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

void setVideoSourceVoltageMv(uint32_t whiteMv)
{
    // TODO get measured VREF via ADC and use instead of VIDEO_DAC_VCC here?
    uint32_t dacWhiteRaw = (whiteMv * 0x0FFF) / (VIDEO_DAC_VCC * 1000);

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacWhiteRaw);
}


//
// Layer support
//
// Foreground only for now.

bool spracingPixelOSDLayerSupported(displayPortLayer_e layer)
{
    if (layer == DISPLAYPORT_LAYER_FOREGROUND) {
        return true;
    } else {
        return false;
    }
}

bool spracingPixelOSDLayerSelect(displayPortLayer_e layer)
{
    if (spracingPixelOSDLayerSupported(layer)) {
        return true;
    } else {
        return false;
    }
}

bool spracingPixelOSDLayerCopy(displayPortLayer_e destLayer, displayPortLayer_e sourceLayer)
{
    UNUSED(sourceLayer);
    UNUSED(destLayer);
    return false;
}

//
// Init
//
static bool spracingPixelOSDInitialised = false;

bool spracingPixelOSDIsInitialised(void) {
    return spracingPixelOSDInitialised;
}

bool spracingPixelOSDInit(const struct spracingPixelOSDConfig_s *spracingPixelOSDConfig, const struct vcdProfile_s *vcdProfile)
{
    UNUSED(spracingPixelOSDConfig);
    UNUSED(vcdProfile);

    spracingPixelOSD_initIO();


    //
    // Frame
    //

    frameBuffer_eraseInit();

    uint8_t * fb0 = frameBuffer_getBuffer(0);
    uint8_t * fb1 = frameBuffer_getBuffer(1);

    frameBuffer_erase(fb0);
    frameBuffer_erase(fb1);

    //frameBuffer_createTestPattern1(fb0);
    //frameBuffer_createTestPattern1(fb1);

    //frameBuffer_createTestPattern2(fb0);

    framebuffer_drawRectangle(fb0, 0, 0, PIXEL_COUNT - 1, PAL_VISIBLE_LINES - 1, FRAME_PIXEL_WHITE);


    frameBuffer_slowWriteString(fb0, 50, 150, (uint8_t*)"SP RACING PIXEL OSD", 19);

    //
    // Sync detection
    //

    avoidMCO1SyncClash();

    MX_COMP2_Init();
    MX_DAC1_Init();

    MX_TIM2_Init();


    // DAC CH2 - Generate comparator reference voltage

    setComparatorTargetMv(0);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

    // DAC CH1 - White voltage

    setVideoSourceVoltageMv(2500);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);



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

#ifdef DEBUG
    extern uint8_t pixelBufferA[];
    extern uint8_t pixelBufferB[];

    pixelBuffer_createTestPattern1(pixelBufferA, 16);
    pixelBuffer_createTestPattern1(pixelBufferB, 8);
#endif

    pixelInit();

    pixelGateAndBlankStart(); // Requires that TIM1 is initialised.

    // IRQ handler requires that timers are initialised.
    if(HAL_COMP_Start_IT(&hcomp2) != HAL_OK)
    {
      Error_Handler();
    }

    spracingPixelOSDInitialised = true;

    return true;
}

void spracingPixelOSDPause(void)
{
    // Note: This doesn't stop everything, e.g. comparator and all timers

    if (!cameraConnected) {
        syncStopDMA();
        syncStopPWM();
    }

    pixelStopDMA();
    disableComparatorBlanking();
}

void spracingPixelOSDRestart(void)
{
    spracingPixelOSDSyncTimerInit();
    pixelGateAndBlankStart();

    //spracingPixelOSDSyncTriggerReset();
    setComparatorTargetMv(0);
    syncInit();

    videoFrame_reset();
    syncDetection_reset();
}


void spracingPixelOSDDrawDebugOverlay(void)
{
    uint8_t *frameBuffer = frameBuffer_getBuffer(0);

    static uint8_t messageBuffer[32];

    uint16_t debugY = FONT_MAX7456_HEIGHT * 13;

    static const char *videoModeNames[] = { "????", "PAL", "NTSC" };
    tfp_sprintf((char *)messageBuffer, "P:%04X V:%04X E:%04X M:%04s",
            frameState.totalPulseErrors,
            frameState.validFrameCounter,
            frameState.frameStartCounter - frameState.validFrameCounter,
            videoModeNames[detectedVideoMode]
    );
    int messageLength = strlen((char *)messageBuffer);
    frameBuffer_slowWriteString(frameBuffer, (360 - (12 * messageLength)) / 2, debugY, messageBuffer, messageLength);

    debugY += FONT_MAX7456_HEIGHT;

    tfp_sprintf((char *)messageBuffer, "L:%03d FL:%03d FH:%03d T:%03d",
            syncDetectionState.minimumLevelForLineThreshold,
            syncDetectionState.minimumLevelForValidFrameMv,
            syncDetectionState.maximumLevelForValidFrameMv,
            syncDetectionState.syncThresholdMv
    );
    messageLength = strlen((char *)messageBuffer);
    frameBuffer_slowWriteString(frameBuffer, (360 - (12 * messageLength)) / 2, debugY, messageBuffer, messageLength);

}

#endif // USE_SPRACING_PIXEL_OSD
