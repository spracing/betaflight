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
 * Author: Dominic Clifton - Sync generation, Sync Detection, Video Overlay and first-cut of working OSD system.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SPRACING_PIXEL_OSD

#include "build/debug.h"

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

#include "drivers/spracing_pixel_osd.h"


// Pins 8-15 of GPIOE reserved for OSD use when in GPIO OUTPUT MODE (Upper 8 bits of GPIO port)
// Pins 8-15 on GPIOE *can* be used for other functions, just not GPIO OUTPUT
// although using the BSRR register instead of ODR could also be implemented for greater IO flexibility

#if (SPRACINGH7CINE_REV <= 1)
// Rev A
#define SPRACING_PIXEL_OSD_BLACK_PIN                    PE13
#define SPRACING_PIXEL_OSD_WHITE_PIN                    PE14
#define SPRACING_PIXEL_OSD_RESERVED_PIN                 PE15

#define SPRACING_PIXEL_OSD_SYNC_IN_PIN                  PE11 // COMP2_INP
#define SPRACING_PIXEL_OSD_SYNC_OUT_A_PIN               PE12 // TIM1_CH3N
#define SPRACING_PIXEL_OSD_SYNC_OUT_B_PIN               PA8  // TIM1_CH1 / MCO / Currently Unused
#define SPRACING_PIXEL_OSD_SYNC_OUT_PIN                 SPRACING_PIXEL_OSD_SYNC_OUT_A_PIN
#define USE_TIM1_CH3_FOR_SYNC
#define SYNC_TIMER_CHANNEL TIM_CHANNEL_3

#define SPRACING_PIXEL_OSD_VIDEO_THRESHOLD_DEBUG_PIN    PA5  // DAC1_OUT2
#define SPRACING_PIXEL_OSD_PIXEL_DEBUG_1_PIN            PE5  // TIM15_CH1 - For DMA updates
#define SPRACING_PIXEL_OSD_PIXEL_DEBUG_2_PIN            PE6  // TIM15_CH2 - Spare
#define SPRACING_PIXEL_OSD_PIXEL_GATING_DEBUG_PIN       PB0 // TIM1_CH2N // actual gating is on CH4
#define SPRACING_PIXEL_OSD_PIXEL_BLANKING_DEBUG_PIN     PB1 // TIM1_CH3N // actual blanking is on CH5
#else
// Rev B
#define SPRACING_PIXEL_OSD_WHITE_SOURCE_SELECT_PIN      PE12
#define SPRACING_PIXEL_OSD_BLACK_PIN                    PE13
#define SPRACING_PIXEL_OSD_MASK_ENABLE_PIN              PE14
#define SPRACING_PIXEL_OSD_WHITE_PIN                    PE15

#define SPRACING_PIXEL_OSD_SYNC_IN_PIN                  PE11 // COMP2_INP
#define SPRACING_PIXEL_OSD_SYNC_OUT_PIN                 PA8  // TIM1_CH1
#define USE_TIM1_CH1_FOR_SYNC
#define SYNC_TIMER_CHANNEL TIM_CHANNEL_1

#define SPRACING_PIXEL_OSD_WHITE_SOURCE_PIN             PA4  // DAC1_OUT1
#define SPRACING_PIXEL_OSD_VIDEO_THRESHOLD_DEBUG_PIN    PA5  // DAC1_OUT2
#define SPRACING_PIXEL_OSD_PIXEL_DEBUG_1_PIN            PE5  // TIM15_CH1 - For DMA updates
#define SPRACING_PIXEL_OSD_PIXEL_DEBUG_2_PIN            PE6  // TIM15_CH2 - Spare
#define SPRACING_PIXEL_OSD_PIXEL_GATING_DEBUG_PIN       PB0 // TIM1_CH2N // actual gating is on CH4
#define SPRACING_PIXEL_OSD_PIXEL_BLANKING_DEBUG_PIN     PB1 // TIM1_CH3N // actual blanking is on CH5
#endif

#if 1
#define DEBUG_PULSE_STATISTICS
//#define DEBUG_PULSE_ERRORS      // debug led 2
#define DEBUG_COMP_TRIGGER      // debug led 2
#define DEBUG_OSD_EVENTS
#define DEBUG_PIXEL_DMA         // debug led 1
#define DEBUG_BLANKING          // signal on M8
#define DEBUG_GATING            // signal on M7
#else
#define DEBUG_PIXEL_BUFFER_FILL
#define DEBUG_LAST_HALF_LINE
#define DEBUG_PIXEL_BUFFER
#define DEBUG_SYNC_PWM
#define DEBUG_FIELD_START
#define DEBUG_SHORT_PULSE
#define DEBUG_FIRST_SYNC_PULSE
#endif

static void pixelDebug1Set(bool state);
static void pixelDebug2Set(bool state);
static void pixelDebug1Low(void);
static void pixelDebug2Low(void);
static void pixelDebug1High(void);
static void pixelDebug2High(void);
static void pixelDebug1Toggle(void);
static void pixelDebug2Toggle(void);

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

// 48us * 80mhz = 3840 clocks.  3840 clocks / 720 = 5.33 clocks per pixel.
// resolution scale of 2 = 10.77 clocks per pixel = 360 pixels.
// 5 * 720 = 3600 clocks / 80 = 45us.
// 6 * 720 = 4320 clocks / 80 = 54us == Too long!

// 48us * 100mhz = 4800 clocks.  4800 clocks / 720 = 6.6 clocks per pixel.
// resolution scale of 2 = 13.33 clocks per pixel = 360 pixels.
// 7 * 720 = 5040 clocks / 100 = 50.04us.

#define CLOCKS_PER_PIXEL 6.6

#define HORIZONTAL_RESOLUTION 720
#define RESOLUTION_SCALE 2
#define OVERLAY_LENGTH ((CLOCKS_PER_PIXEL * HORIZONTAL_RESOLUTION) / TIMER_CLOCKS_PER_US) // us


//
// Timing
//

#define TIMER_BUS_CLOCK   200000000
#define TIMER_CLOCK  100000000

#define TIMER_CLOCKS_PER_US                      (TIMER_CLOCK / 1000000)
#define _US_TO_CLOCKS(__us)                      ((uint32_t)((__us) * TIMER_CLOCKS_PER_US))


//
// It takes some time between the comparator being triggered and the IRQ handler beging called.
// it can be measured by togging a GPIO high/low in the IRQ handler and measuring the time between
// the input signal and the gpio being toggled.
// Note: the value varies based on CPU clock-speed and compiler optimisations, i.e. DEBUG build = more time, faster CPU = less time.
//
#define VIDEO_COMPARATOR_TO_IRQ_OFFSET 0.4 // us

#ifdef USE_NTSC
#define VIDEO_LINE_LEN            63.556  // us
#define VIDEO_SYNC_SHORT           2.000  // us
#define VIDEO_SYNC_HSYNC           4.700  // us
#define VIDEO_BLANKING            10.900  // us
#define VIDEO_FRONT_PORCH          1.500  // us
#else
#define VIDEO_LINE_LEN            64.000  // us
#define VIDEO_SYNC_SHORT           2.000  // us
#define VIDEO_SYNC_HSYNC           4.700  // us
#define VIDEO_BLANKING            12.000  // us
#define VIDEO_FRONT_PORCH          1.650  // us
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
#define VIDEO_ACTIVE            (VIDEO_LINE_LEN - VIDEO_BLANKING)
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
#define VIDEO_SYNC_SHORT_MIN    _US_TO_CLOCKS(VIDEO_SYNC_SHORT / 2.0)
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

#define GATING_DEBUG_Pin GPIO_PIN_0
#define GATING_DEBUG_GPIO_Port GPIOB
#define BLANKING_DEBUG_Pin GPIO_PIN_1
#define BLANKING_DEBUG_GPIO_Port GPIOB

#if (SPRACINGH7CINE_REV <= 1)
#define SYNC_OUT_Pin GPIO_PIN_12
#define SYNC_OUT_GPIO_Port GPIOE
#else
#define SYNC_OUT_Pin GPIO_PIN_8
#define SYNC_OUT_GPIO_Port GPIOA
#endif

#define PIXEL_COUNT (HORIZONTAL_RESOLUTION / RESOLUTION_SCALE)
#define PIXEL_BUFFER_SIZE PIXEL_COUNT + 1 // one more pixel which must always be transparent to reset output level during sync

#define USE_PIXEL_OUT_GPIOE

// NOTE: for optimal CPU usage the current design requires that the pixel black and white GPIO bits are adjacent.
// BLACK bit must be before WHITE bit.
#ifdef USE_PIXEL_OUT_GPIOE
#define PIXEL_ODR       GPIOE->ODR
#define PIXEL_ODR_OFFSET 8 // 0 = PE0-PE7, 8 = PE8-PE15

#if (SPRACINGH7CINE_REV <= 1)
#define PIXEL_BLACK_BIT 5 // PE13
#define PIXEL_WHITE_BIT 6 // PE14
#else
#define PIXEL_WHITE_SOURCE_SELECT_BIT   4 // PE12
#define PIXEL_BLACK_BIT                 5 // PE13
#define PIXEL_MASK_ENABLE_BIT           6 // PE14
#define PIXEL_WHITE_BIT                 7 // PE15
#endif
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

#define PIXEL_ADDRESS   ((uint32_t)&(PIXEL_ODR) + (PIXEL_ODR_OFFSET / 8)) // +1 for upper 8 bits

DMA_RAM uint8_t pixelBufferA[PIXEL_BUFFER_SIZE] __attribute__((aligned(32)));
DMA_RAM uint8_t pixelBufferB[PIXEL_BUFFER_SIZE] __attribute__((aligned(32)));

uint8_t *fillPixelBuffer = NULL;
uint8_t *outputPixelBuffer = NULL;

#define PIXEL_WHITE_ON 1
#define PIXEL_WHITE_OFF 0

#define PIXEL_MASK_ON 1
#define PIXEL_MASK_OFF 0

// black is inverted (open drain)
#define PIXEL_BLACK_ON 0
#define PIXEL_BLACK_OFF 1

#define PIXEL_WHITE       ((PIXEL_WHITE_ON  << PIXEL_WHITE_BIT) | (PIXEL_BLACK_OFF << PIXEL_BLACK_BIT))
#define PIXEL_BLACK       ((PIXEL_WHITE_OFF << PIXEL_WHITE_BIT) | (PIXEL_BLACK_ON  << PIXEL_BLACK_BIT))
#define PIXEL_GREY        ((PIXEL_WHITE_ON  << PIXEL_WHITE_BIT) | (PIXEL_BLACK_ON  << PIXEL_BLACK_BIT))
#define PIXEL_TRANSPARENT ((PIXEL_WHITE_OFF << PIXEL_WHITE_BIT) | (PIXEL_BLACK_OFF << PIXEL_BLACK_BIT))

#define PIXEL_WITH_MASK     (PIXEL_MASK_ON << PIXEL_MASK_ENABLE_BIT)
#define PIXEL_WITHOUT_MASK  (PIXEL_MASK_OFF << PIXEL_MASK_ENABLE_BIT)
//
// Frame
//

#define FRAME_BLACK_BIT_OFFSET 0
#define FRAME_WHITE_BIT_OFFSET 1
#define BITS_PER_PIXEL 2 // the current implementation only supports 2.

#define FRAME_PIXEL_WHITE       ((PIXEL_WHITE_ON  << FRAME_WHITE_BIT_OFFSET) | (PIXEL_BLACK_OFF << FRAME_BLACK_BIT_OFFSET))
#define FRAME_PIXEL_BLACK       ((PIXEL_WHITE_OFF << FRAME_WHITE_BIT_OFFSET) | (PIXEL_BLACK_ON  << FRAME_BLACK_BIT_OFFSET))
#define FRAME_PIXEL_GREY        ((PIXEL_WHITE_ON  << FRAME_WHITE_BIT_OFFSET) | (PIXEL_BLACK_ON  << FRAME_BLACK_BIT_OFFSET))
#define FRAME_PIXEL_TRANSPARENT ((PIXEL_WHITE_OFF << FRAME_WHITE_BIT_OFFSET) | (PIXEL_BLACK_OFF << FRAME_BLACK_BIT_OFFSET))

#define FRAME_PIXEL_MASK        ((1 << 1) | (1 << 0))

#define BLOCK_TRANSPARENT ((FRAME_PIXEL_TRANSPARENT << 6) | (FRAME_PIXEL_TRANSPARENT << 4) | (FRAME_PIXEL_TRANSPARENT << 2) | (FRAME_PIXEL_TRANSPARENT << 0))

#define BLOCK_DEBUG ((FRAME_PIXEL_WHITE << 6) | (FRAME_PIXEL_BLACK << 4) | (FRAME_PIXEL_GREY << 2) | (FRAME_PIXEL_TRANSPARENT << 0))

#define PIXELS_PER_BYTE (8 / BITS_PER_PIXEL)

#define BITS_PER_BYTE 8

#define FRAME_BUFFER_LINE_SIZE ((PIXEL_COUNT / BITS_PER_BYTE) * BITS_PER_PIXEL)
#define FRAME_BUFFER_SIZE (FRAME_BUFFER_LINE_SIZE * PAL_VISIBLE_LINES)

DMA_RAM uint8_t frameBuffers[2][FRAME_BUFFER_SIZE] __attribute__((aligned(32)));

void pixelBuffer_fillFromFrameBuffer(uint8_t *destinationPixelBuffer, uint8_t frameBufferIndex, uint16_t lineIndex);

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
    uint32_t frameStartCounter;
    uint32_t validFrameCounter;
    uint32_t lineCounter;
    uint16_t pulseErrors;
    uint16_t totalPulseErrors;
    frameStatus_t status;
    uint32_t vsyncAt;
} frameState_t;


volatile int16_t frameFallingEdgeIndex = 0;
volatile uint8_t fallingEdgesRemainingBeforeNewField = 0;

uint16_t firstVisibleLine;
uint16_t lastVisibleLine;
bool nextLineIsVisible;
volatile uint16_t visibleLineIndex;

volatile bool frameStartFlag = false;
volatile uint16_t fillLineIndex = 0;

#ifdef DEBUG_PULSE_STATISTICS
uint16_t syncPulseRisingStatisticIndex = 0;
uint16_t syncPulseRisingStatistics[PAL_LINES] __attribute__((used));
uint16_t syncPulseFallingStatisticIndex = 0;
uint16_t syncPulseFallingStatistics[PAL_LINES] __attribute__((used));
#endif

//
// State
//

volatile bool cameraConnected = true;
static uint32_t osdFrameTimeoutAt = 0;

typedef struct spracingPixelOSDIO_s {
    IO_t blackPin;
    IO_t whitePin;
    IO_t syncInPin;
    IO_t debug1Pin;
    IO_t debug2Pin;
#ifdef DEBUG_BLANKING
    IO_t blankingDebugPin;
#endif
#ifdef DEBUG_GATING
    IO_t gatingDebugPin;
#endif
    IO_t whiteSourceSelectPin;
    IO_t maskEnablePin;
} spracingPixelOSDIO_t;

static spracingPixelOSDIO_t spracingPixelOSDIO = {
    .blackPin               = IO_NONE,
    .whitePin               = IO_NONE,
    .syncInPin              = IO_NONE,
    .debug1Pin              = IO_NONE,
    .debug2Pin              = IO_NONE,
#ifdef DEBUG_BLANKING
    .blankingDebugPin       = IO_NONE,
#endif
#ifdef DEBUG_GATING
    .gatingDebugPin         = IO_NONE,
#endif
    .whiteSourceSelectPin   = IO_NONE,
    .maskEnablePin          = IO_NONE,
};

#define IO_PIXEL_BLACK_CFG                  IO_CONFIG(GPIO_MODE_OUTPUT_OD, GPIO_SPEED_FREQ_MEDIUM,  GPIO_NOPULL)
#define IO_PIXEL_WHITE_CFG                  IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_MEDIUM,  GPIO_NOPULL)

#define IO_PIXEL_MASK_ENABLE_CFG            IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_MEDIUM,  GPIO_PULLDOWN)
#define IO_PIXEL_WHITE_SOURCE_SELECT_CFG    IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_MEDIUM,  GPIO_PULLDOWN)

#define IO_PIXEL_DEBUG_CFG                  IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_MEDIUM,  GPIO_PULLDOWN)

#define IO_VIDEO_SYNC_IN_CFG                IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_LOW,     GPIO_NOPULL)

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
// Sync Generation
//

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
    pixelDebug2Set(fillPixelBuffer == pixelBufferA);
#endif

    if (HAL_DMA_Start_IT(hPixelOutDMA, (uint32_t)fillPixelBuffer, PIXEL_ADDRESS, PIXEL_BUFFER_SIZE) != HAL_OK)
    {
      //Error_Handler(); // FIXME the first time this is called it fails.
    }

}

static inline void disableComparatorBlanking(void) {
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

void pixelXferCpltCallback(struct __DMA_HandleTypeDef *hdma)
{
    UNUSED(hdma);

#ifdef DEBUG_PIXEL_DMA
    pixelDebug1Low();
#endif

#ifdef STOP_START_PIXEL_TIMER
    (&htim15)->Instance->CR1 &= ~(TIM_CR1_CEN); // stop
    (&htim15)->Instance->SMCR &= ~TIM_SMCR_SMS; // disable slave mode
#endif

    pixelOutputDisable();

    __HAL_TIM_DISABLE_DMA(&htim15, TIM_DMA_CC1);

    //disableComparatorBlanking();


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
}

void pixelGateAndBlankStart(void)
{
    // OC4REF used to gate TIM15
    if (HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
    {
      Error_Handler();
    }
#ifdef DEBUG_GATING
    if (HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
    {
      Error_Handler();
    }
#endif

    // OC5 used as comparator blanking
    if (HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_5) != HAL_OK)
    {
      Error_Handler();
    }
#ifdef DEBUG_BLANKING
    if (HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
    {
      Error_Handler();
    }
#endif
}

static inline void pixelStartDMA(void)
{
#ifdef DEBUG_PIXEL_DMA
    pixelDebug1High();
#endif

    (&htim15)->Instance->CNT = 0;
#ifdef STOP_START_PIXEL_TIMER
    (&htim15)->Instance->SMCR |= TIM_SLAVEMODE_GATED;
    (&htim15)->Instance->CR1 |= (TIM_CR1_CEN);
#endif

    __HAL_TIM_ENABLE_DMA(&htim15, TIM_DMA_CC1);

    pixelDMAActive = true;
}

static void pixelStopDMA(void)
{
    __HAL_TIM_DISABLE_DMA(&htim15, TIM_DMA_CC1);

    HAL_DMA_Abort(hPixelOutDMA);

    pixelDMAActive = false;

    pixelOutputDisable();

    disableComparatorBlanking();
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

typedef enum {
    OUTPUT_DISABLED = 0,
    SEARCHING_FOR_LINE_MIN_LEVEL,
    SEARCHING_FOR_FRAME_MIN_LEVEL,
    SEARCHING_FOR_FRAME_MAX_LEVEL,
    GENERATING_VIDEO,
} pixelOsdState_t;

pixelOsdState_t pixelOsdState = SEARCHING_FOR_LINE_MIN_LEVEL;

uint32_t targetMv = 0;

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

//
// Frame
//

uint8_t *frameBuffer_getBuffer(uint8_t index)
{
    uint8_t *frameBuffer = frameBuffers[index];
    return frameBuffer;
}

DMA_RAM uint32_t fillColor __attribute__((aligned(32)));
static MDMA_HandleTypeDef     frameBuffer_MDMA_Handle_Erase = { 0 };

void frameBuffer_eraseInit(void)
{
    __HAL_RCC_MDMA_CLK_ENABLE();

    frameBuffer_MDMA_Handle_Erase.Instance = MDMA_Channel0;
    frameBuffer_MDMA_Handle_Erase.Init.Request              = MDMA_REQUEST_SW;
    frameBuffer_MDMA_Handle_Erase.Init.TransferTriggerMode  = MDMA_REPEAT_BLOCK_TRANSFER;
    frameBuffer_MDMA_Handle_Erase.Init.Priority             = MDMA_PRIORITY_HIGH;
    frameBuffer_MDMA_Handle_Erase.Init.Endianness           = MDMA_LITTLE_ENDIANNESS_PRESERVE;

    frameBuffer_MDMA_Handle_Erase.Init.DataAlignment        = MDMA_DATAALIGN_PACKENABLE;
    frameBuffer_MDMA_Handle_Erase.Init.BufferTransferLength = 128;

    frameBuffer_MDMA_Handle_Erase.Init.DestinationInc       = MDMA_DEST_INC_WORD;
    frameBuffer_MDMA_Handle_Erase.Init.DestDataSize         = MDMA_DEST_DATASIZE_WORD;
    frameBuffer_MDMA_Handle_Erase.Init.DestBurst            = MDMA_DEST_BURST_SINGLE;
    frameBuffer_MDMA_Handle_Erase.Init.DestBlockAddressOffset    = 0;

    frameBuffer_MDMA_Handle_Erase.Init.SourceInc            = MDMA_SRC_INC_DISABLE;
    frameBuffer_MDMA_Handle_Erase.Init.SourceDataSize       = MDMA_SRC_DATASIZE_WORD;
    frameBuffer_MDMA_Handle_Erase.Init.SourceBurst          = MDMA_SOURCE_BURST_SINGLE;
    frameBuffer_MDMA_Handle_Erase.Init.SourceBlockAddressOffset  = 0;

    if (HAL_MDMA_Init(&frameBuffer_MDMA_Handle_Erase) != HAL_OK) {
      Error_Handler();
    }

    fillColor = BLOCK_TRANSPARENT << 24 | BLOCK_TRANSPARENT << 16 | BLOCK_TRANSPARENT << 8 | BLOCK_TRANSPARENT;
    //fillColor = BLOCK_DEBUG << 24 | BLOCK_DEBUG << 16 | BLOCK_DEBUG << 8 | BLOCK_DEBUG;
}

void frameBuffer_erase(uint8_t *frameBuffer)
{
#if 0
    memset(frameBuffer, BLOCK_TRANSPARENT, FRAME_BUFFER_SIZE);
#else
    uint32_t hal_status;

    uint32_t bytesRemaining = FRAME_BUFFER_SIZE;

    while (bytesRemaining > 0) {

        uint32_t bytesToFill = 4096 * sizeof(fillColor); // 4096 is maximum block count.
        if (bytesRemaining < bytesToFill) {
            bytesToFill = bytesRemaining;
        }

        uint32_t offset = FRAME_BUFFER_SIZE - bytesRemaining;

        hal_status = HAL_MDMA_Start(&frameBuffer_MDMA_Handle_Erase, (uint32_t)&fillColor,
                                                  (uint32_t)frameBuffer + offset,
                                                  sizeof(fillColor),
                                                  bytesToFill / sizeof(fillColor));
        if(hal_status != HAL_OK)
        {
          /* Transfer Error */
          Error_Handler();
        }

        const uint32_t eraseTimeoutTicks = 1000;
        hal_status = HAL_MDMA_PollForTransfer(&frameBuffer_MDMA_Handle_Erase, HAL_MDMA_FULL_TRANSFER, eraseTimeoutTicks);
        if(hal_status != HAL_OK)
        {
          /* Transfer Error */
          Error_Handler();
        }

        bytesRemaining -= bytesToFill;
    }
#endif
}

#if 0
void pixelBuffer_fillFromFrameBuffer(uint8_t *destinationPixelBuffer, uint8_t frameBufferIndex, uint16_t lineIndex)
{
    // This method only works for BITS_PER_PIXEL == 2
    // And only when PIXEL_BLACK_BIT == WHITE_BLACK_BIT - 1 (adjacent GPIO pins)

#ifdef DEBUG_PIXEL_BUFFER_FILL
    pixelDebug2Toggle();
#endif
    uint8_t *frameBuffer = frameBuffers[frameBufferIndex];
    uint8_t *frameBufferLine = frameBuffer + (FRAME_BUFFER_LINE_SIZE * lineIndex);
#if USE_SLOW_PIXEL_BUFFER_FILL_METHOD
    // XXX - After compilier optimization this is slower than the implementation below.
    uint8_t *pixel = destinationPixelBuffer;
    for (int i = 0; i < FRAME_BUFFER_LINE_SIZE; i++) {
        uint8_t frameBlock = *(frameBufferLine + i);

        uint8_t mask = (1 << 7) | ( 1 << 6);
        *pixel++ = (frameBlock & mask) >> (BITS_PER_PIXEL * 3) << PIXEL_BLACK_BIT;

        mask = mask >> BITS_PER_PIXEL;
        *pixel++ = (frameBlock & mask) >> (BITS_PER_PIXEL * 2) << PIXEL_BLACK_BIT;

        mask = mask >> BITS_PER_PIXEL;
        *pixel++ = (frameBlock & mask) >> (BITS_PER_PIXEL * 1) << PIXEL_BLACK_BIT;

        mask = mask >> BITS_PER_PIXEL;
        *pixel++ = (frameBlock & mask) >> (BITS_PER_PIXEL * 0) << PIXEL_BLACK_BIT;
    }
#else
    uint32_t *pixels = (uint32_t *)destinationPixelBuffer;
    for (int i = 0; i < FRAME_BUFFER_LINE_SIZE; i++) {
        uint8_t frameBlock = *(frameBufferLine + i);

        *pixels++ = (
            ((frameBlock & (0x03 << 0)) >> (BITS_PER_PIXEL * 0) << 24) |
            ((frameBlock & (0x03 << 2)) >> (BITS_PER_PIXEL * 1) << 16) |
            ((frameBlock & (0x03 << 4)) >> (BITS_PER_PIXEL * 2) << 8) |
            ((frameBlock & (0x03 << 6)) >> (BITS_PER_PIXEL * 3) << 0)
        ) << PIXEL_BLACK_BIT;
    }
#endif
    destinationPixelBuffer[PIXEL_COUNT] = PIXEL_TRANSPARENT; // IMPORTANT!  The white source/black sink must be disabled before the SYNC signal, otherwise we change the sync voltage level.
#ifdef DEBUG_PIXEL_BUFFER_FILL
    pixelDebug2Toggle();
#endif
}
#else


void pixelBuffer_fillFromFrameBuffer(uint8_t *destinationPixelBuffer, uint8_t frameBufferIndex, uint16_t lineIndex)
{
    // Rev B has 4 IO lines for White Source, Black, Mask and White, black and white are NOT adjacent so the bits cannot be copied and shifted together...
#ifdef DEBUG_PIXEL_BUFFER_FILL
    pixelDebug2Toggle();
#endif

    uint8_t *frameBuffer = frameBuffers[frameBufferIndex];
    uint8_t *frameBufferLine = frameBuffer + (FRAME_BUFFER_LINE_SIZE * lineIndex);

    uint32_t *pixels = (uint32_t *)destinationPixelBuffer;
    for (int i = 0; i < FRAME_BUFFER_LINE_SIZE; i++) {
        uint8_t frameBlock = *(frameBufferLine + i);

#if 0 // old
        uint32_t blackBits = (
            ((frameBlock & (0x01 << 0)) >> (BITS_PER_PIXEL * 0) << 24) |
            ((frameBlock & (0x01 << 2)) >> (BITS_PER_PIXEL * 1) << 16) |
            ((frameBlock & (0x01 << 4)) >> (BITS_PER_PIXEL * 2) << 8) |
            ((frameBlock & (0x01 << 6)) >> (BITS_PER_PIXEL * 3) << 0)
        );

        uint32_t whiteBits = (
            ((frameBlock & (0x02 << 0)) >> (BITS_PER_PIXEL * 0) << 24) |
            ((frameBlock & (0x02 << 2)) >> (BITS_PER_PIXEL * 1) << 16) |
            ((frameBlock & (0x02 << 4)) >> (BITS_PER_PIXEL * 2) << 8) |
            ((frameBlock & (0x02 << 6)) >> (BITS_PER_PIXEL * 3) << 0)
        );

        *pixels++ = blackBits << (PIXEL_BLACK_BIT - FRAME_BLACK_BIT_OFFSET)
            | whiteBits << (PIXEL_WHITE_BIT - FRAME_WHITE_BIT_OFFSET)
            | whiteBits << (PIXEL_MASK_ENABLE_BIT - FRAME_WHITE_BIT_OFFSET);

        /*
        uint32_t gpioBits = 0;
        gpioBits |= blackBits << (PIXEL_BLACK_BIT - FRAME_BLACK_BIT_OFFSET);
        gpioBits |= whiteBits << (PIXEL_WHITE_BIT - FRAME_WHITE_BIT_OFFSET);
        gpioBits |= whiteBits << (PIXEL_MASK_ENABLE_BIT - FRAME_WHITE_BIT_OFFSET);
        */
#else

        uint32_t frameBlockBits = (
            ((frameBlock & (0x03 << 0)) >> (BITS_PER_PIXEL * 0) << 24) |
            ((frameBlock & (0x03 << 2)) >> (BITS_PER_PIXEL * 1) << 16) |
            ((frameBlock & (0x03 << 4)) >> (BITS_PER_PIXEL * 2) << 8) |
            ((frameBlock & (0x03 << 6)) >> (BITS_PER_PIXEL * 3) << 0)
        );

        uint32_t blackGpioBitMask  = ((1 << 24) | (1 << 16) | (1 << 8) | (1 << 0)) << PIXEL_BLACK_BIT;
        uint32_t whiteGpioBitMask  = ((1 << 24) | (1 << 16) | (1 << 8) | (1 << 0)) << PIXEL_WHITE_BIT;
        uint32_t maskGpioBitMask   = ((1 << 24) | (1 << 16) | (1 << 8) | (1 << 0)) << PIXEL_MASK_ENABLE_BIT;


        // gpio/frame level for black is inverted, so 0 = ON, 1 = OFF.

        uint32_t gpioBlackBits = (frameBlockBits << (PIXEL_BLACK_BIT - FRAME_BLACK_BIT_OFFSET)) & blackGpioBitMask;
        uint32_t gpioWhiteBits = (frameBlockBits << (PIXEL_WHITE_BIT - FRAME_WHITE_BIT_OFFSET)) & whiteGpioBitMask;

        uint32_t gpioNotBlackBits = ~(gpioBlackBits) & blackGpioBitMask; // now 1 = ON, 0 = OFF, for each black bit.

        uint32_t frameMaskOnBlackBits    = gpioBlackBits >> (PIXEL_BLACK_BIT);
        uint32_t frameMaskOnNotBlackBits = gpioNotBlackBits >> (PIXEL_BLACK_BIT);
        uint32_t frameMaskOnWhiteBits    = gpioWhiteBits >> (PIXEL_WHITE_BIT);

        uint32_t gpioMaskOnBlackBits = (frameMaskOnBlackBits << PIXEL_MASK_ENABLE_BIT) & maskGpioBitMask;
        uint32_t gpioMaskOnNotBlackBits = (frameMaskOnNotBlackBits << PIXEL_MASK_ENABLE_BIT) & maskGpioBitMask;
        uint32_t gpioMaskOnWhiteBits = (frameMaskOnWhiteBits << PIXEL_MASK_ENABLE_BIT) & maskGpioBitMask;

        uint32_t gpioWhiteBitsForEachBlackOn = (frameMaskOnNotBlackBits << PIXEL_WHITE_BIT) & whiteGpioBitMask;

        //uint32_t gpioBits = gpioBlackBits | gpioWhiteBits; // works fine, not using mask
        uint32_t gpioBits = gpioBlackBits | gpioWhiteBits | gpioMaskOnNotBlackBits; // doesn't work, why? - because voltage goes to 0 and comparator triggers!

        //uint32_t gpioBits = gpioBlackBits | gpioWhiteBits | gpioMaskOnNotBlackBits | gpioWhiteBitsForEachBlackOn; // works!  Needs to be a voltage source when black so that black level is not 0v, i.e. black level must be above comparator threshold.
        //uint32_t gpioBits = gpioBlackBits | gpioWhiteBits | gpioMaskOnWhiteBits | gpioMaskOnNotBlackBits | gpioWhiteBitsForEachBlackOn; // works too, whites are a bit grey though, but 'white' level is masked correctly - white pixels always the same white regargless of camera signal.

        //uint32_t gpioBits = gpioBlackBits | gpioWhiteBits | gpioMaskOnWhiteBits; // works but white pixels are a bit dark. Doesn't work when using BLOCK_DEBUG fill.
        //uint32_t gpioBits = gpioBlackBits | gpioWhiteBits | gpioMaskOnWhiteBits | gpioMaskOnNotBlackBits; // doesn't work
        //uint32_t gpioBits = gpioBlackBits | gpioWhiteBits | gpioMaskOnWhiteBits | gpioMaskOnBlackBits; // doesn't work,

        //uint32_t gpioBits = gpioBlackBits | gpioMaskOnWhiteBits; // doesn't work.  odd frame is legible during v line level detection.
        //uint32_t gpioBits = gpioBlackBits | gpioMaskOnNotBlackBits; // doesn't work.

        //uint32_t gpioBits = BLOCK_TRANSPARENT | gpioMaskOnNotBlackBits; // doesn't work.
        //uint32_t gpioBits = BLOCK_TRANSPARENT | gpioMaskOnWhiteBits; // doesn't work.
        *pixels++ = gpioBits;
#endif

    }
    destinationPixelBuffer[PIXEL_COUNT] = PIXEL_TRANSPARENT & ~(PIXEL_MASK_ON << PIXEL_MASK_ENABLE_BIT); // IMPORTANT!  The white source/black sink must be disabled before the SYNC signal, otherwise we change the sync voltage level.
#ifdef DEBUG_PIXEL_BUFFER_FILL
    pixelDebug2Toggle();
#endif
}
#endif


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

// unoptimized for now
void frameBuffer_slowWriteCharacter(uint8_t *frameBuffer, uint16_t x, uint16_t y, uint8_t characterIndex)
{
    uint16_t fontCharacterOffset = characterIndex * FONT_MAX7456_12x18_BYTES_PER_CHARACTER;

    for (int row = 0; row < FONT_MAX7456_HEIGHT; row++) {
        uint16_t fy = y + row;
        uint16_t fx = x;

        for (int b = 0; b < 3; b++) {
            uint8_t c = font_max7456_12x18[fontCharacterOffset];
            fontCharacterOffset++;

            for (int p = 0; p <= 3; p++) {
                uint8_t mp = (c >> (2 * (3 - p))) & ((1 << 1) | (1 << 0)); // extract max7456 pixel from character
                uint8_t mode = FRAME_PIXEL_TRANSPARENT;

                if (mp == ((0 << 1) | (0 << 0))) {
                    mode = FRAME_PIXEL_BLACK;
                } else if (mp == ((1 << 1) | (0 << 0))) {
                    mode = FRAME_PIXEL_WHITE;
                }
                if (mode != 0xFF) {
                    frameBuffer_setPixel(frameBuffer, fx, fy, mode);
                }
                fx++;
            }
        }

    }
}

// unoptimized for now
void frameBuffer_slowWriteString(uint8_t *frameBuffer, uint16_t x, uint16_t y, const uint8_t *message, uint8_t messageLength)
{
    uint16_t fx = x;
    for (int mi = 0; mi < messageLength; mi++) {
#if USE_FONT_MAPPING
        uint8_t c = font_max7456_12x18_asciiToFontMapping[message[mi]];
#else
        uint8_t c = message[mi];
#endif

        frameBuffer_slowWriteCharacter(frameBuffer, fx, y, c);
        fx+= 12; // font width
    }
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
// Debug
//

static inline void pixelDebug1Set(bool state)
{
    IOWrite(spracingPixelOSDIO.debug1Pin, state);
}

static inline void pixelDebug1Low(void)
{
    IOLo(spracingPixelOSDIO.debug1Pin);
}

static inline void pixelDebug1High(void)
{
    IOHi(spracingPixelOSDIO.debug1Pin);
}

static inline void pixelDebug1Toggle(void)
{
    IOToggle(spracingPixelOSDIO.debug1Pin);
}

static inline void pixelDebug2Set(bool state)
{
    IOWrite(spracingPixelOSDIO.debug2Pin, state);
}

static inline void pixelDebug2Low(void)
{
    IOLo(spracingPixelOSDIO.debug2Pin);
}

static inline void pixelDebug2High(void)
{
    IOHi(spracingPixelOSDIO.debug2Pin);
}

static inline void pixelDebug2Toggle(void)
{
    IOToggle(spracingPixelOSDIO.debug2Pin);
}

//
// Init
//

bool spracingPixelOSDInit(const struct spracingPixelOSDConfig_s *spracingPixelOSDConfig, const struct vcdProfile_s *vcdProfile)
{
    UNUSED(spracingPixelOSDConfig);
    UNUSED(vcdProfile);

    spracingPixelOSDIO.blackPin = IOGetByTag(IO_TAG(SPRACING_PIXEL_OSD_BLACK_PIN));
    IOHi(spracingPixelOSDIO.blackPin);
    IOInit(spracingPixelOSDIO.blackPin, OWNER_OSD, 0);
    IOConfigGPIO(spracingPixelOSDIO.blackPin, IO_PIXEL_BLACK_CFG);

#ifdef SPRACING_PIXEL_OSD_MASK_ENABLE_PIN
    spracingPixelOSDIO.maskEnablePin = IOGetByTag(IO_TAG(SPRACING_PIXEL_OSD_MASK_ENABLE_PIN));
    IOLo(spracingPixelOSDIO.maskEnablePin); // Low = Mask disabled, High = Mask Enabled.
    IOInit(spracingPixelOSDIO.maskEnablePin, OWNER_OSD, 0);
    IOConfigGPIO(spracingPixelOSDIO.maskEnablePin, IO_PIXEL_MASK_ENABLE_CFG);
#endif

    spracingPixelOSDIO.whitePin = IOGetByTag(IO_TAG(SPRACING_PIXEL_OSD_WHITE_PIN));
    IOLo(spracingPixelOSDIO.whitePin);
    IOInit(spracingPixelOSDIO.whitePin, OWNER_OSD, 0);
    IOConfigGPIO(spracingPixelOSDIO.whitePin, IO_PIXEL_WHITE_CFG);

#ifdef SPRACING_PIXEL_OSD_WHITE_SOURCE_SELECT_PIN
    spracingPixelOSDIO.whiteSourceSelectPin = IOGetByTag(IO_TAG(SPRACING_PIXEL_OSD_WHITE_SOURCE_SELECT_PIN));
    IOLo(spracingPixelOSDIO.whiteSourceSelectPin); // Low = Fixed Voltage, High = Linked to DAC1_OUT1 voltage.
    IOInit(spracingPixelOSDIO.whiteSourceSelectPin, OWNER_OSD, 0);
    IOConfigGPIO(spracingPixelOSDIO.whiteSourceSelectPin, IO_PIXEL_WHITE_SOURCE_SELECT_CFG);
#endif

    spracingPixelOSDIO.syncInPin = IOGetByTag(IO_TAG(SPRACING_PIXEL_OSD_SYNC_IN_PIN));
    IOLo(spracingPixelOSDIO.syncInPin);
    IOInit(spracingPixelOSDIO.syncInPin, OWNER_OSD, 0);
    IOConfigGPIO(spracingPixelOSDIO.syncInPin, IO_VIDEO_SYNC_IN_CFG);

#ifdef DEBUG_BLANKING
    spracingPixelOSDIO.blankingDebugPin = IOGetByTag(IO_TAG(SPRACING_PIXEL_OSD_PIXEL_BLANKING_DEBUG_PIN));
    IOLo(spracingPixelOSDIO.blankingDebugPin);
    IOInit(spracingPixelOSDIO.blankingDebugPin, OWNER_OSD, 0);
    IOConfigGPIO(spracingPixelOSDIO.blankingDebugPin, IO_PIXEL_DEBUG_CFG);
#endif

#ifdef DEBUG_GATING
    spracingPixelOSDIO.gatingDebugPin = IOGetByTag(IO_TAG(SPRACING_PIXEL_OSD_PIXEL_GATING_DEBUG_PIN));
    IOLo(spracingPixelOSDIO.gatingDebugPin);
    IOInit(spracingPixelOSDIO.gatingDebugPin, OWNER_OSD, 0);
    IOConfigGPIO(spracingPixelOSDIO.gatingDebugPin, IO_PIXEL_DEBUG_CFG);
#endif

    spracingPixelOSDIO.debug1Pin = IOGetByTag(IO_TAG(SPRACING_PIXEL_OSD_PIXEL_DEBUG_1_PIN));
    IOLo(spracingPixelOSDIO.debug1Pin);
    IOInit(spracingPixelOSDIO.debug1Pin, OWNER_OSD, 0);
    IOConfigGPIO(spracingPixelOSDIO.debug1Pin, IO_PIXEL_DEBUG_CFG);

    spracingPixelOSDIO.debug2Pin = IOGetByTag(IO_TAG(SPRACING_PIXEL_OSD_PIXEL_DEBUG_2_PIN));
    IOLo(spracingPixelOSDIO.debug2Pin);
    IOInit(spracingPixelOSDIO.debug2Pin, OWNER_OSD, 0);
    IOConfigGPIO(spracingPixelOSDIO.debug2Pin, IO_PIXEL_DEBUG_CFG);


    //
    // Frame
    //

    frameBuffer_eraseInit();

    frameBuffer_erase(frameBuffers[0]);
    frameBuffer_erase(frameBuffers[1]);

    //frameBuffer_createTestPattern1(frameBuffers[0]);
    //frameBuffer_createTestPattern1(frameBuffers[1]);

    frameBuffer_createTestPattern2(frameBuffers[0]);

    frameBuffer_slowWriteString(frameBuffers[0], 50, 150, (uint8_t*)"SP RACING PIXEL OSD", 19);

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

    pixelInit();

    pixelGateAndBlankStart(); // Requires that TIM1 is initialised.

    // IRQ handler requires that timers are initialised.
    if(HAL_COMP_Start_IT(&hcomp2) != HAL_OK)
    {
      Error_Handler();
    }

    return true;
}

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

uint32_t pulseErrorsPerSecond = 0;
uint32_t framesPerSecond = 0;

syncDetectionState_t syncDetectionState = { 0 };
static uint32_t nextEventAt = 0;

bool spracingPixelOSDShouldProcessNow(timeUs_t currentTimeUs)
{
    bool handleEventNow = cmp32(currentTimeUs, nextEventAt) > 0;
    return handleEventNow;
}

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

void spracingPixelOSDPause(void)
{
    // Note: This doesn't stop everything, e.g. comparator and all timers

    if (!cameraConnected) {
        syncStopDMA();
        syncStopPWM();
    }

    pixelStopDMA();
}

void spracingPixelOSDRestart(void)
{
    spracingPixelOSDSyncTimerInit();
    pixelGateAndBlankStart();

    //spracingPixelOSDSyncTriggerReset();
    setComparatorTargetMv(0);
    syncInit();

    memset(&frameState, 0x00, sizeof(frameState));
    memset(&fieldState, 0x00, sizeof(fieldState));
    memset(&syncDetectionState, 0x00, sizeof(syncDetectionState));
}


#define MAXIMIM_LINE_LEVEL_THRESHOLD_MV 2000
#define MAXIMIM_FRAME_LEVEL_THRESHOLD_MV (MAXIMIM_LINE_LEVEL_THRESHOLD_MV + 1000)
#define MAXIMIM_FRAME_LEVEL_DIFFERENCE_MV 400 // was 300
#define MAXIMIM_LINE_LEVEL_DIFFERENCE_MV 300


void spracingPixelOSDProcess(timeUs_t currentTimeUs)
{
    const uint16_t requiredLines = 100; // ~64us * 100 = 6.4ms

    const uint32_t lineCounterDelayUs = (VIDEO_LINE_LEN) * (requiredLines);
    const uint32_t minimumFrameDelayUs = (VIDEO_LINE_LEN) * (PAL_LINES + 10);

    debug[0] = frameState.validFrameCounter;
    debug[1] = frameState.totalPulseErrors;

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

                    //debug[1] = lastTotalPulseErrors;
                    debug[2] = pulseErrorsPerSecond;

                    int32_t recentFrames = frameState.validFrameCounter - lastValidFrameCounter;
                    framesPerSecond = recentFrames * 1000000 / timeDeltaUs;
                    debug[3] = framesPerSecond;

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

void spracingPixelOSDDrawDebugOverlay(void)
{
    uint8_t *frameBuffer = frameBuffer_getBuffer(0);

    static uint8_t messageBuffer[32];

    uint16_t debugY = 18 * 13;

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
