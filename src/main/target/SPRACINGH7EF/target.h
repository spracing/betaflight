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

#pragma once

#define TARGET_BOARD_IDENTIFIER "SP7E"
#define USBD_PRODUCT_STRING "SPRacingH7EF"

#define USE_TARGET_CONFIG

#define USE_SPRACING_PERSISTENT_RTC_WORKAROUND

#define LED0_PIN                PE5
#define LED1_PIN                PE6

#define USE_BEEPER
#define BEEPER_PIN              PD11
#define BEEPER_INVERTED

// Force two buttons to look at the single button so config reset on button works
#define USE_BUTTONS
#define BUTTON_A_PIN            PD10
#define BUTTON_A_PIN_INVERTED // Active high
#define BUTTON_B_PIN            PD10
#define BUTTON_B_PIN_INVERTED // Active high

// FC has 2 flash chips, one for logging on SPI6, one for code/data storage on OCTOSPIM_P1 (Memory mapped mode)
// Config is to be stored on the flash chip used for code/data storage, this requires that the
// config load/save routines must run from RAM and a method to enable/disable memory mapped mode is needed.

#define USE_OCTOSPI
#define USE_OCTOSPI_DEVICE_1

#define OCTOSPIM_P1_SCK_PIN PB2

#define OCTOSPIM_P1_IO0_PIN NONE
#define OCTOSPIM_P1_IO1_PIN NONE
#define OCTOSPIM_P1_IO2_PIN NONE
#define OCTOSPIM_P1_IO3_PIN NONE

// Using IO4:7
#define OCTOSPIM_P1_IO4_PIN PE7
#define OCTOSPIM_P1_IO5_PIN PE8
#define OCTOSPIM_P1_IO6_PIN PE9
#define OCTOSPIM_P1_IO7_PIN PE10
#define OCTOSPIM_P1_CS_PIN PB10

#define OCTOSPIM_P1_MODE OCTOSPIM_P1_MODE_IO47_ONLY
#define OCTOSPIM_P1_CS_FLAGS (OCTOSPIM_P1_CS_HARDWARE)

#define USE_FLASH_CHIP
#define CONFIG_IN_EXTERNAL_FLASH
//#define CONFIG_IN_SDCARD
//#define CONFIG_IN_RAM
#if !defined(CONFIG_IN_RAM) && !defined(CONFIG_IN_SDCARD) && !defined(CONFIG_IN_EXTERNAL_FLASH)
#error "EEPROM storage location not defined"
#endif

#define USE_UART

#define USE_UART1
#define UART1_RX_PIN            PB7
#define UART1_TX_PIN            PB6

#define USE_UART2
#define UART2_RX_PIN            PD4
#define UART2_TX_PIN            PD5

#define USE_UART3
#define UART3_RX_PIN            PD9
#define UART3_TX_PIN            PD8

#define USE_UART4
#define UART4_RX_PIN            PD0
#define UART4_TX_PIN            PD1

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            NONE

//#define USE_UART6
#define UART6_RX_PIN            NONE
#define UART6_TX_PIN            NONE

//#define USE_UART7
#define UART7_RX_PIN            NONE
#define UART7_TX_PIN            NONE

#define USE_UART8
#define UART8_RX_PIN            PE0
#define UART8_TX_PIN            PE1

#define USE_UART9
#define UART9_RX_PIN            PD14
#define UART9_TX_PIN            PD15

#define USE_UART10
#define UART10_RX_PIN           PE2
#define UART10_TX_PIN           PE3

#define USE_VCP
#define USE_USB_ID

#define SERIAL_PORT_COUNT       9

#define USE_SPI

#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15
#define SPI2_NSS_PIN            PB12

#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12
#define SPI3_NSS_PIN            PA15

#define USE_SPI_DEVICE_6
#define SPI6_SCK_PIN            PB3
#define SPI6_MISO_PIN           PB4
#define SPI6_MOSI_PIN           PB5
#define SPI6_NSS_PIN            PD7 // SOFTWARE CS ONLY, SPI6_NSS signal unavailable (Only on PA15, PA4, PA0)

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB6
#define I2C1_SDA                PB7
#define I2C_DEVICE              (I2CDEV_1)

#define USE_I2C_DEVICE_4
#define I2C4_SCL                PD12
#define I2C4_SDA                PD13

#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883

#define USE_BARO
#define USE_BARO_BMP388

#define USE_GYRO
#define USE_GYRO_SPI_ICM42605
#define USE_MULTI_GYRO
#undef USE_GYRO_REGISTER_DUMP

#define USE_EXTI
#define USE_GYRO_EXTI

// TODO Verify EXTI/FSYNC pins
#define GYRO_1_EXTI_PIN         PC6
#define GYRO_2_EXTI_PIN         PC7
#define GYRO_1_FSYNC_PIN        PC8
#define GYRO_2_FSYNC_PIN        PC9

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW


#define GYRO_1_CS_PIN           SPI3_NSS_PIN
#define GYRO_1_SPI_INSTANCE     SPI3

#define GYRO_2_CS_PIN           SPI2_NSS_PIN
#define GYRO_2_SPI_INSTANCE     SPI2

#define USE_ACC
#define USE_ACC_SPI_ICM42605

#define GYRO_1_ALIGN            CW0_DEG
#define GYRO_2_ALIGN            CW180_DEG

#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_BOTH

#define USE_FLASHFS
#define USE_FLASH_TOOLS
#define USE_FLASH_W25Q128
#define USE_FLASH_M25P16

#define FLASH_CS_PIN            SPI6_NSS_PIN
#define FLASH_SPI_INSTANCE      SPI6

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#ifdef USE_DMA_SPEC
//#define UART1_TX_DMA_OPT        0
//#define UART2_TX_DMA_OPT        1
//#define UART3_TX_DMA_OPT        2
//#define UART4_TX_DMA_OPT        3
//#define UART5_TX_DMA_OPT        4
//#define UART6_TX_DMA_OPT        5
//#define UART7_TX_DMA_OPT        6
//#define UART8_TX_DMA_OPT        7
//#define ADC1_DMA_OPT 8 // ADC1 reserved for VIDEO
//#define ADC2_DMA_OPT 9 // ADC2 not used
#define ADC3_DMA_OPT 10
#else
//#define ADC1_DMA_STREAM DMA2_Stream0 // ADC1 reserved for VIDEO
//#define ADC2_DMA_STREAM DMA2_Stream1 // ADC2 not used
#define ADC3_DMA_STREAM DMA2_Stream2
#endif

#define USE_ADC
#define USE_ADC_INTERNAL // ADC3

#define ADC_INSTANCE ADC3 // Use ADC3 by default, for as many pins as possible.

#define ADC1_INSTANCE ADC1 // ADC1 reserved for VIDEO
//#define ADC2_INSTANCE ADC2 // ADC2 not used
#define ADC3_INSTANCE ADC3 // ADC3 for monitoring, core temp and vrefint

// TODO verify ADC pins
#define RSSI_ADC_PIN                PC0 // ADC3_INP10
#define RSSI_ADC_INSTANCE           ADC3
#define VBAT_ADC_PIN                PC1 // ADC3_INP11
#define VBAT_ADC_INSTANCE           ADC3
#define CURRENT_METER_ADC_PIN       PC2 // ADC3_INP0
#define CURRENT_METER_ADC_INSTANCE  ADC3
#define EXTERNAL1_ADC_PIN           PC3 // ADC3_INP1
#define EXTERNAL1_ADC_INSTANCE      ADC3

// TODO verify ADC pins
#define VIDEO_IN_ADC_PIN        PC4 // ADC1_INP4 - Reserved for video
#define VIDEO_OUT_ADC_PIN       PC5 // ADC1_INP8 - Reserved for video

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define DEFAULT_FEATURES        (FEATURE_RSSI_ADC | FEATURE_TELEMETRY | FEATURE_LED_STRIP)

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff
#define TARGET_IO_PORTG 0xffff

#define USABLE_TIMER_CHANNEL_COUNT 19

#define USED_TIMERS  ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(12) | TIM_N(15) )
