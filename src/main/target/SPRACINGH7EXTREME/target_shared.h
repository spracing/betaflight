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

#define LED0_PIN                PE3

// Force two buttons to look at the single button so config reset on button works
#define USE_BUTTONS
#define	BUTTON_A_PIN            PE4
#define BUTTON_A_PIN_INVERTED // Active high
#define	BUTTON_B_PIN            PE4
#define BUTTON_B_PIN_INVERTED // Active high

#define USE_QUADSPI
#define USE_QUADSPI_DEVICE_1

#define QUADSPI1_SCK_PIN PB2

#define QUADSPI1_BK1_IO0_PIN PD11
#define QUADSPI1_BK1_IO1_PIN PD12
#define QUADSPI1_BK1_IO2_PIN PE2
#define QUADSPI1_BK1_IO3_PIN PD13
#define QUADSPI1_BK1_CS_PIN PB10

#define QUADSPI1_BK2_IO0_PIN PE7
#define QUADSPI1_BK2_IO1_PIN PE8
#define QUADSPI1_BK2_IO2_PIN PE9
#define QUADSPI1_BK2_IO3_PIN PE10
#define QUADSPI1_BK2_CS_PIN NONE

#define QUADSPI1_MODE QUADSPI_MODE_BK1_ONLY
#define QUADSPI1_CS_FLAGS (QUADSPI_BK1_CS_HARDWARE | QUADSPI_BK2_CS_NONE | QUADSPI_CS_MODE_LINKED)

#define USE_FLASH_CHIP
#define EEPROM_IN_EXTERNAL_FLASH
//#define EEPROM_IN_SDCARD
//#define EEPROM_IN_RAM
#if !defined(EEPROM_IN_RAM) && !defined(EEPROM_IN_SDCARD) && !defined(EEPROM_IN_EXTERNAL_FLASH)
#error "EEPROM storage location not defined"
#endif

#define EEPROM_SIZE     4096

#ifdef EEPROM_SIZE
extern uint8_t eepromData[EEPROM_SIZE];
#define __config_start (*eepromData)
#define __config_end (*ARRAYEND(eepromData))
#endif

// USE_SPI only required due to flash driver, but in this example the flash is connected to the QUADSPI interface.
#define USE_SPI
#define USE_SPI_DEVICE_4
#define SPI4_SCK_PIN            PE12
#define SPI4_MISO_PIN           PE13
#define SPI4_MOSI_PIN           PE14
#define SPI4_NSS_PIN            PE11


#define USE_FLASH_W25N01G
#define FLASH_QUADSPI_INSTANCE    QUADSPI

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff
#define TARGET_IO_PORTG 0xffff

