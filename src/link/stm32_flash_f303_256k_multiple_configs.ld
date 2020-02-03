/*
*****************************************************************************
**
**  File        : stm32_flash.ld
**
**  Abstract    : Linker script for STM32F30x Device with
**                256KByte FLASH and 40KByte RAM
**
*****************************************************************************
*/

/* Specify the memory areas. */
MEMORY
{
    FLASH (rx)        : ORIGIN = 0x08000000, LENGTH = 248K
    FLASH_CONFIG_A (r)  : ORIGIN = 0x0803E000, LENGTH = 4K
    FLASH_CONFIG_B (r)  : ORIGIN = 0x0803F000, LENGTH = 4K

    RAM (xrw)         : ORIGIN = 0x20000000, LENGTH = 40K
    CCM (xrw)         : ORIGIN = 0x10000000, LENGTH = 8K
    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
}

__config_start_a = ORIGIN(FLASH_CONFIG_A);
__config_end_a = ORIGIN(FLASH_CONFIG_A) + LENGTH(FLASH_CONFIG_A);

__config_start_b = ORIGIN(FLASH_CONFIG_B);
__config_end_b = ORIGIN(FLASH_CONFIG_B) + LENGTH(FLASH_CONFIG_B);

REGION_ALIAS("STACKRAM", CCM)
REGION_ALIAS("FASTRAM", CCM)

INCLUDE "stm32_flash.ld"
