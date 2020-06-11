TARGET_MCU        := STM32H750xx
TARGET_MCU_FAMILY := STM32H7

HSE_VALUE          = 8000000

ifneq ($(EXST),yes)
TARGET_FLASH_SIZE   := 1024
LD_SCRIPT            = $(LINKER_DIR)/stm32_flash_h750_1m.ld
endif

ifeq ($(EXST),yes)
# EXST flash chip configuration

ifeq ($(EXST_FLASH_CHIP),)
EXST_FLASH_CHIP = W25N01GV
endif

ifeq ($(EXST_FLASH_CHIP),W25N01GV)
EXST_ADJUST_VMA = 0x97CE0000 # 128MB NAND flash
else
ifeq ($(EXST_FLASH_CHIP),W25Q128JV)
EXST_ADJUST_VMA = 0x90F90000 # 16MB NOR flash
endif
endif

#ifeq ($(EXST_ADJUST_VMA),)
#EXST_ADJUST_VMA = 0x97CE0000
#endif

endif