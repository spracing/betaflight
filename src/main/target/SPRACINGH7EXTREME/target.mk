H750xB_TARGETS += $(TARGET)

HSE_VALUE    = 8000000

FEATURES       += VCP ONBOARDFLASH SDCARD_SDIO


ifneq ($(EXST),)
EXST = yes
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
endif

ifneq ($(EXST),yes)
TARGET_FLASH_SIZE := 1024
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_h750_1024k.ld
endif


TARGET_SRC += \
            drivers/bus_quadspi_hal.c \
            drivers/bus_quadspi.c \
            drivers/max7456.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_qmc5883l.c \
            drivers/barometer/barometer_bmp388.c \
            drivers/vtx_rtc6705.c \
            drivers/vtx_rtc6705_soft_spi.c
