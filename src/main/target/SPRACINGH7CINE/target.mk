H750xB_TARGETS += $(TARGET)

HSE_VALUE    = 8000000

FEATURES       += VCP ONBOARDFLASH SDCARD_SDIO SPRACING_PIXEL_OSD

EXST ?= yes

ifneq ($(EXST),)
EXST = yes
EXST_ADJUST_VMA = 0x97CE0000
LD_SCRIPT       = $(LINKER_DIR)/stm32_ram_h750_exst_spracingpixelosd.ld
endif

ifneq ($(EXST),yes)
TARGET_FLASH_SIZE := 1024
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_h750_1m_spracingpixelosd.ld
endif

TARGET_SRC += \
            drivers/bus_quadspi_hal.c \
            drivers/bus_quadspi.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/accgyro/accgyro_spi_icm426xx.c \
            drivers/vtx_rtc6705.c \
            drivers/vtx_rtc6705_soft_spi.c \
            $(addprefix drivers/compass/,$(notdir $(wildcard $(SRC_DIR)/drivers/compass/*.c))) \
            $(addprefix drivers/barometer/,$(notdir $(wildcard $(SRC_DIR)/drivers/barometer/*.c))) \

