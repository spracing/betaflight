H730xB_TARGETS += $(TARGET)

HSE_VALUE    = 8000000

# W25Q16JV 16MBit/2MByte flash, 2097152 bytes (1024*1024*2)

# Offsets:
# 0x00000000-0x000FFFFF 1MB Reserved area
# 0x00100000-0x001FFFFF 1MB Firmware area + EXST Hash

# VMA: 
# 0x90000000-0x900FFFFF 1MB Reserved area
# 0x90100000-0x901FFFFF 1MB Firmware area + EXST Hash

# Currently the first 1MB of flash is reserved for the following:
# * Flash layout/partition information
# * Flight controller config
# * OSD firmware
# * OSD fonts
# * OSD logos

EXST ?= yes

ifneq ($(EXST),)
EXST = yes
EXST_ADJUST_VMA = 0x90100000
LD_SCRIPT       = $(LINKER_DIR)/stm32_ram_h730_exst_spracingpixelosd.ld
endif

FEATURES       += VCP ONBOARDFLASH SPRACING_PIXEL_OSD

TARGET_SRC += \
            drivers/bus_quadspi_hal.c \
            drivers/bus_quadspi.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/accgyro/accgyro_spi_icm426xx.c \
            $(addprefix drivers/compass/,$(notdir $(wildcard $(SRC_DIR)/drivers/compass/*.c))) \
            $(addprefix drivers/barometer/,$(notdir $(wildcard $(SRC_DIR)/drivers/barometer/*.c))) \
