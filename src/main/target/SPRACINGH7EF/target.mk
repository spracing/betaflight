H730xB_TARGETS += $(TARGET)

HSE_VALUE    = 8000000

ifneq ($(EXST),)
EXST = yes
EXST_ADJUST_VMA = 0x90100000
endif

ifneq ($(EXST),yes)
# pretend there's 1MB of flash, on the H750 there is, H730 requires testing
TARGET_FLASH_SIZE   := 1024
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_h730_1m.ld
endif

FEATURES       += VCP ONBOARDFLASH

TARGET_SRC += \
            drivers/bus_quadspi_hal.c \
            drivers/bus_quadspi.c \
            drivers/max7456.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_icm42605.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_qmc5883l.c \
            drivers/barometer/barometer_bmp388.c \
