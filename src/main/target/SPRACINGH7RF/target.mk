H730xB_TARGETS += $(TARGET)

HSE_VALUE    = 8000000

ifneq ($(EXST),)
EXST = yes
EXST_ADJUST_VMA = 0x90100000
endif

FEATURES       += VCP ONBOARDFLASH SDCARD_SDIO

TARGET_SRC += \
            drivers/bus_quadspi_hal.c \
            drivers/bus_quadspi.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_icm42605.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_qmc5883l.c \
            drivers/barometer/barometer_bmp388.c \
