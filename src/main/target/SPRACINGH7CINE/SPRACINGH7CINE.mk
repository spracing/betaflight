FEATURES       += VCP ONBOARDFLASH SDCARD_SDIO

ifneq ($(EXST),)
EXST = yes
endif

ifneq ($(EXST),yes)
TARGET_FLASH_SIZE := 1024
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_h750_1m.ld
endif

TARGET_SRC += \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/accgyro/accgyro_spi_icm42605.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/barometer/barometer_bmp388.c \
            drivers/vtx_rtc6705_soft_spi.c \
            drivers/spracing_pixel_osd.c \
            io/displayport_spracing_pixel_osd.c \

