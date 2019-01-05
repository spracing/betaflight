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
 *
 * BMP388 Driver author: Dominic Clifton
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"

#include "barometer.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"
#include "drivers/nvic.h"

#include "barometer_bmp388.h"

//#define DEBUG_BARO_BMP388

#if defined(USE_BARO) && (defined(USE_BARO_BMP388) || defined(USE_BARO_SPI_BMP388))

// see Datasheet 3.11.1 Memory Map Trimming Coefficients
typedef struct bmp388_calib_param_s {
    uint16_t T1;
    uint16_t T2;
    int8_t T3;
    int16_t P1;
    int16_t P2;
    int8_t P3;
    int8_t P4;
    uint16_t P5;
    uint16_t P6;
    int8_t P7;
    int8_t P8;
    int16_t P9;
    int8_t P10;
    int8_t P11;
} __attribute__((packed)) bmp388_calib_param_t;

static uint8_t bmp388_chip_id = 0;
STATIC_UNIT_TESTED bmp388_calib_param_t bmp388_cal;
// uncompensated pressure and temperature
uint32_t bmp388_up = 0;
uint32_t bmp388_ut = 0;
STATIC_UNIT_TESTED int64_t t_lin = 0;

static void bmp388_start_ut(baroDev_t *baro);
static void bmp388_get_ut(baroDev_t *baro);
static void bmp388_start_up(baroDev_t *baro);
static void bmp388_get_up(baroDev_t *baro);

STATIC_UNIT_TESTED void bmp388_calculate(int32_t *pressure, int32_t *temperature);

void bmp388_extiHandler(extiCallbackRec_t* cb)
{
#ifdef DEBUG
    static uint32_t bmp388ExtiCallbackCounter = 0;

    bmp388ExtiCallbackCounter++;
#endif

    baroDev_t *baro = container_of(cb, baroDev_t, exti);

    uint8_t intStatus = 0;
    busReadRegisterBuffer(&baro->busdev, BMP388_INT_STATUS_REG, &intStatus, 1);
}

void bmp388BusInit(busDevice_t *busdev)
{
#ifdef USE_BARO_SPI_BMP388
    if (busdev->bustype == BUSTYPE_SPI) {
        IOHi(busdev->busdev_u.spi.csnPin); // Disable
        IOInit(busdev->busdev_u.spi.csnPin, OWNER_BARO_CS, 0);
        IOConfigGPIO(busdev->busdev_u.spi.csnPin, IOCFG_OUT_PP);
        spiSetDivisor(busdev->busdev_u.spi.instance, SPI_CLOCK_STANDARD); // XXX
    }
#else
    UNUSED(busdev);
#endif
}

void bmp388BusDeinit(busDevice_t *busdev)
{
#ifdef USE_BARO_SPI_BMP388
    if (busdev->bustype == BUSTYPE_SPI) {
        spiPreinitCsByIO(busdev->busdev_u.spi.csnPin);
    }
#else
    UNUSED(busdev);
#endif
}

void bmp388BeginForcedMeasurement(busDevice_t *busdev)
{
    // enable pressure measurement, temperature measurement, set power mode and start sampling
    uint8_t mode = (BMP388_MODE_FORCED << 4) | (1 << 1) | (1 << 0);
    busWriteRegister(busdev, BMP388_PWR_CTRL_REG, mode);
}

bool bmp388Detect(const bmp388Config_t *config, baroDev_t *baro)
{
    delay(20);

#if defined(BARO_EXTI_PIN)
    IO_t baroIntIO = IO_NONE;
#endif

#if defined(BARO_EXTI_PIN) && defined(USE_EXTI)
    if (config && config->baroInitExtiTag) {
        baroIntIO = IOGetByTag(config->baroInitExtiTag);
        IOInit(baroIntIO, OWNER_BARO_EXTI, 0);
        EXTIHandlerInit(&baro->exti, bmp388_extiHandler);
        EXTIConfig(baroIntIO, &baro->exti, NVIC_PRIO_BARO_EXTI, IOCFG_IN_FLOATING, EXTI_TRIGGER_RISING);
        EXTIEnable(baroIntIO, true);
    }
#else
    UNUSED(config);
#endif

    busDevice_t *busdev = &baro->busdev;
    bool defaultAddressApplied = false;

    bmp388BusInit(busdev);

    if ((busdev->bustype == BUSTYPE_I2C) && (busdev->busdev_u.i2c.address == 0)) {
        // Default address for BMP388
        busdev->busdev_u.i2c.address = BMP388_I2C_ADDR;
        defaultAddressApplied = true;
    }

    busReadRegisterBuffer(busdev, BMP388_CHIP_ID_REG, &bmp388_chip_id, 1);

    if (bmp388_chip_id != BMP388_DEFAULT_CHIP_ID) {
        bmp388BusDeinit(busdev);
        if (defaultAddressApplied) {
            busdev->busdev_u.i2c.address = 0;
        }
        return false;
    }

    // read calibration
    STATIC_ASSERT(sizeof(bmp388_calib_param_t) == BMP388_TRIMMING_DATA_LENGTH, bmp388_calibration_structure_incorrectly_packed);

#if defined(BARO_EXTI_PIN) && defined(USE_EXTI)
    uint8_t intCtrlValue = (1 << BMP388_INT_DRDY_EN_BIT) | (0 << BMP388_INT_FFULL_EN_BIT) | (0 << BMP388_INT_FWTM_EN_BIT) | (0 << BMP388_INT_LATCH_BIT) | (1 << BMP388_INT_LEVEL_BIT) | (0 << BMP388_INT_OD_BIT);
    busWriteRegister(busdev, BMP388_INT_CTRL_REG, intCtrlValue);
#endif

    busReadRegisterBuffer(busdev, BMP388_TRIMMING_NVM_PAR_T1_LSB_REG, (uint8_t *)&bmp388_cal, sizeof(bmp388_calib_param_t));


    // set oversampling
    busWriteRegister(busdev, BMP388_OSR_REG,
        ((BMP388_PRESSURE_OSR << BMP388_OSR_P_BIT) & BMP388_OSR_P_MASK) |
        ((BMP388_TEMPERATURE_OSR << BMP388_OSR4_T_BIT) & BMP388_OSR4_T_MASK)
    );

    bmp388BeginForcedMeasurement(busdev);

    // these are dummy as temperature is measured as part of pressure
    baro->ut_delay = 0;
    baro->get_ut = bmp388_get_ut;
    baro->start_ut = bmp388_start_ut;
    // only _up part is executed, and gets both temperature and pressure
    baro->start_up = bmp388_start_up;
    baro->get_up = bmp388_get_up;

    // See datasheet 3.9.2 "Measurement rate in forced mode and normal mode"
    baro->up_delay = 234 +
        (392 + (powerf(2, BMP388_PRESSURE_OSR + 1) * 2000)) +
        (313 + (powerf(2, BMP388_TEMPERATURE_OSR + 1) * 2000));

    baro->calculate = bmp388_calculate;

    return true;
}

static void bmp388_start_ut(baroDev_t *baro)
{
    UNUSED(baro);
    // dummy
}

static void bmp388_get_ut(baroDev_t *baro)
{
    UNUSED(baro);
    // dummy
}

static void bmp388_start_up(baroDev_t *baro)
{
    // start measurement
    bmp388BeginForcedMeasurement(&baro->busdev);
}

static void bmp388_get_up(baroDev_t *baro)
{
    uint8_t dataFrame[BMP388_DATA_FRAME_LENGTH];

    // read data from sensor
    busReadRegisterBuffer(&baro->busdev, BMP388_DATA_0_REG, dataFrame, BMP388_DATA_FRAME_LENGTH);

    bmp388_up = ((uint32_t)(dataFrame[0]) << 0) | ((uint32_t)(dataFrame[1]) << 8) | ((uint32_t)(dataFrame[2]) << 16);
    bmp388_ut = ((uint32_t)(dataFrame[3]) << 0) | ((uint32_t)(dataFrame[4]) << 8) | ((uint32_t)(dataFrame[5]) << 16);
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC
static int64_t bmp388_compensate_T(uint32_t uncomp_temperature)
{
    uint64_t partial_data1;
    uint64_t partial_data2;
    uint64_t partial_data3;
    int64_t partial_data4;
    int64_t partial_data5;
    int64_t partial_data6;
    int64_t comp_temp;

    partial_data1 = uncomp_temperature - (256 * bmp388_cal.T1);
    partial_data2 = bmp388_cal.T2 * partial_data1;
    partial_data3 = partial_data1 * partial_data1;
    partial_data4 = (int64_t)partial_data3 * bmp388_cal.T3;
    partial_data5 = ((int64_t)(partial_data2 * 262144) + partial_data4);
    partial_data6 = partial_data5 / 4294967296;
    /* Update t_lin, needed for pressure calculation */
    t_lin = partial_data6;
    comp_temp = (int64_t)((partial_data6 * 25)  / 16384);

#ifdef DEBUG_BARO_BMP388
    debug[0] = comp_temp;
#endif
    return comp_temp;
}

static uint64_t bmp388_compensate_P(uint32_t uncomp_pressure)
{
    int64_t partial_data1;
    int64_t partial_data2;
    int64_t partial_data3;
    int64_t partial_data4;
    int64_t partial_data5;
    int64_t partial_data6;
    int64_t offset;
    int64_t sensitivity;
    uint64_t comp_press;

    partial_data1 = t_lin * t_lin;
    partial_data2 = partial_data1 / 64;
    partial_data3 = (partial_data2 * t_lin) / 256;
    partial_data4 = (bmp388_cal.P8 * partial_data3) / 32;
    partial_data5 = (bmp388_cal.P7 * partial_data1) * 16;
    partial_data6 = (bmp388_cal.P6 * t_lin) * 4194304;
    offset = (bmp388_cal.P5 * 140737488355328) + partial_data4 + partial_data5 + partial_data6;

    partial_data2 = (bmp388_cal.P4 * partial_data3) / 32;
    partial_data4 = (bmp388_cal.P3 * partial_data1) * 4;
    partial_data5 = (bmp388_cal.P2 - 16384) * t_lin * 2097152;
    sensitivity = ((bmp388_cal.P1 - 16384) * 70368744177664) + partial_data2 + partial_data4
            + partial_data5;

    partial_data1 = (sensitivity / 16777216) * uncomp_pressure;
    partial_data2 = bmp388_cal.P10 * t_lin;
    partial_data3 = partial_data2 + (65536 * bmp388_cal.P9);
    partial_data4 = (partial_data3 * uncomp_pressure) / 8192;
    partial_data5 = (partial_data4 * uncomp_pressure) / 512;
    partial_data6 = (int64_t)((uint64_t)uncomp_pressure * (uint64_t)uncomp_pressure);
    partial_data2 = (bmp388_cal.P11 * partial_data6) / 65536;
    partial_data3 = (partial_data2 * uncomp_pressure) / 128;
    partial_data4 = (offset / 4) + partial_data1 + partial_data5 + partial_data3;
    comp_press = (((uint64_t)partial_data4 * 25) / (uint64_t)1099511627776);

#ifdef DEBUG_BARO_BMP388
    debug[1] = ((comp_press & 0xFFFF0000) >> 32);
    debug[2] = ((comp_press & 0x0000FFFF) >>  0);
#endif
    return comp_press;
}

STATIC_UNIT_TESTED void bmp388_calculate(int32_t *pressure, int32_t *temperature)
{
    // calculate
    int64_t t;
    int64_t p;

    t = bmp388_compensate_T(bmp388_ut);
    p = bmp388_compensate_P(bmp388_up);

    if (pressure)
        *pressure = (int32_t)(p / 256);
    if (temperature)
        *temperature = t;
}

#endif
