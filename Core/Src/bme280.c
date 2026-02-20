/* bme280_simple.c - Simplified BME280 driver with only used functions */
#include "bme280.h"

/* Bit manipulation macros */
#define BME280_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | ((data << bitname##_POS) & bitname##_MSK))
#define BME280_GET_BITS(reg_data, bitname) ((reg_data & (bitname##_MSK)) >> (bitname##_POS))
#define BME280_GET_BITS_POS_0(reg_data, bitname) (reg_data & (bitname##_MSK))
#define BME280_SET_BITS_POS_0(reg_data, bitname, data) ((reg_data & ~(bitname##_MSK)) | (data & bitname##_MSK))

/* Register bit positions and masks */
#define BME280_SENSOR_MODE_POS          0
#define BME280_SENSOR_MODE_MSK          0x03
#define BME280_CTRL_PRESS_POS           2
#define BME280_CTRL_PRESS_MSK           0x1C
#define BME280_CTRL_TEMP_POS            5
#define BME280_CTRL_TEMP_MSK            0xE0
#define BME280_CTRL_HUM_POS             0
#define BME280_CTRL_HUM_MSK             0x07
#define BME280_FILTER_POS               2
#define BME280_FILTER_MSK               0x1C
#define BME280_STANDBY_POS              5
#define BME280_STANDBY_MSK              0xE0
#define BME280_STATUS_IM_UPDATE         0x01

/* Bit shift definitions */
#define BME280_4_BIT_SHIFT              4
#define BME280_8_BIT_SHIFT              8
#define BME280_12_BIT_SHIFT             12

/* Concatenate two bytes */
#define BME280_CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)

/* Global I2C handle - set during setup */
static I2C_HandleTypeDef* g_hi2c = NULL;

/* Internal function prototypes */
static int8_t bme280_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
static int8_t bme280_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
static void bme280_delay_us(uint32_t period, void *intf_ptr);
static int8_t bme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, bme280_dev_t *dev);
static int8_t bme280_set_regs(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, bme280_dev_t *dev);
static int8_t bme280_soft_reset(bme280_dev_t *dev);
static int8_t get_calib_data(bme280_dev_t *dev);
static void parse_temp_press_calib_data(const uint8_t *reg_data, bme280_dev_t *dev);
static void parse_humidity_calib_data(const uint8_t *reg_data, bme280_dev_t *dev);
static void parse_sensor_data(const uint8_t *reg_data, uint32_t *temp, uint32_t *press, uint32_t *hum);
static double compensate_temperature(uint32_t uncomp_temperature, bme280_calib_data_t *calib_data);
static double compensate_pressure(uint32_t uncomp_pressure, const bme280_calib_data_t *calib_data);
static double compensate_humidity(uint32_t uncomp_humidity, const bme280_calib_data_t *calib_data);

/* I2C interface functions */
static int8_t bme280_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = BME280_I2C_ADDR;

    if (HAL_I2C_Mem_Read(g_hi2c, dev_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT,
                         data, len, HAL_MAX_DELAY) != HAL_OK) {
        return BME280_E_COMM_FAIL;
    }
    return BME280_OK;
}

static int8_t bme280_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = BME280_I2C_ADDR;

    if (HAL_I2C_Mem_Write(g_hi2c, dev_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT,
                          (uint8_t*)data, len, HAL_MAX_DELAY) != HAL_OK) {
        return BME280_E_COMM_FAIL;
    }
    return BME280_OK;
}

static void bme280_delay_us(uint32_t period, void *intf_ptr)
{
    uint32_t delay_ms = (period + 999) / 1000;
    if (delay_ms == 0) delay_ms = 1;
    HAL_Delay(delay_ms);
}

/* Register access functions */
static int8_t bme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, bme280_dev_t *dev)
{
    int8_t rslt = dev->read(reg_addr, reg_data, len, dev->intf_ptr);
    if (dev->intf_rslt != BME280_INTF_RET_SUCCESS) {
        rslt = BME280_E_COMM_FAIL;
    }
    return rslt;
}

static int8_t bme280_set_regs(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, bme280_dev_t *dev)
{
    int8_t rslt = dev->write(reg_addr, reg_data, len, dev->intf_ptr);
    if (dev->intf_rslt != BME280_INTF_RET_SUCCESS) {
        rslt = BME280_E_COMM_FAIL;
    }
    return rslt;
}

/* Soft reset function */
static int8_t bme280_soft_reset(bme280_dev_t *dev)
{
    int8_t rslt;
    uint8_t status_reg = 0;
    uint8_t try_run = 5;
    uint8_t soft_rst_cmd = BME280_SOFT_RESET_COMMAND;

    rslt = bme280_set_regs(BME280_REG_RESET, &soft_rst_cmd, 1, dev);

    if (rslt == BME280_OK) {
        do {
            dev->delay_us(BME280_STARTUP_DELAY, dev->intf_ptr);
            rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev);
        } while ((rslt == BME280_OK) && (try_run--) && (status_reg & BME280_STATUS_IM_UPDATE));

        if (status_reg & BME280_STATUS_IM_UPDATE) {
            rslt = BME280_E_NVM_COPY_FAILED;
        }
    }
    return rslt;
}

/* Calibration data functions */
static int8_t get_calib_data(bme280_dev_t *dev)
{
    int8_t rslt;
    uint8_t calib_data[BME280_LEN_TEMP_PRESS_CALIB_DATA] = {0};

    rslt = bme280_get_regs(BME280_REG_TEMP_PRESS_CALIB_DATA, calib_data, BME280_LEN_TEMP_PRESS_CALIB_DATA, dev);

    if (rslt == BME280_OK) {
        parse_temp_press_calib_data(calib_data, dev);
        rslt = bme280_get_regs(BME280_REG_HUMIDITY_CALIB_DATA, calib_data, BME280_LEN_HUMIDITY_CALIB_DATA, dev);

        if (rslt == BME280_OK) {
            parse_humidity_calib_data(calib_data, dev);
        }
    }
    return rslt;
}

static void parse_temp_press_calib_data(const uint8_t *reg_data, bme280_dev_t *dev)
{
    bme280_calib_data_t *calib_data = &dev->calib_data;

    calib_data->dig_t1 = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    calib_data->dig_t2 = (int16_t)BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
    calib_data->dig_t3 = (int16_t)BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);
    calib_data->dig_p1 = BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
    calib_data->dig_p2 = (int16_t)BME280_CONCAT_BYTES(reg_data[9], reg_data[8]);
    calib_data->dig_p3 = (int16_t)BME280_CONCAT_BYTES(reg_data[11], reg_data[10]);
    calib_data->dig_p4 = (int16_t)BME280_CONCAT_BYTES(reg_data[13], reg_data[12]);
    calib_data->dig_p5 = (int16_t)BME280_CONCAT_BYTES(reg_data[15], reg_data[14]);
    calib_data->dig_p6 = (int16_t)BME280_CONCAT_BYTES(reg_data[17], reg_data[16]);
    calib_data->dig_p7 = (int16_t)BME280_CONCAT_BYTES(reg_data[19], reg_data[18]);
    calib_data->dig_p8 = (int16_t)BME280_CONCAT_BYTES(reg_data[21], reg_data[20]);
    calib_data->dig_p9 = (int16_t)BME280_CONCAT_BYTES(reg_data[23], reg_data[22]);
    calib_data->dig_h1 = reg_data[25];
}

static void parse_humidity_calib_data(const uint8_t *reg_data, bme280_dev_t *dev)
{
    bme280_calib_data_t *calib_data = &dev->calib_data;
    int16_t dig_h4_lsb, dig_h4_msb, dig_h5_lsb, dig_h5_msb;

    calib_data->dig_h2 = (int16_t)BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    calib_data->dig_h3 = reg_data[2];
    dig_h4_msb = (int16_t)(int8_t)reg_data[3] * 16;
    dig_h4_lsb = (int16_t)(reg_data[4] & 0x0F);
    calib_data->dig_h4 = dig_h4_msb | dig_h4_lsb;
    dig_h5_msb = (int16_t)(int8_t)reg_data[5] * 16;
    dig_h5_lsb = (int16_t)(reg_data[4] >> 4);
    calib_data->dig_h5 = dig_h5_msb | dig_h5_lsb;
    calib_data->dig_h6 = (int8_t)reg_data[6];
}

/* Data parsing and compensation functions */
static void parse_sensor_data(const uint8_t *reg_data, uint32_t *temp, uint32_t *press, uint32_t *hum)
{
    uint32_t data_xlsb, data_lsb, data_msb;

    /* Parse pressure data */
    data_msb = (uint32_t)reg_data[0] << BME280_12_BIT_SHIFT;
    data_lsb = (uint32_t)reg_data[1] << BME280_4_BIT_SHIFT;
    data_xlsb = (uint32_t)reg_data[2] >> BME280_4_BIT_SHIFT;
    *press = data_msb | data_lsb | data_xlsb;

    /* Parse temperature data */
    data_msb = (uint32_t)reg_data[3] << BME280_12_BIT_SHIFT;
    data_lsb = (uint32_t)reg_data[4] << BME280_4_BIT_SHIFT;
    data_xlsb = (uint32_t)reg_data[5] >> BME280_4_BIT_SHIFT;
    *temp = data_msb | data_lsb | data_xlsb;

    /* Parse humidity data */
    data_msb = (uint32_t)reg_data[6] << BME280_8_BIT_SHIFT;
    data_lsb = (uint32_t)reg_data[7];
    *hum = data_msb | data_lsb;
}

static double compensate_temperature(uint32_t uncomp_temperature, bme280_calib_data_t *calib_data)
{
    double var1, var2, temperature, temperature_min = -40, temperature_max = 85;

    var1 = (((double)uncomp_temperature) / 16384.0 - ((double)calib_data->dig_t1) / 1024.0);
    var1 = var1 * ((double)calib_data->dig_t2);
    var2 = (((double)uncomp_temperature) / 131072.0 - ((double)calib_data->dig_t1) / 8192.0);
    var2 = (var2 * var2) * ((double)calib_data->dig_t3);
    calib_data->t_fine = (int32_t)(var1 + var2);
    temperature = (var1 + var2) / 5120.0;

    if (temperature < temperature_min) temperature = temperature_min;
    else if (temperature > temperature_max) temperature = temperature_max;

    return temperature;
}

static double compensate_pressure(uint32_t uncomp_pressure, const bme280_calib_data_t *calib_data)
{
    double var1, var2, var3, pressure, pressure_min = 30000.0, pressure_max = 110000.0;

    var1 = ((double)calib_data->t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)calib_data->dig_p6) / 32768.0;
    var2 = var2 + var1 * ((double)calib_data->dig_p5) * 2.0;
    var2 = (var2 / 4.0) + (((double)calib_data->dig_p4) * 65536.0);
    var3 = ((double)calib_data->dig_p3) * var1 * var1 / 524288.0;
    var1 = (var3 + ((double)calib_data->dig_p2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)calib_data->dig_p1);

    if (var1 > (0.0)) {
        pressure = 1048576.0 - (double)uncomp_pressure;
        pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((double)calib_data->dig_p9) * pressure * pressure / 2147483648.0;
        var2 = pressure * ((double)calib_data->dig_p8) / 32768.0;
        pressure = pressure + (var1 + var2 + ((double)calib_data->dig_p7)) / 16.0;

        if (pressure < pressure_min) pressure = pressure_min;
        else if (pressure > pressure_max) pressure = pressure_max;
    } else {
        pressure = pressure_min;
    }
    return pressure;
}

static double compensate_humidity(uint32_t uncomp_humidity, const bme280_calib_data_t *calib_data)
{
    double humidity, humidity_min = 0.0, humidity_max = 100.0;
    double var1, var2, var3, var4, var5, var6;

    var1 = ((double)calib_data->t_fine) - 76800.0;
    var2 = (((double)calib_data->dig_h4) * 64.0 + (((double)calib_data->dig_h5) / 16384.0) * var1);
    var3 = uncomp_humidity - var2;
    var4 = ((double)calib_data->dig_h2) / 65536.0;
    var5 = (1.0 + (((double)calib_data->dig_h3) / 67108864.0) * var1);
    var6 = 1.0 + (((double)calib_data->dig_h6) / 67108864.0) * var1 * var5;
    var6 = var3 * var4 * (var5 * var6);
    humidity = var6 * (1.0 - ((double)calib_data->dig_h1) * var6 / 524288.0);

    if (humidity > humidity_max) humidity = humidity_max;
    else if (humidity < humidity_min) humidity = humidity_min;

    return humidity;
}

/* Public API functions */
int8_t BME280_Setup(bme280_dev_t* dev, I2C_HandleTypeDef* hi2c)
{
    int8_t rslt;
    uint8_t chip_id = 0;
    uint8_t settings_sel;

    /* Store I2C handle globally */
    g_hi2c = hi2c;

    /* Check if device is ready */
    if (HAL_I2C_IsDeviceReady(hi2c, BME280_I2C_ADDR << 1, 3, 1000) != HAL_OK) {
        return BME280_E_COMM_FAIL;
    }

    /* Initialize device structure */
    dev->intf = BME280_I2C_INTF;
    dev->read = bme280_i2c_read;
    dev->write = bme280_i2c_write;
    dev->delay_us = bme280_delay_us;
    dev->intf_ptr = NULL;
    dev->intf_rslt = BME280_INTF_RET_SUCCESS;

    /* Read chip ID */
    rslt = bme280_get_regs(BME280_REG_CHIP_ID, &chip_id, 1, dev);
    if (rslt != BME280_OK) return rslt;

    /* Verify chip ID */
    if (chip_id != BME280_CHIP_ID) {
        return BME280_E_DEV_NOT_FOUND;
    }
    dev->chip_id = chip_id;

    /* Soft reset */
    rslt = bme280_soft_reset(dev);
    if (rslt != BME280_OK) return rslt;

    /* Get calibration data */
    rslt = get_calib_data(dev);
    if (rslt != BME280_OK) return rslt;

    /* Configure sensor settings */
    uint8_t ctrl_hum = BME280_OVERSAMPLING_1X & 0x07;
    uint8_t ctrl_meas, config;

    /* Set humidity oversampling */
    rslt = bme280_set_regs(BME280_REG_CTRL_HUM, &ctrl_hum, 1, dev);
    if (rslt != BME280_OK) return rslt;

    /* Set pressure and temperature oversampling + power mode */
    ctrl_meas = (BME280_OVERSAMPLING_2X << 5) |  /* Temperature oversampling */
                (BME280_OVERSAMPLING_16X << 2) |  /* Pressure oversampling */
                BME280_POWERMODE_NORMAL;           /* Normal mode */
    rslt = bme280_set_regs(BME280_REG_CTRL_MEAS, &ctrl_meas, 1, dev);
    if (rslt != BME280_OK) return rslt;

    /* Set filter and standby time */
    config = (BME280_STANDBY_TIME_62_5_MS << 5) |  /* Standby time */
             (BME280_FILTER_COEFF_16 << 2);         /* Filter coefficient */
    rslt = bme280_set_regs(BME280_REG_CONFIG, &config, 1, dev);

    return rslt;
}

int8_t BME280_GetSensorData(bme280_dev_t* dev, bme280_data_t* comp_data)
{
    int8_t rslt;
    uint8_t reg_data[BME280_LEN_P_T_H_DATA] = {0};
    uint32_t uncomp_press, uncomp_temp, uncomp_hum;

    /* Read sensor data registers */
    rslt = bme280_get_regs(BME280_REG_DATA, reg_data, BME280_LEN_P_T_H_DATA, dev);
    if (rslt != BME280_OK) return rslt;

    /* Parse the read data */
    parse_sensor_data(reg_data, &uncomp_temp, &uncomp_press, &uncomp_hum);

    /* Compensate the data */
    comp_data->temperature = compensate_temperature(uncomp_temp, &dev->calib_data);
    comp_data->pressure = compensate_pressure(uncomp_press, &dev->calib_data);
    comp_data->humidity = compensate_humidity(uncomp_hum, &dev->calib_data);

    return BME280_OK;
}
