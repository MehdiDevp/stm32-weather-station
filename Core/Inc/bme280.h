/* bme280_simple.h - Simplified BME280 driver with only used functions */
#ifndef BME280_SIMPLE_H
#define BME280_SIMPLE_H

#include "main.h"
#include <stdint.h>

/* BME280 I2C Address */
#define BME280_I2C_ADDR                 0x76

/* BME280 Registers */
#define BME280_REG_CHIP_ID              0xD0
#define BME280_REG_RESET                0xE0
#define BME280_REG_TEMP_PRESS_CALIB_DATA 0x88
#define BME280_REG_HUMIDITY_CALIB_DATA  0xE1
#define BME280_REG_CTRL_HUM             0xF2
#define BME280_REG_STATUS               0xF3
#define BME280_REG_CTRL_MEAS            0xF4
#define BME280_REG_CONFIG               0xF5
#define BME280_REG_DATA                 0xF7

/* BME280 Constants */
#define BME280_CHIP_ID                  0x60
#define BME280_SOFT_RESET_COMMAND       0xB6
#define BME280_STARTUP_DELAY            2000

/* BME280 Interface */
#define BME280_I2C_INTF                 0

/* BME280 Return codes */
#define BME280_OK                       0
#define BME280_E_NULL_PTR              -1
#define BME280_E_COMM_FAIL             -2
#define BME280_E_DEV_NOT_FOUND         -3
#define BME280_E_INVALID_LEN           -4
#define BME280_E_NVM_COPY_FAILED       -5

/* BME280 Power modes */
#define BME280_POWERMODE_SLEEP          0x00
#define BME280_POWERMODE_FORCED         0x01
#define BME280_POWERMODE_NORMAL         0x03

/* BME280 Oversampling settings */
#define BME280_OVERSAMPLING_NONE        0x00
#define BME280_OVERSAMPLING_1X          0x01
#define BME280_OVERSAMPLING_2X          0x02
#define BME280_OVERSAMPLING_4X          0x03
#define BME280_OVERSAMPLING_8X          0x04
#define BME280_OVERSAMPLING_16X         0x05

/* BME280 Filter settings */
#define BME280_FILTER_COEFF_OFF         0x00
#define BME280_FILTER_COEFF_2           0x01
#define BME280_FILTER_COEFF_4           0x02
#define BME280_FILTER_COEFF_8           0x03
#define BME280_FILTER_COEFF_16          0x04

/* BME280 Standby time settings */
#define BME280_STANDBY_TIME_0_5_MS      0x00
#define BME280_STANDBY_TIME_62_5_MS     0x01
#define BME280_STANDBY_TIME_125_MS      0x02
#define BME280_STANDBY_TIME_250_MS      0x03
#define BME280_STANDBY_TIME_500_MS      0x04
#define BME280_STANDBY_TIME_1000_MS     0x05
#define BME280_STANDBY_TIME_10_MS       0x06
#define BME280_STANDBY_TIME_20_MS       0x07

/* BME280 Settings selection */
#define BME280_SEL_OSR_PRESS            0x01
#define BME280_SEL_OSR_TEMP             0x02
#define BME280_SEL_OSR_HUM              0x04
#define BME280_SEL_FILTER               0x08
#define BME280_SEL_STANDBY              0x10
#define BME280_SEL_ALL_SETTINGS         0x1F

/* BME280 Sensor component selection */
#define BME280_PRESS                    0x01
#define BME280_TEMP                     0x02
#define BME280_HUM                      0x04
#define BME280_ALL                      0x07

/* BME280 Data lengths */
#define BME280_LEN_TEMP_PRESS_CALIB_DATA 26
#define BME280_LEN_HUMIDITY_CALIB_DATA   7
#define BME280_LEN_P_T_H_DATA            8

/* Interface return type */
#define BME280_INTF_RET_SUCCESS         0

/* Calibration data structure */
typedef struct {
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
    uint8_t dig_h1;
    int16_t dig_h2;
    uint8_t dig_h3;
    int16_t dig_h4;
    int16_t dig_h5;
    int8_t dig_h6;
    int32_t t_fine;
} bme280_calib_data_t;

/* Settings structure */
typedef struct {
    uint8_t osr_p;
    uint8_t osr_t;
    uint8_t osr_h;
    uint8_t filter;
    uint8_t standby_time;
} bme280_settings_t;

/* Sensor data structure */
typedef struct {
    double temperature;
    double pressure;
    double humidity;
} bme280_data_t;

/* Device structure */
typedef struct {
    uint8_t chip_id;
    uint8_t intf;
    int8_t (*read)(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
    int8_t (*write)(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
    void (*delay_us)(uint32_t period, void *intf_ptr);
    void *intf_ptr;
    int8_t intf_rslt;
    bme280_calib_data_t calib_data;
} bme280_dev_t;


int8_t BME280_Setup(bme280_dev_t* dev, I2C_HandleTypeDef* hi2c);
int8_t BME280_GetSensorData(bme280_dev_t* dev, bme280_data_t* comp_data);

#endif /* BME280_SIMPLE_H */
