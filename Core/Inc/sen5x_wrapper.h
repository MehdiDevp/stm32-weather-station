/* sen5x_wrapper.h */
#ifndef SEN5X_WRAPPER_H
#define SEN5X_WRAPPER_H

#include "main.h"
#include "sen5x_i2c.h"
#include "sensirion_i2c.h"
#include "sensirion_i2c_hal.h"
#include <stdint.h>

/* SEN5x air quality data structure */
typedef struct {
    float pm1p0_ugm3;          /* PM1.0 in µg/m³ */
    float pm2p5_ugm3;          /* PM2.5 in µg/m³ */
    float pm4p0_ugm3;          /* PM4.0 in µg/m³ */
    float pm10p0_ugm3;         /* PM10.0 in µg/m³ */
    float voc_index;           /* VOC Index */
    float nox_index;           /* NOx Index */
    float temperature_index;   /* Temperature from SEN5x */
    float humidity_index;      /* Humidity from SEN5x */

    /* Integer values (not multiplied by 100) */
    int32_t pm1p0_ugm3_x100;      /* PM1.0 as int32_t */
    int32_t pm2p5_ugm3_x100;      /* PM2.5 as int32_t */
    int32_t pm4p0_ugm3_x100;      /* PM4.0 as int32_t */
    int32_t pm10p0_ugm3_x100;     /* PM10.0 as int32_t */
    int32_t voc_index_x100;       /* VOC Index as int32_t */
    int32_t nox_index_x100;       /* NOx Index as int32_t */
    int32_t temperature_index_x100; /* Temperature Index as int32_t */
    int32_t humidity_index_x100;    /* Humidity Index as int32_t */
} AirQualityData_t;

/* Function prototypes */
int16_t SEN5x_Setup(I2C_HandleTypeDef* hi2c);
int16_t SEN5x_ReadAirQuality(I2C_HandleTypeDef* hi2c, AirQualityData_t* air_data);

#endif /* SEN5X_WRAPPER_H */
