/* sen5x_wrapper.c */
#include "sen5x_wrapper.h"

int16_t SEN5x_Setup(I2C_HandleTypeDef* hi2c)
{
  int16_t error;

  // Check if SEN5x is ready
  if (HAL_I2C_IsDeviceReady(hi2c, 0x69 << 1, 3, 1000) != HAL_OK) {
    return -1;
  }

  // Initialize Sensirion I2C HAL
  sensirion_i2c_hal_init();
  HAL_Delay(500);

  // Stop any previous measurement
  error = sen5x_stop_measurement();
  if (error) {
    return error;
  }

  HAL_Delay(200);

  // Start measurement
  error = sen5x_start_measurement();
  if (error) {
    return error;
  }

  // Wait for sensor to stabilize
  HAL_Delay(1000);

  return 0;
}

int16_t SEN5x_ReadAirQuality(I2C_HandleTypeDef* hi2c, AirQualityData_t* air_data)
{
  int16_t error;
  bool data_ready = false;
  uint16_t mass_concentration_pm1p0;
  uint16_t mass_concentration_pm2p5;
  uint16_t mass_concentration_pm4p0;
  uint16_t mass_concentration_pm10p0;
  int16_t ambient_humidity;
  int16_t ambient_temperature;
  int16_t voc_index_raw;
  int16_t nox_index_raw;

  // Check if data is ready
  error = sen5x_read_data_ready(&data_ready);
  if (error || !data_ready) {
    return error ? error : -1;
  }

  // Read the measured values
  error = sen5x_read_measured_values(&mass_concentration_pm1p0,
                                     &mass_concentration_pm2p5,
                                     &mass_concentration_pm4p0,
                                     &mass_concentration_pm10p0,
                                     &ambient_humidity,
                                     &ambient_temperature,
                                     &voc_index_raw,
                                     &nox_index_raw);

  if (error == 0) {
    // Convert raw values to float (scale factor is 10)
    air_data->pm1p0_ugm3 = (float)mass_concentration_pm1p0 / 10.0f;
    air_data->pm2p5_ugm3 = (float)mass_concentration_pm2p5 / 10.0f;
    air_data->pm4p0_ugm3 = (float)mass_concentration_pm4p0 / 10.0f;
    air_data->pm10p0_ugm3 = (float)mass_concentration_pm10p0 / 10.0f;

    air_data->temperature_index = (float)ambient_temperature / 200.0f;  // Scale factor is 200 for temperature
    air_data->humidity_index = (float)ambient_humidity / 100.0f;  // Scale factor is 100 for humidity

    air_data->voc_index = (float)voc_index_raw / 10.0f;
    air_data->nox_index = (float)nox_index_raw / 10.0f;

    // Store as integers
    air_data->pm1p0_ugm3_x100 = (int32_t)air_data->pm1p0_ugm3;
    air_data->pm2p5_ugm3_x100 = (int32_t)air_data->pm2p5_ugm3;
    air_data->pm4p0_ugm3_x100 = (int32_t)air_data->pm4p0_ugm3;
    air_data->pm10p0_ugm3_x100 = (int32_t)air_data->pm10p0_ugm3;
    air_data->voc_index_x100 = (int32_t)air_data->voc_index;
    air_data->nox_index_x100 = (int32_t)air_data->nox_index;
    air_data->temperature_index_x100 = (int32_t)air_data->temperature_index;
    air_data->humidity_index_x100 = (int32_t)air_data->humidity_index;

    // Handle invalid readings (sensor returns 0xFFFF or 0x7FFF for invalid data)
    if (mass_concentration_pm1p0 == 0xFFFF) {
      air_data->pm1p0_ugm3 = 0.0f;
      air_data->pm1p0_ugm3_x100 = 0;
    }
    if (mass_concentration_pm2p5 == 0xFFFF) {
      air_data->pm2p5_ugm3 = 0.0f;
      air_data->pm2p5_ugm3_x100 = 0;
    }
    if (mass_concentration_pm4p0 == 0xFFFF) {
      air_data->pm4p0_ugm3 = 0.0f;
      air_data->pm4p0_ugm3_x100 = 0;
    }
    if (mass_concentration_pm10p0 == 0xFFFF) {
      air_data->pm10p0_ugm3 = 0.0f;
      air_data->pm10p0_ugm3_x100 = 0;
    }
    if (voc_index_raw == 0x7FFF) {
      air_data->voc_index = 0.0f;
      air_data->voc_index_x100 = 0;
    }
    if (nox_index_raw == 0x7FFF) {
      air_data->nox_index = 0.0f;
      air_data->nox_index_x100 = 0;
    }
    if (ambient_temperature == 0x7FFF) {
      air_data->temperature_index = 0.0f;
      air_data->temperature_index_x100 = 0;
    }
    if (ambient_humidity == 0x7FFF) {
      air_data->humidity_index = 0.0f;
      air_data->humidity_index_x100 = 0;
    }
  }

  return error;
}
