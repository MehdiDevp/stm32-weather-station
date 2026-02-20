/* bh1750.c */
#include "bh1750.h"

HAL_StatusTypeDef BH1750_Init(I2C_HandleTypeDef* hi2c)
{
  HAL_StatusTypeDef status;
  uint8_t cmd;

  // Check if device is ready
  status = HAL_I2C_IsDeviceReady(hi2c, BH1750_ADDR << 1, 3, 1000);
  if (status != HAL_OK) {
    return status;
  }

  // Power on the sensor
  cmd = BH1750_POWER_ON;
  status = HAL_I2C_Master_Transmit(hi2c, BH1750_ADDR << 1, &cmd, 1, 1000);
  if (status != HAL_OK) {
    return status;
  }

  HAL_Delay(10);

  // Reset the sensor
  cmd = BH1750_RESET;
  status = HAL_I2C_Master_Transmit(hi2c, BH1750_ADDR << 1, &cmd, 1, 1000);
  if (status != HAL_OK) {
    return status;
  }

  HAL_Delay(10);

  // Start continuous high resolution measurement
  cmd = BH1750_CONT_H_RES;
  status = HAL_I2C_Master_Transmit(hi2c, BH1750_ADDR << 1, &cmd, 1, 1000);

  // Wait for first measurement to be ready
  HAL_Delay(180);

  return status;
}

HAL_StatusTypeDef BH1750_ReadLight(I2C_HandleTypeDef* hi2c, float *lux)
{
  HAL_StatusTypeDef status;
  uint8_t data[2];
  uint16_t raw_data;

  // Read 2 bytes of light data
  status = HAL_I2C_Master_Receive(hi2c, BH1750_ADDR << 1, data, 2, 1000);

  if (status == HAL_OK) {
    // Combine the two bytes and convert to lux
    raw_data = (data[0] << 8) | data[1];
    *lux = (float)raw_data / 1.2f;
  } else {
    *lux = 0.0f; 
  }

  return status;
}
