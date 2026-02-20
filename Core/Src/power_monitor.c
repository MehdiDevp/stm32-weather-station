/* power_monitor.c */
#include "power_monitor.h"
#include <stdlib.h>

/* Static variables for SOC calculation */
static float accumulated_charge_mah = 0.0f;           /* Tracked charge in mAh */
static uint32_t last_measurement_time = 0;            /* For time difference calculation */
static uint8_t soc_initialized = 0;                  /* SOC initialization flag */

static float current_soc_percent = 100.0f;      /* Current SOC in percent */
static uint32_t last_soc_time = 0;              /* Last calculation time */
static uint8_t soc_ready = 0;                   /* SOC system ready flag */

/* 3.2V LiPo discharge curve (LiFePO4 chemistry) */
static const battery_curve_point_t lipo_32v_discharge_curve[] = {
		{3.65f, 100.0f}, /* Full charge for 3.2V LiPo */
		{3.60f, 95.0f},
		{3.55f, 90.0f},
		{3.50f, 85.0f},
		{3.45f, 80.0f},
		{3.40f, 75.0f},
		{3.35f, 70.0f},
		{3.32f, 65.0f},
		{3.30f, 60.0f},
		{3.28f, 55.0f},
		{3.26f, 50.0f},
		{3.24f, 45.0f},
		{3.22f, 40.0f},
		{3.20f, 35.0f},
		{3.18f, 30.0f},
		{3.16f, 25.0f},
		{3.14f, 20.0f},
		{3.12f, 15.0f},
		{3.10f, 10.0f},
		{3.05f, 5.0f},
		{2.80f, 0.0f} /* Empty */
		};

#define LIPO_32V_CURVE_POINTS (sizeof(lipo_32v_discharge_curve) / sizeof(battery_curve_point_t))

HAL_StatusTypeDef PowerMonitor_Init(I2C_HandleTypeDef* hi2c)
{
  HAL_StatusTypeDef status;
  uint8_t config_data[2];

  // Check if INA219 is ready
  status = HAL_I2C_IsDeviceReady(hi2c, INA219_ADDR << 1, 3, 500);
  if (status != HAL_OK) {
    return status;
  }

  // Configure INA219
  uint16_t config = INA219_CONFIG_16V_RANGE | INA219_CONFIG_GAIN_320MV |
                    INA219_CONFIG_12BIT_ADC | INA219_CONFIG_CONT_MODE;

  config_data[0] = (config >> 8) & 0xFF;
  config_data[1] = config & 0xFF;

  status = HAL_I2C_Mem_Write(hi2c, INA219_ADDR << 1, INA219_CONFIG,
                             I2C_MEMADD_SIZE_8BIT, config_data, 2, 1000);
  if (status != HAL_OK) {
    return status;
  }

  // Set calibration value
  uint16_t calibration = 4096;
  config_data[0] = (calibration >> 8) & 0xFF;
  config_data[1] = calibration & 0xFF;

  status = HAL_I2C_Mem_Write(hi2c, INA219_ADDR << 1, INA219_CAL,
                             I2C_MEMADD_SIZE_8BIT, config_data, 2, 1000);

  HAL_Delay(10);
  return status;
}

HAL_StatusTypeDef PowerMonitor_ReadPower(I2C_HandleTypeDef* hi2c, PowerData_t* power_data)
{
  HAL_StatusTypeDef status;
  uint8_t data[2];
  uint16_t raw_voltage, raw_current, raw_power;

  /* Read bus voltage */
  status = HAL_I2C_Mem_Read(hi2c, INA219_ADDR << 1, INA219_BUS_V,
                            I2C_MEMADD_SIZE_8BIT, data, 2, 1000);
  if (status == HAL_OK) {
    raw_voltage = (data[0] << 8) | data[1];
    power_data->voltage = (float)(raw_voltage >> 3) * 0.004f;
    power_data->voltage_x100 = (int32_t)(power_data->voltage * 100);
  } else {
    power_data->voltage = 0.0f;  // Changed from -999.0f
    power_data->voltage_x100 = 0;  // Changed from -999
    return status;
  }

  /* Read current */
  status = HAL_I2C_Mem_Read(hi2c, INA219_ADDR << 1, INA219_CURRENT,
                            I2C_MEMADD_SIZE_8BIT, data, 2, 1000);
  if (status == HAL_OK) {
    raw_current = (data[0] << 8) | data[1];
    int16_t signed_current = (int16_t)raw_current;
    power_data->current = (float)signed_current;
    power_data->current_x100 = (int32_t)power_data->current;
  } else {
    power_data->current = 0.0f;  // Changed from -999.0f
    power_data->current_x100 = 0;  // Changed from -999
  }

  /* Read power */
  status = HAL_I2C_Mem_Read(hi2c, INA219_ADDR << 1, INA219_POWER,
                            I2C_MEMADD_SIZE_8BIT, data, 2, 1000);
  if (status == HAL_OK) {
    raw_power = (data[0] << 8) | data[1];
    power_data->power = (float)raw_power * 20.0f;
    power_data->power_x100 = (int32_t)power_data->power;
  } else {
    power_data->power = 0.0f;  // Changed from -999.0f
    power_data->power_x100 = 0;  // Changed from -999
  }

  return status;
}

float PowerMonitor_EstimateSOCFromVoltage(float voltage)
{
  /* Handle edge cases */
  if (voltage >= lipo_32v_discharge_curve[0].voltage) {
    return 100.0f;
  }
  if (voltage <= lipo_32v_discharge_curve[LIPO_32V_CURVE_POINTS - 1].voltage) {
    return 0.0f;
  }

  /* Linear interpolation between curve points */
  for (uint8_t i = 0; i < LIPO_32V_CURVE_POINTS - 1; i++) {
    if (voltage <= lipo_32v_discharge_curve[i].voltage && voltage >= lipo_32v_discharge_curve[i + 1].voltage) {
      float voltage_range = lipo_32v_discharge_curve[i].voltage - lipo_32v_discharge_curve[i + 1].voltage;
      float soc_range = lipo_32v_discharge_curve[i].soc - lipo_32v_discharge_curve[i + 1].soc;
      float voltage_offset = lipo_32v_discharge_curve[i].voltage - voltage;

      float interpolated_soc = lipo_32v_discharge_curve[i].soc - (voltage_offset / voltage_range) * soc_range;
      return interpolated_soc;
    }
  }

  return 50.0f;  /* Default fallback */
}

void PowerMonitor_UpdateCoulombCounting(float current_ma, uint32_t delta_time_ms)
{
  /* Convert time to hours */
  float delta_time_hours = (float)delta_time_ms / (1000.0f * 3600.0f);

  /* Calculate charge change in mAh */
  float charge_change_mah = current_ma * delta_time_hours;

  /* Update accumulated charge (positive current = discharge) */
  accumulated_charge_mah += charge_change_mah;

  /* Clamp to battery capacity limits */
  if (accumulated_charge_mah > BATTERY_CAPACITY_MAH) {
    accumulated_charge_mah = BATTERY_CAPACITY_MAH;
  }
  if (accumulated_charge_mah < 0.0f) {
    accumulated_charge_mah = 0.0f;
  }
}

float PowerMonitor_CalculateSOCCoulombCounting(void)
{
  float remaining_capacity = BATTERY_CAPACITY_MAH - accumulated_charge_mah;
  float soc = (remaining_capacity / BATTERY_CAPACITY_MAH) * 100.0f;

  /* Clamp to valid range */
  if (soc > 100.0f) soc = 100.0f;
  if (soc < 0.0f) soc = 0.0f;

  return soc;
}

float PowerMonitor_CalculateSOCCombined(float voltage, float current_ma, uint32_t delta_time_ms)
{
  static float voltage_soc_weight = 0.3f;    /* 30% voltage, 70% coulomb counting */
  static float coulomb_soc_weight = 0.7f;

  /* Get voltage-based SOC */
  float voltage_soc = PowerMonitor_EstimateSOCFromVoltage(voltage);

  /* Update coulomb counting */
  if (delta_time_ms > 0) {
    PowerMonitor_UpdateCoulombCounting(current_ma, delta_time_ms);
  }

  /* Get coulomb counting SOC */
  float coulomb_soc = PowerMonitor_CalculateSOCCoulombCounting();

  /* If this is the first measurement, initialize coulomb counting from voltage */
  if (!soc_initialized) {
    /* Set initial accumulated charge based on voltage SOC */
    accumulated_charge_mah = BATTERY_CAPACITY_MAH * (1.0f - voltage_soc / 100.0f);
    soc_initialized = 1;
    return voltage_soc;
  }

  /* Weighted combination */
  float combined_soc = (voltage_soc * voltage_soc_weight) + (coulomb_soc * coulomb_soc_weight);

  /* Drift correction: if voltage indicates very low/high, adjust coulomb counting */
  if (voltage_soc < 10.0f && coulomb_soc > 20.0f) {
    /* Voltage says battery is very low, trust voltage more */
    voltage_soc_weight = 0.8f;
    coulomb_soc_weight = 0.2f;
    combined_soc = (voltage_soc * voltage_soc_weight) + (coulomb_soc * coulomb_soc_weight);
  } else if (voltage_soc > 95.0f && coulomb_soc < 80.0f) {
    /* Voltage says battery is full, trust voltage more */
    voltage_soc_weight = 0.8f;
    coulomb_soc_weight = 0.2f;
    combined_soc = (voltage_soc * voltage_soc_weight) + (coulomb_soc * coulomb_soc_weight);
  } else {
    /* Normal operation, reset weights */
    voltage_soc_weight = 0.3f;
    coulomb_soc_weight = 0.7f;
  }

  return combined_soc;
}

void PowerMonitor_CalculateSOC(PowerData_t* power_data)
{
    uint32_t current_time = HAL_GetTick();

    /* Initialize system on first call */
    if (!soc_ready) {
        last_soc_time = current_time;

        /* SIMPLE CONSERVATIVE APPROACH */
        float voltage_estimate = PowerMonitor_EstimateSOCFromVoltage(power_data->voltage);

        /* Conservative adjustments */
        if (voltage_estimate > 90.0f) {
            current_soc_percent = 85.0f;  /* Don't assume nearly full */
        } else if (voltage_estimate > 70.0f) {
            current_soc_percent = voltage_estimate - 10.0f;  /* Subtract 10% safety margin */
        } else if (voltage_estimate > 30.0f) {
            current_soc_percent = voltage_estimate - 5.0f;   /* Subtract 5% safety margin */
        } else {
            current_soc_percent = voltage_estimate;  /* Trust low voltage readings */
        }

        /* Ensure valid range */
        if (current_soc_percent > 100.0f) current_soc_percent = 100.0f;
        if (current_soc_percent < 0.0f) current_soc_percent = 0.0f;

        soc_ready = 1;
        power_data->soc_percent = current_soc_percent;
        power_data->soc_percent_x100 = (int32_t)(current_soc_percent * 100.0f);
        return;
    }


    uint32_t time_delta_ms = current_time - last_soc_time;
    last_soc_time = current_time;

    if (time_delta_ms > 0) {
        float time_delta_hours = (float)time_delta_ms / 3600000.0f;
        float charge_delta_mah = power_data->current * time_delta_hours;
        float soc_delta_percent = (charge_delta_mah / BATTERY_CAPACITY_MAH) * 100.0f;

        current_soc_percent -= soc_delta_percent;

        if (current_soc_percent > 100.0f) current_soc_percent = 100.0f;
        if (current_soc_percent < 0.0f) current_soc_percent = 0.0f;
    }

    power_data->soc_percent = current_soc_percent;
    power_data->soc_percent_x100 = (int32_t)(current_soc_percent * 100.0f);

    if (power_data->voltage <= 0.0f) {
        power_data->soc_percent = 0.0f;
        power_data->soc_percent_x100 = 0;
    }
}


void PowerMonitor_ResetSOCToFull(void)
{
    current_soc_percent = 100.0f;
    soc_ready = 1;
	accumulated_charge_mah = 0.0f;
	soc_initialized = 1;
}

uint32_t PowerMonitor_GetEstimatedRuntimeMinutes(const PowerData_t* power_data)
{
  if (power_data->current <= 0.0f || power_data->soc_percent <= 0.0f) {
    return 0;
  }

  float remaining_mah = BATTERY_CAPACITY_MAH * (power_data->soc_percent / 100.0f);
  float runtime_hours = remaining_mah / power_data->current;

  return (uint32_t)(runtime_hours * 60.0f);
}
