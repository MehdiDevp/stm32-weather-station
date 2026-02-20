/* power_monitor.h */
#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

#include "main.h"
#include <stdint.h>

/* INA219 I2C Address and Registers */
#define INA219_ADDR        0x40  /* INA219 I2C address */
#define INA219_CONFIG      0x00  /* Configuration Register */
#define INA219_SHUNT_V     0x01  /* Shunt Voltage Register */
#define INA219_BUS_V       0x02  /* Bus Voltage Register */
#define INA219_POWER       0x03  /* Power Register */
#define INA219_CURRENT     0x04  /* Current Register */
#define INA219_CAL         0x05  /* Calibration Register */

/* INA219 Configuration */
#define INA219_CONFIG_RESET         0x8000  /* Reset bit */
#define INA219_CONFIG_16V_RANGE     0x2000  /* 16V FSR */
#define INA219_CONFIG_GAIN_320MV    0x1800  /* 320mV shunt voltage range */
#define INA219_CONFIG_12BIT_ADC     0x0400  /* 12-bit ADC resolution */
#define INA219_CONFIG_CONT_MODE     0x0007  /* Continuous measurement mode */

/* Battery Configuration for 3.2V LiPo 6000mAh */
#define BATTERY_TYPE_LIPO_32V  1  /* 3.2V LiFePO4/LiPo battery */
#define BATTERY_TYPE           BATTERY_TYPE_LIPO_32V
#define BATTERY_CAPACITY_MAH   6000  /* 6000mAh capacity */

/* Battery discharge curve structure */
typedef struct {
    float voltage;
    float soc;
} battery_curve_point_t;

/* Power monitoring data structure */
typedef struct {
    float voltage;              /* Battery voltage in V */
    float current;              /* Battery current in mA */
    float power;                /* Battery power in mW */
    float soc_percent;          /* State of Charge in % */
    int32_t voltage_x100;       /* Battery voltage as int32_t * 100 */
    int32_t current_x100;       /* Battery current as int32_t * 100 */
    int32_t power_x100;         /* Battery power as int32_t * 100 */
    int32_t soc_percent_x100;   /* SOC as int32_t * 100 */
} PowerData_t;

/* Function prototypes */
HAL_StatusTypeDef PowerMonitor_Init(I2C_HandleTypeDef* hi2c);
HAL_StatusTypeDef PowerMonitor_ReadPower(I2C_HandleTypeDef* hi2c, PowerData_t* power_data);
void PowerMonitor_CalculateSOC(PowerData_t* power_data);
void PowerMonitor_ResetSOCToFull(void);
uint32_t PowerMonitor_GetEstimatedRuntimeMinutes(const PowerData_t* power_data);

/* Internal function prototypes */
float PowerMonitor_EstimateSOCFromVoltage(float voltage);
void PowerMonitor_UpdateCoulombCounting(float current_ma, uint32_t delta_time_ms);
float PowerMonitor_CalculateSOCCoulombCounting(void);
float PowerMonitor_CalculateSOCCombined(float voltage, float current_ma, uint32_t delta_time_ms);

#endif /* POWER_MONITOR_H */
