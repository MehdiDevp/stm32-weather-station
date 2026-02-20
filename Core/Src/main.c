/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body with BME280, BH1750, SEN5x, INA219 and ESP8266 Library
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bme280.h"   /* Simplified BME280 sensor library */
#include "bh1750.h"          /* BH1750 light sensor library */
#include "sen5x_wrapper.h"   /* SEN5x sensor wrapper */
#include "power_monitor.h"   /* INA219 and battery monitoring */
#include "esp8266.h"         /* ESP8266 Library */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Debug disabled for faster startup */
#define DEBUG_ENABLED      0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
bme280_dev_t bme280;
bme280_data_t comp_data;

/* Variables to store sensor values */
float temperature_celsius = 0.0f;  /* Temperature in degrees Celsius */
float humidity_percent = 0.0f;     /* Humidity in percentage (0-100%) */
float ambient_light_lux = 0.0f;    /* Ambient light in lux */
float pressure_hpa = 0.0f;         /* Pressure in hPa */

/* Integer values (not multiplied by 100) */
int32_t temperature_celsius_x100 = 0;  /* Temperature as int32_t */
int32_t humidity_percent_x100 = 0;     /* Humidity as int32_t */
int32_t ambient_light_lux_x100 = 0;    /* Light as int32_t */
int32_t pressure_hpa_x100 = 0;         /* Pressure as int32_t */

/* Data structures for modular sensors */
AirQualityData_t air_quality_data;
PowerData_t power_data;
ESP8266_t esp8266;
SensorData_t sensor_data_for_transmission;

/* Sensor control variables */
uint8_t data_transmission_enabled = 0;
uint32_t server_timestamp = 0;

/* Sensor Status Variables */
uint8_t bme280_initialized = 0;
uint8_t bh1750_initialized = 0;
uint8_t sen5x_initialized = 0;
uint8_t ina219_initialized = 0;

/* Debug counters */
uint32_t bme280_read_errors = 0;
uint32_t bh1750_read_errors = 0;
uint32_t sen5x_read_errors = 0;
uint32_t http_transmission_errors = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */
// BME280 setup function - no longer needed as individual functions
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

  /* Small delay to ensure peripherals are ready */
  HAL_Delay(100);

  /* LED ON to show system start */
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

  /* Initialize BME280 sensor first */
  int8_t bme_rslt = BME280_Setup(&bme280, &hi2c1);
  if (bme_rslt == BME280_OK) {
    bme280_initialized = 1;
  }

  HAL_Delay(100);

  /* Initialize BH1750 sensor */
  HAL_StatusTypeDef bh_rslt = BH1750_Init(&hi2c1);
  if (bh_rslt == HAL_OK) {
    bh1750_initialized = 1;
  }

  HAL_Delay(100);

  /* Initialize SEN5x air quality sensor */
  int16_t sen5x_rslt = SEN5x_Setup(&hi2c1);
  if (sen5x_rslt == 0) {
    sen5x_initialized = 1;
  }

  HAL_Delay(100);

  /* Initialize INA219 power monitor */
  HAL_StatusTypeDef ina_rslt = PowerMonitor_Init(&hi2c1);
  if (ina_rslt == HAL_OK) {
    ina219_initialized = 1;
  }

  HAL_Delay(100);

  /* Initialize ESP8266 WiFi module using library */
  ESP8266_Status_t esp_rslt = ESP8266_Setup(&esp8266, &huart5);
  if (esp_rslt == ESP8266_OK) {
    data_transmission_enabled = 1;
  }

  /* Final status: Long blink if basic sensors OK */
  if (bme280_initialized && bh1750_initialized) {
    HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Read BME280 sensor data */
    int8_t bme_rslt = -1;
    if (bme280_initialized) {
      bme_rslt = BME280_GetSensorData(&bme280, &comp_data);
    }

    /* Read BH1750 light sensor data */
    HAL_StatusTypeDef bh_rslt = HAL_ERROR;
    if (bh1750_initialized) {
      bh_rslt = BH1750_ReadLight(&hi2c1, &ambient_light_lux);
    }

    /* Read SEN5x air quality data */
    int16_t sen5x_rslt = -1;
    if (sen5x_initialized) {
      sen5x_rslt = SEN5x_ReadAirQuality(&hi2c1, &air_quality_data);
    }

    /* Read INA219 power data */
    HAL_StatusTypeDef ina_rslt = HAL_ERROR;
    if (ina219_initialized) {
      ina_rslt = PowerMonitor_ReadPower(&hi2c1, &power_data);
    }

    /* Process BME280 data */
    if (bme_rslt == BME280_OK) {
      temperature_celsius = comp_data.temperature;
      humidity_percent = comp_data.humidity;
      pressure_hpa = comp_data.pressure / 100.0f;

      temperature_celsius_x100 = (int32_t)comp_data.temperature;
      humidity_percent_x100 = (int32_t)comp_data.humidity;
      pressure_hpa_x100 = (int32_t)pressure_hpa;

      /* Ensure humidity is within valid range 0-100% */
      if (humidity_percent > 100.0f) {
        humidity_percent = 100.0f;
        humidity_percent_x100 = 100;
      } else if (humidity_percent < 0.0f) {
        humidity_percent = 0.0f;
        humidity_percent_x100 = 0;
      }
    } else {
      bme280_read_errors++;
      temperature_celsius = 0.0f;  
      humidity_percent = 0.0f;     
      pressure_hpa = 0.0f;         
      temperature_celsius_x100 = 0;  
      humidity_percent_x100 = 0;     
      pressure_hpa_x100 = 0;         
    }

    /* Process BH1750 data */
    if (bh_rslt == HAL_OK) {
      ambient_light_lux_x100 = (int32_t)ambient_light_lux;
    } else {
      bh1750_read_errors++;
      ambient_light_lux = 0.0f;   
      ambient_light_lux_x100 = 0; 
    }

    /* Process SEN5x data */
    if (sen5x_rslt != 0) {
      sen5x_read_errors++;
      // Error values are already set to 0 in the SEN5x wrapper
    }

    /* Process INA219 data and calculate SOC */
    if (ina_rslt != HAL_OK) {
      // Error values are already set to 0 in the power monitor module
    } else {
      /* Calculate SOC using combined approach */
      PowerMonitor_CalculateSOC(&power_data);
    }

    /* Prepare sensor data for transmission */
    sensor_data_for_transmission.temperature_celsius = temperature_celsius;
    sensor_data_for_transmission.humidity_percent = humidity_percent;
    sensor_data_for_transmission.ambient_light_lux = ambient_light_lux;
    sensor_data_for_transmission.pressure_hpa = pressure_hpa;
    sensor_data_for_transmission.pm2p5_ugm3 = air_quality_data.pm2p5_ugm3;
    sensor_data_for_transmission.pm10p0_ugm3 = air_quality_data.pm10p0_ugm3;
    sensor_data_for_transmission.battery_soc_percent = power_data.soc_percent;

    /* Toggle LED to show activity */
    HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);

    /* Send data via HTTP API if enabled and connected */
    if (data_transmission_enabled && esp8266.wifi_connected) {
      ESP8266_Status_t http_status = ESP8266_SendSensorDataHTTP(&esp8266, &huart5, &sensor_data_for_transmission);
      if (http_status != ESP8266_OK) {
        http_transmission_errors++;
      }
    }

    /* Wait 2 seconds before next reading */
    HAL_Delay(2000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20404768;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */

/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BOOST_EN_GPIO_Port, BOOST_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BOOST_EN_Pin */
  GPIO_InitStruct.Pin = BOOST_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOOST_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
