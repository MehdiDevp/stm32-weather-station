/* esp8266.h */
#ifndef ESP8266_H
#define ESP8266_H

#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// ESP8266 Configuration
#define ESP8266_TIMEOUT 5000
#define ESP8266_BUFFER_SIZE 1024

// WiFi and Server Configuration
#define WIFI_SSID          "celab"         /* Change to your WiFi network name */
#define WIFI_PASSWORD      "celabsecpass1532!"        /* Change to your WiFi password */
#define SERVER_HOST        "10.42.39.19"    /* Change to your server IP */
#define SERVER_PORT        "8000"            /* HTTP API port */

/* HTTP API Configuration */
#define USER_ID            "1"               /* Your user ID */
#define API_TIME_ENDPOINT  "/time"
#define API_DATA_ENDPOINT  "/data"

/* Value IDs for different sensors */
#define VAL_ID_TEMPERATURE 1  // temperature_air
#define VAL_ID_HUMIDITY    2  // humidity_air
#define VAL_ID_LIGHT       4  // illuminance
#define VAL_ID_PRESSURE    5  // pressure_air
#define VAL_ID_PM2P5       6  // particulates_2_5
#define VAL_ID_PM10P0      7  // particulates_10
#define VAL_ID_SOC         100 // Battery State of Charge

// ESP8266 Status
typedef enum {
    ESP8266_OK = 0,
    ESP8266_ERROR,
    ESP8266_TIMEOUT_ERROR,
    ESP8266_WIFI_NOT_CONNECTED,
    ESP8266_SERVER_ERROR
} ESP8266_Status_t;

// ESP8266 Structure
typedef struct {
    UART_HandleTypeDef* huart;
    char buffer[ESP8266_BUFFER_SIZE];
    uint8_t wifi_connected;
    char ssid[32];
    char password[64];
} ESP8266_t;

// Sensor data structure for HTTP transmission
typedef struct {
    float temperature_celsius;
    float humidity_percent;
    float ambient_light_lux;
    float pressure_hpa;
    float pm2p5_ugm3;
    float pm10p0_ugm3;
    float battery_soc_percent;
} SensorData_t;

// Function prototypes - only used functions
ESP8266_Status_t ESP8266_Setup(ESP8266_t* esp, UART_HandleTypeDef* huart);
ESP8266_Status_t ESP8266_SendSensorDataHTTP(ESP8266_t* esp, UART_HandleTypeDef* huart, const SensorData_t* sensor_data);

#endif /* ESP8266_H */
