/* esp8266.c */
#include "esp8266.h"
#include <stdlib.h>

// Internal function prototypes
static ESP8266_Status_t ESP8266_Init(ESP8266_t* esp, UART_HandleTypeDef* huart);
static ESP8266_Status_t ESP8266_ConnectWiFi(ESP8266_t* esp, const char* ssid, const char* password);
static ESP8266_Status_t ESP8266_SendHTTP(ESP8266_t* esp, const char* host, const char* port,
                                 const char* endpoint, const char* json_data);
static ESP8266_Status_t ESP8266_SendCommand(ESP8266_t* esp, const char* cmd, const char* expected, uint32_t timeout);
static ESP8266_Status_t ESP8266_CheckConnection(ESP8266_t* esp);
static void ESP8266_ClearBuffer(ESP8266_t* esp);
static ESP8266_Status_t ESP8266_GetServerTime(ESP8266_t* esp, UART_HandleTypeDef* huart, uint32_t* timestamp);
static void ESP8266_FloatToString2Dec(float value, char *buffer);
static ESP8266_Status_t ESP8266_ParseTimestampFromResponse(const char* response, uint32_t* timestamp);

static ESP8266_Status_t ESP8266_Init(ESP8266_t* esp, UART_HandleTypeDef* huart) {
    esp->huart = huart;
    esp->wifi_connected = 0;

    ESP8266_ClearBuffer(esp);

    // Wait for ESP8266 boot
    HAL_Delay(3000);

    // Test communication
    if (ESP8266_SendCommand(esp, "AT\r\n", "OK", 3000) != ESP8266_OK) {
        return ESP8266_ERROR;
    }

    // Reset ESP8266 for clean start
    ESP8266_SendCommand(esp, "AT+RST\r\n", "ready", 5000);
    HAL_Delay(2000);

    // Disable echo
    ESP8266_SendCommand(esp, "ATE0\r\n", "OK", 2000);

    // Set WiFi mode to station
    if (ESP8266_SendCommand(esp, "AT+CWMODE=1\r\n", "OK", 3000) != ESP8266_OK) {
        return ESP8266_ERROR;
    }

    return ESP8266_OK;
}

static ESP8266_Status_t ESP8266_ConnectWiFi(ESP8266_t* esp, const char* ssid, const char* password) {
    char cmd[128];

    strncpy(esp->ssid, ssid, sizeof(esp->ssid) - 1);
    strncpy(esp->password, password, sizeof(esp->password) - 1);

    // Disconnect from any previous connection
    ESP8266_SendCommand(esp, "AT+CWQAP\r\n", "OK", 3000);
    HAL_Delay(1000);

    // Connect to WiFi
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, password);

    ESP8266_Status_t result = ESP8266_SendCommand(esp, cmd, "WIFI CONNECTED", 25000);

    if (result == ESP8266_OK) {
        esp->wifi_connected = 1;
        HAL_Delay(3000); // Wait for IP assignment

        // Check IP address
        ESP8266_SendCommand(esp, "AT+CIFSR\r\n", "OK", 3000);

        return ESP8266_OK;
    }

    // Check alternative success responses
    if (strstr(esp->buffer, "WIFI GOT IP") || strstr(esp->buffer, "OK")) {
        esp->wifi_connected = 1;
        return ESP8266_OK;
    }

    return ESP8266_ERROR;
}

static ESP8266_Status_t ESP8266_CheckConnection(ESP8266_t* esp) {
    if (ESP8266_SendCommand(esp, "AT+CWJAP?\r\n", "OK", 3000) == ESP8266_OK) {
        if (strstr(esp->buffer, "No AP")) {
            esp->wifi_connected = 0;
            return ESP8266_WIFI_NOT_CONNECTED;
        } else {
            esp->wifi_connected = 1;
            return ESP8266_OK;
        }
    }
    return ESP8266_ERROR;
}

static ESP8266_Status_t ESP8266_SendHTTP(ESP8266_t* esp, const char* host, const char* port,
                                 const char* endpoint, const char* json_data) {
    char cmd[128];
    char http_request[600];
    char send_cmd[32];

    if (!esp->wifi_connected) {
        return ESP8266_WIFI_NOT_CONNECTED;
    }

    // Prepare HTTP request
    snprintf(http_request, sizeof(http_request),
             "POST %s HTTP/1.1\r\n"
             "Host: %s:%s\r\n"
             "Content-Type: application/json\r\n"
             "Content-Length: %d\r\n"
             "Connection: close\r\n\r\n"
             "%s",
             endpoint, host, port, (int)strlen(json_data), json_data);

    // Start TCP connection
    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%s\r\n", host, port);

    if (ESP8266_SendCommand(esp, cmd, "CONNECT", 15000) != ESP8266_OK) {
        return ESP8266_SERVER_ERROR;
    }

    // Send data length
    snprintf(send_cmd, sizeof(send_cmd), "AT+CIPSEND=%d\r\n", (int)strlen(http_request));

    if (ESP8266_SendCommand(esp, send_cmd, ">", 5000) != ESP8266_OK) {
        ESP8266_SendCommand(esp, "AT+CIPCLOSE\r\n", "OK", 3000);
        return ESP8266_ERROR;
    }

    // Send HTTP request
    if (ESP8266_SendCommand(esp, http_request, "SEND OK", 10000) != ESP8266_OK) {
        ESP8266_SendCommand(esp, "AT+CIPCLOSE\r\n", "OK", 3000);
        return ESP8266_ERROR;
    }

    // Wait for response
    HAL_Delay(1000);

    // Close connection
    ESP8266_SendCommand(esp, "AT+CIPCLOSE\r\n", "OK", 3000);

    return ESP8266_OK;
}

static ESP8266_Status_t ESP8266_SendCommand(ESP8266_t* esp, const char* cmd, const char* expected, uint32_t timeout) {
    uint32_t start_time = HAL_GetTick();

    ESP8266_ClearBuffer(esp);

    // Send command
    if (HAL_UART_Transmit(esp->huart, (uint8_t*)cmd, strlen(cmd), 2000) != HAL_OK) {
        return ESP8266_ERROR;
    }

    // Wait for response
    while ((HAL_GetTick() - start_time) < timeout) {
        uint8_t received_char;
        if (HAL_UART_Receive(esp->huart, &received_char, 1, 100) == HAL_OK) {

            // Prevent buffer overflow
            if (strlen(esp->buffer) < (ESP8266_BUFFER_SIZE - 2)) {
                strncat(esp->buffer, (char*)&received_char, 1);
            }

            // Check if expected response is received
            if (expected && strstr(esp->buffer, expected)) {
                return ESP8266_OK;
            }

            // Check for common error responses
            if (strstr(esp->buffer, "ERROR") || strstr(esp->buffer, "FAIL")) {
                return ESP8266_ERROR;
            }
        }
    }

    return ESP8266_TIMEOUT_ERROR;
}

static void ESP8266_ClearBuffer(ESP8266_t* esp) {
    memset(esp->buffer, 0, ESP8266_BUFFER_SIZE);
}

static void ESP8266_FloatToString2Dec(float value, char *buffer)
{
  int32_t integer_part;
  int32_t decimal_part;

  if (value < 0) {
    *buffer++ = '-';
    value = -value;
  }

  integer_part = (int32_t)value;
  decimal_part = (int32_t)((value - integer_part) * 100 + 0.5f);

  if (decimal_part >= 100) {
    integer_part++;
    decimal_part = 0;
  }

  sprintf(buffer, "%ld.%02ld", integer_part, decimal_part);
}

static ESP8266_Status_t ESP8266_ParseTimestampFromResponse(const char* response, uint32_t* timestamp)
{
  char *json_start, *unix_start, *unix_end;
  char timestamp_str[16];
  int i;

  json_start = strstr(response, "{");
  if (json_start == NULL) {
    return ESP8266_ERROR;
  }

  unix_start = strstr(json_start, "\"unix_s\"");
  if (unix_start == NULL) {
    return ESP8266_ERROR;
  }

  unix_start = strchr(unix_start, ':');
  if (unix_start == NULL) {
    return ESP8266_ERROR;
  }
  unix_start++;

  while (*unix_start == ' ' || *unix_start == '\t') {
    unix_start++;
  }

  unix_end = unix_start;
  while (*unix_end >= '0' && *unix_end <= '9') {
    unix_end++;
  }

  int timestamp_length = unix_end - unix_start;
  if (timestamp_length <= 0 || timestamp_length >= sizeof(timestamp_str)) {
    return ESP8266_ERROR;
  }

  for (i = 0; i < timestamp_length; i++) {
    timestamp_str[i] = unix_start[i];
  }
  timestamp_str[timestamp_length] = '\0';

  *timestamp = strtoul(timestamp_str, NULL, 10);

  if (*timestamp < 1700000000 || *timestamp > 2000000000) {
    return ESP8266_ERROR;
  }

  return ESP8266_OK;
}

static ESP8266_Status_t ESP8266_GetServerTime(ESP8266_t* esp, UART_HandleTypeDef* huart, uint32_t *timestamp)
{
  ESP8266_Status_t status;
  char response_buffer[512];
  char at_command[256];
  int bytes_received = 0;
  uint32_t timeout_start;

  status = ESP8266_CheckConnection(esp);
  if (status != ESP8266_OK) {
    return status;
  }

  memset(response_buffer, 0, sizeof(response_buffer));

  snprintf(at_command, sizeof(at_command), "AT+CIPSTART=\"TCP\",\"%s\",%s", SERVER_HOST, SERVER_PORT);
  HAL_UART_Transmit(huart, (uint8_t*)at_command, strlen(at_command), 1000);
  HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, 1000);
  HAL_Delay(1000);

  char http_request[300];
  int request_length = snprintf(http_request, sizeof(http_request),
    "GET %s HTTP/1.1\r\n"
    "Host: %s:%s\r\n"
    "Connection: close\r\n"
    "Content-Type: application/json\r\n"
    "\r\n",
    API_TIME_ENDPOINT, SERVER_HOST, SERVER_PORT);

  snprintf(at_command, sizeof(at_command), "AT+CIPSEND=%d", request_length);
  HAL_UART_Transmit(huart, (uint8_t*)at_command, strlen(at_command), 1000);
  HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, 1000);
  HAL_Delay(500);

  HAL_UART_Transmit(huart, (uint8_t*)http_request, request_length, 2000);

  timeout_start = HAL_GetTick();
  bytes_received = 0;

  while ((HAL_GetTick() - timeout_start) < 5000 && bytes_received < (sizeof(response_buffer) - 1)) {
    uint8_t received_byte;
    if (HAL_UART_Receive(huart, &received_byte, 1, 100) == HAL_OK) {
      response_buffer[bytes_received++] = received_byte;
    }
  }

  response_buffer[bytes_received] = '\0';

  HAL_UART_Transmit(huart, (uint8_t*)"AT+CIPCLOSE\r\n", 13, 1000);
  HAL_Delay(500);

  if (bytes_received > 0) {
    return ESP8266_ParseTimestampFromResponse(response_buffer, timestamp);
  }

  return ESP8266_ERROR;
}

// Public functions used by main
ESP8266_Status_t ESP8266_Setup(ESP8266_t* esp, UART_HandleTypeDef* huart)
{
  ESP8266_Status_t status;

  status = ESP8266_Init(esp, huart);
  if (status != ESP8266_OK) {
    return status;
  }

  status = ESP8266_ConnectWiFi(esp, WIFI_SSID, WIFI_PASSWORD);
  if (status != ESP8266_OK) {
    return status;
  }

  return ESP8266_OK;
}

ESP8266_Status_t ESP8266_SendSensorDataHTTP(ESP8266_t* esp, UART_HandleTypeDef* huart, const SensorData_t* sensor_data)
{
  ESP8266_Status_t status;
  char json_payload[700];  /* Reduced size for 7 measurements */
  char temp_str[12], humid_str[12], pressure_str[12], light_str[12];
  char pm25_str[12], pm10_str[12], soc_str[12];
  uint32_t server_timestamp = 0;

  status = ESP8266_CheckConnection(esp);
  if (status != ESP8266_OK) {
    ESP8266_ConnectWiFi(esp, WIFI_SSID, WIFI_PASSWORD);
    return status;
  }

  ESP8266_GetServerTime(esp, huart, &server_timestamp);

  /* Convert float values to strings */
  ESP8266_FloatToString2Dec(sensor_data->temperature_celsius, temp_str);
  ESP8266_FloatToString2Dec(sensor_data->humidity_percent, humid_str);
  ESP8266_FloatToString2Dec(sensor_data->pressure_hpa, pressure_str);
  ESP8266_FloatToString2Dec(sensor_data->ambient_light_lux, light_str);
  ESP8266_FloatToString2Dec(sensor_data->pm2p5_ugm3, pm25_str);
  ESP8266_FloatToString2Dec(sensor_data->pm10p0_ugm3, pm10_str);
  ESP8266_FloatToString2Dec(sensor_data->battery_soc_percent, soc_str);

  /* Build JSON payload with 7 measurements */
  snprintf(json_payload, sizeof(json_payload),
           "{"
           "\"uid\":\"%s\","
           "\"val_ids\":[%d,%d,%d,%d,%d,%d,%d],"
           "\"unix_s\":[%lu,%lu,%lu,%lu,%lu,%lu,%lu],"
           "\"vals\":[%s,%s,%s,%s,%s,%s,%s]"
           "}",
           USER_ID,
           VAL_ID_TEMPERATURE, VAL_ID_HUMIDITY, VAL_ID_PRESSURE, VAL_ID_LIGHT,
           VAL_ID_PM2P5, VAL_ID_PM10P0, VAL_ID_SOC,
           server_timestamp, server_timestamp, server_timestamp, server_timestamp,
           server_timestamp, server_timestamp, server_timestamp,
           temp_str, humid_str, pressure_str, light_str, pm25_str, pm10_str, soc_str);

  status = ESP8266_SendHTTP(esp, SERVER_HOST, SERVER_PORT, API_DATA_ENDPOINT, json_payload);

  return status;
}
