/*
 * system_config.h
 *
 *  Created on: 4 nov 2025
 *      Author: user2
 */

#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "driver/adc.h"
#include "esp_crt_bundle.h"
#include "esp_mac.h"
#include "driver/i2c.h"

// =============================================================================
// CONFIGURACIÓN - ¡ACTUALIZA CON TUS DATOS REALES!
// =============================================================================
#define WIFI_SSID      "SBC"
#define WIFI_PASS      "SBCwifi$"      
#define MAX_INTENTOS   10
#define AP_PASSWORD    "config123"
#define THINGSBOARD_MQTT_URI "mqtt://demo.thingsboard.io"
#define THINGSBOARD_ACCESS_TOKEN "om8tccnqiv43ukzaprgp"
#define GITHUB_FIRMWARE_URL "https://raw.githubusercontent.com/carlxx2/Estacion_Meteo_T07/main/firmware/firmware.bin"
#define LDR_ADC_CHANNEL    ADC1_CHANNEL_4
// =============================================================================
// CONFIGURACIÓN BME680
// =============================================================================
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000
#define BME680_ADDR 0x76
// =============================================================================
// ESTRUCTURAS DE DATOS
// =============================================================================

// Estructura para datos del sensor BME680
typedef struct {
    float temperature;
    float humidity;
    float pressure;
    uint32_t gas_resistance;
    float air_quality;
    int raw_gas;
} bme680_data_t;

// Estructura para datos del sistema
typedef struct {
    float luminosity;
    bme680_data_t bme680;
    bool wifi_connected;
    bool ap_mode;
} system_data_t;

// =============================================================================
// DECLARACIONES DE FUNCIONES
// =============================================================================

// WiFi Manager
void wifi_init_sta(void);
bool wifi_is_connected(void);
bool wifi_is_ap_mode(void);
const char* wifi_get_ap_ssid(void);

// MQTT Client
void mqtt_init(void);
void send_mqtt_telemetry(float luminosity, bme680_data_t *bme_data);
bool mqtt_is_connected(void);

// OTA Updater
void check_ota_updates(void);
void verify_github_url(void);

// Sensor Reader
void init_sensors(void);
float read_ldr_value(void);

// BME680 Sensor
void bme680_init(void);
esp_err_t bme680_read_all_data(bme680_data_t *sensor_data);
bool bme680_is_connected(void);
void bme680_start_reading_task(void);
esp_err_t bme680_configure_sensor(void);



#endif
