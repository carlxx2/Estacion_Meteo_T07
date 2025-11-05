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

// =============================================================================
// CONFIGURACIÓN - ¡ACTUALIZA CON TUS DATOS REALES!
// =============================================================================
#define WIFI_SSID      "SBC"
#define WIFI_PASS      "SBCwifi$"      
#define MAX_INTENTOS   10
#define THINGSBOARD_MQTT_URI "mqtt://demo.thingsboard.io"
#define THINGSBOARD_ACCESS_TOKEN "CC34vYEp44Z00eoPKLfV"
#define GITHUB_FIRMWARE_URL "https://github.com/carlxx2/Estacion_Meteo_T07/main/firmware/firmware.bin"
#define LDR_ADC_CHANNEL    ADC1_CHANNEL_4

// =============================================================================
// DECLARACIONES DE FUNCIONES
// =============================================================================

// WiFi Manager
void wifi_init_sta(void);
bool wifi_is_connected(void);

// MQTT Client
void mqtt_init(void);
void send_mqtt_telemetry(float luminosity);
bool mqtt_is_connected(void);

// OTA Updater
void check_ota_updates(void);
void verify_github_url(void);

// Sensor Reader
void init_sensors(void);
float read_ldr_value(void);



#endif
