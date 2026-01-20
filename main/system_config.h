// system_config.h - CONFIGURACIÓN COMPLETA DEL SISTEMA
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
#include <math.h>
#include "esp_timer.h"
#include <time.h>

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

// ==================== CONFIGURACIÓN ADC ====================
#define PLUVIOMETRO_ADC_CHANNEL    ADC1_CHANNEL_4   // GPIO32
#define ANEMOMETRO_ADC_CHANNEL     ADC1_CHANNEL_5   // GPIO33

// Rangos de calibración para sensores ADC (AJUSTAR SEGÚN TU SENSOR)
#define PLUVIOMETRO_MIN_VOLTAGE    0.0f     // 0V = 0mm (seco)
#define PLUVIOMETRO_MAX_VOLTAGE    3.3f     // 3.3V = lluvia máxima
#define PLUVIOMETRO_MAX_MM         100.0f   // Máxima lluvia medible en mm

#define ANEMOMETRO_MIN_VOLTAGE     0.0f     // 0.0V = 0 m/s (calma)
#define ANEMOMETRO_MAX_VOLTAGE     3.3f     // 3.3V = viento máximo
#define ANEMOMETRO_MAX_MS          50.0f    // Máxima velocidad en m/s

// =============================================================================
// CONFIGURACIÓN BME680 - MEJORADA
// =============================================================================
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000
#define BME680_ADDR 0x77

// =============================================================================
// CALIBRACIÓN BME680 (AJUSTAR SEGÚN TU SENSOR)
// =============================================================================
#define BME680_TEMP_SCALE       1.0f      // Factor de escala temperatura
#define BME680_TEMP_OFFSET_C    6.0f      // Offset temperatura en °C (+6°C para corregir)
#define BME680_HUM_SCALE        1.0f      // Factor de escala humedad
#define BME680_HUM_OFFSET_PCT   0.0f      // Offset humedad en %

// Modos de operación (de gschorcht)
typedef enum {
    BME680_SLEEP_MODE = 0,
    BME680_FORCED_MODE,
    BME680_PARALLEL_MODE,
    BME680_NORMAL_MODE = 3
} bme680_mode_t;

// Oversampling rates (de gschorcht)
typedef enum {
    BME680_OSR_NONE = 0,
    BME680_OSR_1X,
    BME680_OSR_2X,
    BME680_OSR_4X,
    BME680_OSR_8X,
    BME680_OSR_16X
} bme680_osr_t;

// =============================================================================
// BUFFER CIRCULAR PARA ALMACENAMIENTO LOCAL
// =============================================================================
#define MAX_BUFFER_SIZE          100     // Máximo 100 lecturas en buffer
#define BUFFER_FLASH_KEY         "sensor_buffer"  // Clave para guardar en flash

// =============================================================================
// ESTRUCTURAS DE DATOS - MEJORADAS
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

// Nueva estructura de configuración (de gschorcht)
typedef struct {
    bme680_osr_t osr_temperature;
    bme680_osr_t osr_pressure;
    bme680_osr_t osr_humidity;
    bme680_mode_t operation_mode;
    uint16_t heater_temperature;
    uint16_t heater_duration;
    int16_t ambient_temperature;
} bme680_config_t;

// Estructura para lectura almacenada en buffer
typedef struct {
    float temperature;
    float humidity;
    float pressure;
    uint32_t gas_resistance;
    float air_quality;
    float rainfall_mm;
    float wind_speed_ms;
    uint32_t timestamp;          // Timestamp UNIX
} stored_reading_t;

// Buffer circular
typedef struct {
    stored_reading_t readings[MAX_BUFFER_SIZE];
    uint16_t head;               // Índice para escribir
    uint16_t tail;               // Índice para leer  
    uint16_t count;              // Número de lecturas almacenadas
    bool buffer_full;            // Flag de buffer lleno
} circular_buffer_t;

// Estructura para datos del sistema
typedef struct {
    float rainfall_mm;
    float wind_speed_ms;
    bme680_data_t bme680;
    bool wifi_connected;
    bool ap_mode;
} system_data_t;

// =============================================================================
// DECLARACIONES DE FUNCIONES - MEJORADAS
// =============================================================================

// WiFi Manager
void wifi_init_sta(void);
bool wifi_is_connected(void);
bool wifi_is_ap_mode(void);
const char* wifi_get_ap_ssid(void);

// MQTT Client
void mqtt_init(void);
void send_mqtt_telemetry(bme680_data_t *bme_data, float rainfall_mm, float wind_speed_ms);
bool send_mqtt_telemetry_with_timestamp(const char *json_message);  // Nueva función
bool mqtt_is_connected(void);

// OTA Updater
void check_ota_updates(void);
void verify_github_url(void);

// Sensor Reader
void init_sensors(void);
float read_pluviometro_value(void);
float read_anemometro_value(void);

// BME680 Sensor - FUNCIONES MEJORADAS
void bme680_init(void);
esp_err_t bme680_read_all_data(bme680_data_t *sensor_data);
bool bme680_is_connected(void);
void bme680_start_reading_task(void);
esp_err_t bme680_configure_sensor(void);
void i2c_diagnostic(void);

// NUEVAS FUNCIONES DE gschorcht INTEGRADAS
esp_err_t bme680_set_oversampling_rates(bme680_osr_t osr_t, bme680_osr_t osr_p, bme680_osr_t osr_h);
esp_err_t bme680_set_heater_profile(uint16_t temperature, uint16_t duration);
esp_err_t bme680_set_operation_mode(bme680_mode_t mode);
esp_err_t bme680_set_ambient_temperature(int16_t temperature);
bme680_mode_t bme680_get_operation_mode(void);
esp_err_t bme680_apply_config(bme680_config_t *config);

// Data Buffer - NUEVO SISTEMA DE ALMACENAMIENTO LOCAL
void data_buffer_init(void);
bool data_buffer_store_reading(bme680_data_t *bme_data, float rainfall_mm, float wind_speed_ms);
bool data_buffer_send_stored_readings(void);
uint16_t data_buffer_get_count(void);
bool data_buffer_is_full(void);
void data_buffer_clear(void);
void data_buffer_print_status(void);

#endif // SYSTEM_CONFIG_H