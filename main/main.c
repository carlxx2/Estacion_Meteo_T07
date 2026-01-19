#include "system_config.h"

static const char *TAG = "MAIN_SYSTEM";

void app_main(void) {
    ESP_LOGI(TAG, "Iniciando Estacion Meteorologica...");
    
    // 1. INICIALIZAR NVS
    ESP_LOGI(TAG, "Inicializando NVS...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // 2. INICIALIZAR BME680
    bme680_init();
    if (bme680_configure_sensor() == ESP_OK) {
        ESP_LOGI(TAG, "BME680 inicializado correctamente");
        
        // Lectura inicial de prueba
        bme680_data_t sensor_data;
        if (bme680_read_all_data(&sensor_data) == ESP_OK) {
            ESP_LOGI(TAG, "Lectura inicial BME680:");
            ESP_LOGI(TAG, "  Temperatura: %.2fC", sensor_data.temperature);
            ESP_LOGI(TAG, "  Humedad: %.1f%%", sensor_data.humidity);
            ESP_LOGI(TAG, "  Presion: %.2f hPa", sensor_data.pressure);
            ESP_LOGI(TAG, "  Gas: %lu Ohm", (unsigned long)sensor_data.gas_resistance);
            ESP_LOGI(TAG, "  Calidad Aire: %.1f/100", sensor_data.air_quality);
            ESP_LOGI(TAG, "  Gas raw: %d", sensor_data.raw_gas);
        }
    } else {
        ESP_LOGE(TAG, "Error inicializando BME680");
    }
    
    // 3. INICIALIZAR WIFI
    wifi_init_sta();
    
    // 4. INICIALIZAR SENSORES METEOROLÓGICOS
    init_sensors();
    
    // 5. CONFIGURAR MQTT SI HAY WIFI
    if (wifi_is_connected()) {
        ESP_LOGI(TAG, "Modo STA - Conectado a WiFi");
        mqtt_init();
    } else {
        ESP_LOGW(TAG, "Modo AP - Servidor de configuracion activo");
        ESP_LOGI(TAG, "   SSID: %s", wifi_get_ap_ssid());
        ESP_LOGI(TAG, "   Contrasena: %s", AP_PASSWORD);
        ESP_LOGI(TAG, "   IP: 192.168.4.1");
    }
    
    // 6. LOOP PRINCIPAL
    int cycle_count = 0;
    while (1) {
        cycle_count++;
        
        if (wifi_is_connected() && mqtt_is_connected()) {
            ESP_LOGI(TAG, "=== CICLO %d (STA - Envio a nube) ===", cycle_count);
            
            // Leer BME680
            bme680_data_t bme_data;
            esp_err_t bme_result = bme680_read_all_data(&bme_data);
            
            // Leer sensores meteorológicos
            float rainfall_mm = read_pluviometro_value();
            float wind_speed_ms = read_anemometro_value();
            
            // Enviar todos los datos por MQTT
            send_mqtt_telemetry(&bme_data, rainfall_mm, wind_speed_ms);
            
        } else {
            // Modo AP o sin MQTT - solo lecturas locales
            ESP_LOGI(TAG, "=== CICLO %d (AP - Solo lectura local) ===", cycle_count);
            
            // Leer sensores para monitor local
            bme680_data_t bme_data;
            bme680_read_all_data(&bme_data);
            float rainfall = read_pluviometro_value();
            float wind_speed = read_anemometro_value();
            
            ESP_LOGI(TAG, "Lectura local completa:");
            ESP_LOGI(TAG, "  Temperatura: %.1fC, Humedad: %.1f%%, Presion: %.2fhPa", 
                     bme_data.temperature, bme_data.humidity, bme_data.pressure);
            ESP_LOGI(TAG, "  Gas: %lu Ohm, Calidad: %.1f/100", 
                     (unsigned long)bme_data.gas_resistance, bme_data.air_quality);
            ESP_LOGI(TAG, "  Lluvia: %.1fmm, Viento: %.1fm/s (%.1f km/h)",
                     rainfall, wind_speed, wind_speed * 3.6);
        }
        
        // Esperar 5 segundos entre lecturas (ajustable)
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}