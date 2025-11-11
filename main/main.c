#include "system_config.h"

static const char *TAG = "MAIN_SYSTEM";

void app_main(void) {
    ESP_LOGI(TAG, "🚀 Iniciando Sistema...");
    
    // 1. INICIALIZAR NVS
    ESP_LOGI(TAG, "📁 Inicializando NVS...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // 2. INICIALIZAR BME680
    bme680_init();
    if (bme680_configure_sensor() == ESP_OK) {
        ESP_LOGI(TAG, "✅ BME680 inicializado correctamente");
        
        // Lectura inicial de prueba
	bme680_data_t sensor_data;
	if (bme680_read_all_data(&sensor_data) == ESP_OK) {
    	ESP_LOGI(TAG, "📊 Lectura inicial BME680:");
    	ESP_LOGI(TAG, "  🌡️  Temperatura: %.2f°C", sensor_data.temperature);
    	ESP_LOGI(TAG, "  💧 Humedad: %.1f%%", sensor_data.humidity);
    	ESP_LOGI(TAG, "  📊 Presión: %.2f hPa", sensor_data.pressure);
    	ESP_LOGI(TAG, "  🌫️  Gas: %lu Ω", (unsigned long)sensor_data.gas_resistance);  // ✅ CORREGIDO
    	ESP_LOGI(TAG, "  🎯 Calidad Aire: %.1f/100", sensor_data.air_quality);
	}
    } else {
        ESP_LOGE(TAG, "❌ Error inicializando BME680");
    }
    // 3. INICIALIZAR WIFI (AHORA CON MODO AP)
    wifi_init_sta();
    
    if (wifi_is_connected()) {
        ESP_LOGI(TAG, "✅ Modo STA - Conectado a WiFi");
        
        // 5. INICIALIZAR MQTT
        mqtt_init();
        
        // 6. INICIALIZAR SENSORES
        init_sensors();
        
        // 7. VERIFICAR ACTUALIZACIONES OTA
        ESP_LOGI(TAG, "🔍 Verificando OTA...");
        check_ota_updates();
        
        ESP_LOGI(TAG, "✅ Sistema operativo en modo STA");
        
    } else {
        ESP_LOGW(TAG, "📡 Modo AP - Servidor de configuración activo");
        ESP_LOGI(TAG, "   SSID: %s", wifi_get_ap_ssid());
        ESP_LOGI(TAG, "   Contraseña: %s", AP_PASSWORD);
        ESP_LOGI(TAG, "   IP: 192.168.4.1");
        
        // En modo AP, también inicializar sensores pero no MQTT
        init_sensors();
    }
    
    // 8. LOOP PRINCIPAL
int cycle_count = 0;
    while (1) {
        cycle_count++;
        
        if (wifi_is_connected() && mqtt_is_connected()) {
            // Modo STA: Leer sensores y enviar por MQTT
            ESP_LOGI(TAG, "=== CICLO %d (STA) ===", cycle_count);
            
            // Leer LDR
            float luminosity = read_ldr_value();
            
            // Leer BME680
            bme680_data_t bme_data;
            if (bme680_read_all_data(&bme_data) == ESP_OK) {
                // ✅ ENVÍO ÚNICO CON TODOS LOS DATOS
                send_mqtt_telemetry(luminosity, &bme_data);
                
                ESP_LOGI(TAG, "📊 Datos leídos - Lumin: %.2f, Temp: %.2f°C, Hum: %.1f%%, Gas: %luΩ", 
                         luminosity, bme_data.temperature, bme_data.humidity, 
                         (unsigned long)bme_data.gas_resistance);
            }
            
        } else {
            // Modo AP: Solo leer sensores (para I2C/local)
            ESP_LOGI(TAG, "=== CICLO %d (AP) ===", cycle_count);
            float luminosity = read_ldr_value();
            
            // Leer BME680 para mostrar datos localmente
            bme680_data_t bme_data;
            if (bme680_read_all_data(&bme_data) == ESP_OK) {
                ESP_LOGI(TAG, "📊 BME680 - Temp: %.2f°C, Hum: %.1f%%, Pres: %.2fhPa, Gas: %luΩ", 
                         bme_data.temperature, bme_data.humidity, bme_data.pressure, 
                         (unsigned long)bme_data.gas_resistance);
            }
            
            // Intentar reconectar MQTT si WiFi está disponible
            if (wifi_is_connected() && !mqtt_is_connected()) {
                ESP_LOGW(TAG, "🔄 Intentando reconectar MQTT...");
                mqtt_init();
            }
        }
        
        vTaskDelay(10000 / portTICK_PERIOD_MS); // 10 segundos entre ciclos
    }
}