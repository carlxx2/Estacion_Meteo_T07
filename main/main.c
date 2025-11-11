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
        //ESP_LOGI(TAG, "🔍 Verificando OTA...");
        //check_ota_updates();
        
        //ESP_LOGI(TAG, "✅ Sistema operativo en modo STA");
        
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
        	ESP_LOGI(TAG, "=== CICLO %d (STA) ===", cycle_count);
        
        	// Leer LDR (maneja sensores desconectados)
        	float luminosity = read_ldr_value();
        
        	// Leer BME680 - SIEMPRE intentar leer, incluso si falla
        	bme680_data_t bme_data;
        	esp_err_t bme_result = bme680_read_all_data(&bme_data);
        
        	// ✅ ENVIAR SIEMPRE, incluso si el sensor falla
        	send_mqtt_telemetry(luminosity, &bme_data);
        
        	// Log apropiado según el resultado
        	if (bme_result == ESP_OK) {
           		ESP_LOGI(TAG, "📊 BME680 OK - Temp: %.2f°C, Hum: %.1f%%", 
                     bme_data.temperature, bme_data.humidity);
        	} else {
            	ESP_LOGW(TAG, "📊 BME680 FALLIDO - Enviando valores de error");
        }
        
    	} else {
        	// Modo AP
        	ESP_LOGI(TAG, "=== CICLO %d (AP) ===", cycle_count);
        	float luminosity = read_ldr_value();
        
        	// En modo AP también verificar BME680
        	bme680_data_t bme_data;
        	if (bme680_read_all_data(&bme_data) == ESP_OK) {
            	ESP_LOGI(TAG, "📊 BME680 - Temp: %.2f°C", bme_data.temperature);
        	} else {
           	ESP_LOGW(TAG, "⚠️ BME680 no disponible en modo AP");
       	 }
    }
    
    vTaskDelay(10000 / portTICK_PERIOD_MS);
}
}