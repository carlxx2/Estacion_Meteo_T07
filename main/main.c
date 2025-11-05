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
    
    // 2. INICIALIZAR WIFI
    wifi_init_sta();
    
    if (wifi_is_connected()) {
        // 3. INICIALIZAR MQTT
        mqtt_init();
        
        // 4. INICIALIZAR SENSORES
        init_sensors();
        
        // 5. VERIFICAR ACTUALIZACIONES OTA
        ESP_LOGI(TAG, "🔍 Verificando OTA...");
        check_ota_updates();
        
        ESP_LOGI(TAG, "✅ Sistema operativo");
        
        // 6. LOOP PRINCIPAL
        int cycle_count = 0;
        while (1) {
            cycle_count++;
            ESP_LOGI(TAG, "=== CICLO %d ===", cycle_count);
            
            float luminosity = read_ldr_value();
            send_mqtt_telemetry(luminosity);
            
            vTaskDelay(6000 / portTICK_PERIOD_MS);
        }
    } else {
        ESP_LOGE(TAG, "❌ Fallo WiFi - Reiniciando...");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        esp_restart();
    }
}