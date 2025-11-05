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
    
    // 2. INICIALIZAR WIFI (AHORA CON MODO AP)
    wifi_init_sta();
    
    // 3. VERIFICAR MODO DE OPERACIÓN
    if (wifi_is_connected()) {
        ESP_LOGI(TAG, "✅ Modo STA - Conectado a WiFi");
        
        // 4. INICIALIZAR MQTT
        mqtt_init();
        
        // 5. INICIALIZAR SENSORES
        init_sensors();
        
        // 6. VERIFICAR ACTUALIZACIONES OTA
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
    
    // 7. LOOP PRINCIPAL
    int cycle_count = 0;
    while (1) {
        cycle_count++;
        
        if (wifi_is_connected()) {
            // Modo STA: Leer sensores y enviar por MQTT
            ESP_LOGI(TAG, "=== CICLO %d (STA) ===", cycle_count);
            float luminosity = read_ldr_value();
            send_mqtt_telemetry(luminosity);
        } else {
            // Modo AP: Solo leer sensores (para I2C/local)
            ESP_LOGI(TAG, "=== CICLO %d (AP) ===", cycle_count);
            float luminosity = read_ldr_value();
            // Los datos están disponibles via I2C para otros dispositivos
        }
        
        vTaskDelay(6000 / portTICK_PERIOD_MS);
    }
}