#include "system_config.h"

static const char *TAG = "OTA_UPDATER";

void check_ota_updates(void) {
    ESP_LOGI(TAG, "🔍 OTA con diagnóstico completo...");
    
    // 1. Primero verificar la URL
    //verify_github_url();
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // 2. Intentar OTA
    ESP_LOGI(TAG, "📥 Iniciando descarga OTA...");
    
    esp_http_client_config_t config = {
        .url = GITHUB_FIRMWARE_URL,
        .timeout_ms = 60000,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .buffer_size = 4096,
    };
    
    esp_https_ota_config_t ota_config = {
        .http_config = &config,
        .partial_http_download = true,
    };
    
    esp_err_t ret = esp_https_ota(&ota_config);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "🎉 OTA EXITOSO! Reiniciando...");
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        esp_restart();
    } else {
        ESP_LOGE(TAG, "❌ OTA falló: %s", esp_err_to_name(ret));
        
    }
}