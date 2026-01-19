#include "system_config.h"

static const char *TAG = "OTA_MANAGER";

void verify_github_url(void) {
    ESP_LOGI(TAG, "Verificando URL de GitHub...");
    ESP_LOGI(TAG, "URL actual: %s", GITHUB_FIRMWARE_URL);
    
    // Prueba estas URLs alternativas:
    const char* test_urls[] = {
        "https://raw.githubusercontent.com/carlxx2/Hito_6/main/firmware/hito6.bin",
        "https://raw.githubusercontent.com/carlxx2/Hito_6/main/build/hito6.bin",
        "https://github.com/carlxx2/Hito_6/raw/main/firmware/hito6.bin",
    };
    
    for (int i = 0; i < sizeof(test_urls)/sizeof(test_urls[0]); i++) {
        ESP_LOGI(TAG, "Probando: %s", test_urls[i]);
        
        esp_http_client_config_t config = {
            .url = test_urls[i],
            .method = HTTP_METHOD_HEAD,
            .timeout_ms = 10000,
            .crt_bundle_attach = esp_crt_bundle_attach,
        };
        
        esp_http_client_handle_t client = esp_http_client_init(&config);
        esp_err_t err = esp_http_client_perform(client);
        
        if (err == ESP_OK) {
            int status = esp_http_client_get_status_code(client);
            ESP_LOGI(TAG, "Status: %d - %s", status, 
                    status == 200 ? "ARCHIVO ENCONTRADO" : "NO ENCONTRADO");
        } else {
            ESP_LOGE(TAG, "Error: %s", esp_err_to_name(err));
        }
        
        esp_http_client_cleanup(client);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void check_ota_updates(void) {
    ESP_LOGI(TAG, "OTA con diagnóstico completo...");
    
    // 1. Primero verificar la URL
    verify_github_url();
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // 2. Intentar OTA
    ESP_LOGI(TAG, "Iniciando descarga OTA...");
    
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
        ESP_LOGI(TAG, "OTA EXITOSO! Reiniciando...");
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA falló: %s", esp_err_to_name(ret));
        
        // Diagnóstico específico
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Posibles causas:");
            ESP_LOGE(TAG, "   • Archivo no existe en GitHub (404)");
            ESP_LOGE(TAG, "   • Ruta incorrecta");
            ESP_LOGE(TAG, "   • Archivo no commiteado");
        }
    }
}