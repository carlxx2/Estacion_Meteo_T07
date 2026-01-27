#include "system_config.h"

static const char *TAG = "MQTT_CLIENT";

static bool mqtt_conectado = false;
static esp_mqtt_client_handle_t mqtt_client = NULL;

static void mqtt_reconnect(void) {
    static uint32_t last_reconnect_attempt = 0;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    
    if (now - last_reconnect_attempt < 5) {
        return;
    }
    
    last_reconnect_attempt = now;
    
    ESP_LOGI(TAG, "🔄 Intentando reconexión MQTT...");
    
    if (mqtt_client) {
        esp_mqtt_client_stop(mqtt_client);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        esp_mqtt_client_start(mqtt_client);
        ESP_LOGI(TAG, "Reiniciado cliente MQTT");
    }
}

static void mqtt_event_handler(void *arg, esp_event_base_t event_base, 
                              int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_conectado = true;
            ESP_LOGI(TAG, "✅ MQTT conectado a ThingsBoard");
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            mqtt_conectado = false;
            ESP_LOGW(TAG, "❌ MQTT desconectado");
            
            // Programar reconexión automática
            ESP_LOGI(TAG, "Reconectando en 3 segundos...");
            vTaskDelay(3000 / portTICK_PERIOD_MS);
            mqtt_reconnect();
            break;
            
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGD(TAG, "MQTT mensaje publicado (ID: %d)", event->msg_id);
            break;
            
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "Error MQTT");
            mqtt_conectado = false;
            break;
            
        default:
            break;
    }
}

void mqtt_init(void) {
    ESP_LOGI(TAG, "Inicializando MQTT...");
    
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = THINGSBOARD_MQTT_URI,
        .credentials.username = THINGSBOARD_ACCESS_TOKEN,
    };
    
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!mqtt_client) {
        ESP_LOGE(TAG, "Error inicializando cliente MQTT");
        return;
    }
    
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_err_t start_result = esp_mqtt_client_start(mqtt_client);
    if (start_result != ESP_OK) {
        ESP_LOGE(TAG, "Error iniciando cliente MQTT: %s", esp_err_to_name(start_result));
    }
}

void send_mqtt_telemetry(bme680_data_t *bme_data, float rainfall_mm, float wind_speed_ms) {
    if (mqtt_conectado && mqtt_client) {
        char message[512];
        
        // Obtener timestamp actual (milisegundos Unix)
        uint64_t timestamp_ms = time_get_current_ms();
        
        // Crear JSON con timestamp para ThingsBoard
        snprintf(message, sizeof(message),
                "{\"ts\":%" PRIu64 ","           // Timestamp en milisegundos
                "\"temperature\":%.2f,"
                "\"humidity\":%.2f,"
                "\"pressure\":%.2f,"
                "\"gas_resistance\":%lu,"
                "\"air_quality\":%.2f,"
                "\"rainfall_mm\":%.2f,"
                "\"wind_speed_ms\":%.2f,"
                "\"wind_speed_kmh\":%.2f,"
                "\"time_synced\":%s}",
                timestamp_ms,
                bme_data->temperature,
                bme_data->humidity,
                bme_data->pressure,
                (unsigned long)bme_data->gas_resistance,
                bme_data->air_quality,
                rainfall_mm,
                wind_speed_ms,
                wind_speed_ms * 3.6,
                time_is_synced() ? "true" : "false");
        
        int msg_id = esp_mqtt_client_publish(mqtt_client, 
                                           "v1/devices/me/telemetry",
                                           message, 0, 1, 0);
        
        if (msg_id < 0) {
            ESP_LOGE(TAG, "❌ Error publicando telemetría");
        } else {
            // Mostrar timestamp y TODOS los datos en log
            time_t now = time_get_current();
            struct tm *timeinfo = localtime(&now);
            char time_str[32];
            strftime(time_str, sizeof(time_str), "%H:%M:%S", timeinfo);
            
            ESP_LOGI(TAG, "═══════════════════════════════════════════════");
            ESP_LOGI(TAG, "✅ TELEMETRÍA ENVIADA A THINGSBOARD");
            ESP_LOGI(TAG, "   Hora: %s (%s)", 
                     time_str, time_is_synced() ? "NTP" : "estimada");
            ESP_LOGI(TAG, "   Timestamp: %llu ms", timestamp_ms);
            ESP_LOGI(TAG, "═══════════════════════════════════════════════");
            ESP_LOGI(TAG, "🌡️  TEMPERATURA: %.2f °C", bme_data->temperature);
            ESP_LOGI(TAG, "💧 HUMEDAD: %.2f %%", bme_data->humidity);
            ESP_LOGI(TAG, "📊 PRESIÓN: %.2f hPa", bme_data->pressure);
            ESP_LOGI(TAG, "🌀 GAS: %lu Ω", (unsigned long)bme_data->gas_resistance);
            ESP_LOGI(TAG, "🌬️  CALIDAD AIRE: %.2f /100", bme_data->air_quality);
            ESP_LOGI(TAG, "🌧️  LLUVIA: %.2f mm", rainfall_mm);
            ESP_LOGI(TAG, "💨 VIENTO: %.2f m/s (%.2f km/h)", 
                     wind_speed_ms, wind_speed_ms * 3.6);
            ESP_LOGI(TAG, "═══════════════════════════════════════════════");
            ESP_LOGI(TAG, "📤 JSON enviado (%d bytes):", strlen(message));
            ESP_LOGI(TAG, "═══════════════════════════════════════════════");
        }
    } else {
        ESP_LOGW(TAG, "⚠️ MQTT no conectado, no se pueden enviar datos");
    }
}

bool send_mqtt_telemetry_with_timestamp(const char *json_message) {
    if (mqtt_conectado && mqtt_client) {
        int msg_id = esp_mqtt_client_publish(mqtt_client, 
                                           "v1/devices/me/telemetry",
                                           json_message, 0, 1, 0);
        
        if (msg_id < 0) {
            ESP_LOGE(TAG, "❌ Error publicando telemetría con timestamp");
            return false;
        }
        return true;
    }
    return false;
}

bool mqtt_is_connected(void) {
    return mqtt_conectado;
}