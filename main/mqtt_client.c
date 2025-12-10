#include "system_config.h"

static const char *TAG = "MQTT_CLIENT";

static bool mqtt_conectado = false;
static esp_mqtt_client_handle_t mqtt_client = NULL;

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
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "📤 MQTT mensaje publicado (ID: %d)", event->msg_id);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "❌ Error MQTT");
            break;
        default:
            break;
    }
}

void mqtt_init(void) {
    ESP_LOGI(TAG, "🌐 Inicializando MQTT...");
    
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = THINGSBOARD_MQTT_URI,
        .credentials.username = THINGSBOARD_ACCESS_TOKEN,
    };
    
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!mqtt_client) {
        ESP_LOGE(TAG, "❌ Error inicializando cliente MQTT");
        return;
    }
    
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_err_t start_result = esp_mqtt_client_start(mqtt_client);
    if (start_result != ESP_OK) {
        ESP_LOGE(TAG, "❌ Error iniciando cliente MQTT: %s", esp_err_to_name(start_result));
    }
}

// =============================================================================
// FUNCIÓN ACTUALIZADA SIN LUMINOSIDAD
// =============================================================================
void send_mqtt_telemetry(bme680_data_t *bme_data, float rainfall_mm, float wind_speed_ms) {
    if (mqtt_conectado && mqtt_client) {
        char message[512];
        
        // Crear JSON con datos meteorológicos
        snprintf(message, sizeof(message),
                "{\"temperature\":%.2f,"
                "\"humidity\":%.2f,"
                "\"pressure\":%.2f,"
                "\"gas_resistance\":%lu,"
                "\"air_quality\":%.2f,"
                "\"rainfall_mm\":%.2f,"      // Lluvia en mm
                "\"wind_speed_ms\":%.2f,"    // Viento en m/s
                "\"wind_speed_kmh\":%.2f}",  // Viento en km/h
                bme_data->temperature,
                bme_data->humidity,
                bme_data->pressure,
                (unsigned long)bme_data->gas_resistance,
                bme_data->air_quality,
                rainfall_mm,
                wind_speed_ms,
                wind_speed_ms * 3.6);
        
        int msg_id = esp_mqtt_client_publish(mqtt_client, 
                                           "v1/devices/me/telemetry",
                                           message, 0, 1, 0);
        
        if (msg_id < 0) {
            ESP_LOGE(TAG, "❌ Error publicando telemetría por MQTT");
        } else {
            ESP_LOGI(TAG, "📤 Telemetría enviada - Temp: %.2f°C, Lluvia: %.2fmm, Viento: %.1fm/s", 
                     bme_data->temperature, rainfall_mm, wind_speed_ms);
        }
    } else {
        ESP_LOGW(TAG, "⚠️ MQTT no conectado, no se pueden enviar datos");
    }
}

bool mqtt_is_connected(void) {
    return mqtt_conectado;
}