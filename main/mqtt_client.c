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

void send_mqtt_telemetry(float luminosity) {
    if (mqtt_conectado && mqtt_client) {
        char message[100];
        snprintf(message, sizeof(message), "{\"luminosity\":%.2f}", luminosity);
        
        int msg_id = esp_mqtt_client_publish(mqtt_client, 
                                           "v1/devices/me/telemetry",
                                           message, 0, 1, 0);
        
        if (msg_id < 0) {
            ESP_LOGE(TAG, "❌ Error publicando mensaje MQTT");
        } else {
            ESP_LOGI(TAG, "📤 Datos enviados: %s", message);
        }
    } else {
        ESP_LOGW(TAG, "⚠️ MQTT no conectado, no se pueden enviar datos");
    }
}

bool mqtt_is_connected(void) {
    return mqtt_conectado;
}