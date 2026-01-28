#include "system_config.h"

static const char *TAG = "MQTT_CLIENT";

static bool mqtt_conectado = false;
static esp_mqtt_client_handle_t mqtt_client = NULL;
static uint32_t mqtt_reconnect_attempts = 0;
static uint32_t last_mqtt_reconnect_time = 0;
static TimerHandle_t mqtt_reconnect_timer = NULL;
static SemaphoreHandle_t mqtt_reconnect_mutex = NULL;



/**
 * @brief Intenta reconectar MQTT de forma segura
 */
static void mqtt_safe_reconnect(void) {
	if (mqtt_reconnect_mutex == NULL) {
        mqtt_reconnect_mutex = xSemaphoreCreateMutex();
    }
    if (xSemaphoreTake(mqtt_reconnect_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    static uint32_t last_reconnect_attempt = 0;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    
    // Rate limiting: no intentar reconectar más de una vez cada 5 segundos
    if (now - last_reconnect_attempt < 5) {
        ESP_LOGD(TAG, "Reconexión demasiado frecuente, esperando...");
        return;
    }
    
    last_reconnect_attempt = now;
    mqtt_reconnect_attempts++;
    
    ESP_LOGI(TAG, "🔄 Intento de reconexión MQTT #%"PRIu32, mqtt_reconnect_attempts);
    
    // Backoff exponencial: 3s, 6s, 12s, 24s, 30s (máximo)
    uint32_t backoff_seconds = 3 * (1 << (mqtt_reconnect_attempts - 1));
    if (backoff_seconds > 30) backoff_seconds = 30;
    
    if (mqtt_reconnect_attempts > 1) {
        ESP_LOGI(TAG, "⏳ Esperando %"PRIu32" segundos antes de reintentar...", backoff_seconds);
        vTaskDelay(backoff_seconds * 1000 / portTICK_PERIOD_MS);
    }
    
    if (mqtt_client) {
        // 1. Primero verificar si el cliente ya está iniciado
        ESP_LOGI(TAG, "Deteniendo cliente MQTT...");
        esp_err_t stop_ret = esp_mqtt_client_stop(mqtt_client);
        if (stop_ret != ESP_OK) {
            ESP_LOGW(TAG, "Advertencia al detener MQTT: %s", esp_err_to_name(stop_ret));
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        // 2. Reiniciar completamente si hay muchos intentos fallidos
        if (mqtt_reconnect_attempts > 3) {
            ESP_LOGW(TAG, "⚠️ Muchos intentos fallidos, reiniciando cliente MQTT...");
            esp_mqtt_client_destroy(mqtt_client);
            mqtt_client = NULL;
            
            // Volver a inicializar
            mqtt_init();
            return;
        }
        
        // 3. Intentar iniciar de nuevo
        ESP_LOGI(TAG, "Iniciando cliente MQTT...");
        esp_err_t start_ret = esp_mqtt_client_start(mqtt_client);
        if (start_ret != ESP_OK) {
            ESP_LOGE(TAG, "❌ Error iniciando MQTT: %s", esp_err_to_name(start_ret));
        } else {
            ESP_LOGI(TAG, "✅ Cliente MQTT reiniciado");
        }
    } else {
        // Si no hay cliente, inicializar uno nuevo
        ESP_LOGW(TAG, "⚠️ Cliente MQTT no existe, inicializando nuevo...");
        mqtt_init();
      }
    }
}

static void mqtt_reconnect_timer_callback(TimerHandle_t xTimer) {
    ESP_LOGI(TAG, "⏰ Timer de reconexión MQTT activado");
    mqtt_safe_reconnect();
}

/**
 * @brief Reinicia completamente el cliente MQTT
 */
void mqtt_force_reconnect(void) {
    ESP_LOGI(TAG, "🔄 FORZANDO RECONEXIÓN COMPLETA MQTT");
    
    mqtt_conectado = false;
    mqtt_reconnect_attempts = 0;
    
    if (mqtt_client) {
        esp_mqtt_client_stop(mqtt_client);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
    }
    
    // Pequeña pausa
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // Re-inicializar
    mqtt_init();
}

static void mqtt_event_handler(void *arg, esp_event_base_t event_base, 
                              int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_conectado = true;
            mqtt_reconnect_attempts = 0;  // Resetear contador al conectar
            last_mqtt_reconnect_time = (uint32_t)(esp_timer_get_time() / 1000000ULL);
            
            // Detener timer de reconexión si está activo
            if (mqtt_reconnect_timer != NULL) {
                xTimerStop(mqtt_reconnect_timer, 0);
            }
            
            ESP_LOGI(TAG, "✅ MQTT conectado a ThingsBoard");
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            mqtt_conectado = false;
            ESP_LOGW(TAG, "❌ MQTT desconectado");
            
            // Solo intentar reconectar si hay WiFi
            if (wifi_is_connected()) {
                ESP_LOGI(TAG, "WiFi está conectado, programando reconexión MQTT en 3 segundos...");
                
                // Crear timer si no existe
                if (mqtt_reconnect_timer == NULL) {
                    mqtt_reconnect_timer = xTimerCreate(
                        "mqtt_reconnect", 
                        pdMS_TO_TICKS(3000),  // 3 segundos
                        pdFALSE, 
                        (void*)0, 
                        mqtt_reconnect_timer_callback
                    );
                }
                
                if (mqtt_reconnect_timer != NULL) {
                    xTimerStart(mqtt_reconnect_timer, 0);
                } else {
                    ESP_LOGE(TAG, "Error creando timer de reconexión");
                    // Fallback: reconectar directamente después de 3 segundos
                    vTaskDelay(3000 / portTICK_PERIOD_MS);
                    mqtt_safe_reconnect();
                }
            } else {
                ESP_LOGI(TAG, "WiFi también desconectado, esperando reconexión WiFi primero...");
            }
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
    
    // Solo inicializar si hay WiFi
    if (!wifi_is_connected()) {
        ESP_LOGW(TAG, "⚠️ No hay conexión WiFi, omitiendo inicialización MQTT");
        return;
    }
    
    // Si ya existe un cliente, destruirlo primero
    if (mqtt_client != NULL) {
        ESP_LOGI(TAG, "Cliente MQTT existente encontrado, limpiando...");
        esp_mqtt_client_stop(mqtt_client);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
    }
    
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = THINGSBOARD_MQTT_URI,
        },
        .credentials = {
            .username = THINGSBOARD_ACCESS_TOKEN,
        },
        .network = {
            .disable_auto_reconnect = false,
            .reconnect_timeout_ms = 10000,
        },
        .session = {
            .keepalive = 60,
            .disable_clean_session = false,
        },
        .buffer = {
            .size = 2048,
            .out_size = 1024,
        },
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
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
    } else {
        ESP_LOGI(TAG, "✅ Cliente MQTT iniciado, esperando conexión...");
        mqtt_reconnect_attempts = 0;
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
            mqtt_conectado = false;  // Marcar como desconectado
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
        
        // Si hay WiFi pero no MQTT, intentar reconectar
        if (wifi_is_connected() && !mqtt_conectado) {
            static uint32_t last_attempt = 0;
            uint32_t now = (uint32_t)(esp_timer_get_time() / 1000000ULL);
            
            if (now - last_attempt > 10) {  // Solo cada 10 segundos
                ESP_LOGI(TAG, "Intentando reconectar MQTT...");
                last_attempt = now;
                mqtt_safe_reconnect();
            }
        }
    }
}

bool send_mqtt_telemetry_with_timestamp(const char *json_message) {
    if (mqtt_conectado && mqtt_client) {
        int msg_id = esp_mqtt_client_publish(mqtt_client, 
                                           "v1/devices/me/telemetry",
                                           json_message, 0, 1, 0);
        
        if (msg_id < 0) {
            ESP_LOGE(TAG, "❌ Error publicando telemetría con timestamp");
            mqtt_conectado = false;
            return false;
        }
        return true;
    }
    return false;
}

bool mqtt_is_connected(void) {
    return mqtt_conectado;
}

/**
 * @brief Verifica el estado de MQTT y reconecta si es necesario
 * 
 * Esta función debe llamarse periódicamente desde el loop principal
 */
/**
 * @brief Verifica el estado de MQTT y reconecta si es necesario
 */
void mqtt_check_and_reconnect(void) {
    static uint32_t last_check = 0;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    
    // Verificar solo cada 30 segundos
    if (now - last_check < 30) {
        return;
    }
    
    last_check = now;
    
    // Si hay WiFi pero no MQTT, intentar reconectar
    if (wifi_is_connected() && !mqtt_is_connected()) {
        ESP_LOGI(TAG, "🔍 Verificación periódica: WiFi OK pero MQTT desconectado");
        ESP_LOGI(TAG, "🔄 Intentando reconexión MQTT...");
        mqtt_safe_reconnect();
    }
}