#include "system_config.h"
#include <inttypes.h>

static const char *TAG = "DATA_BUFFER";

// =============================================================================
// VARIABLES PRIVADAS
// =============================================================================
static circular_buffer_t sensor_buffer = {0};
static bool buffer_initialized = false;

// =============================================================================
// FUNCIONES PRIVADAS (UTILIDAD)
// =============================================================================

static bool save_buffer_to_nvs(void) {
    if (sensor_buffer.count == 0) {
        return true;
    }
    
    nvs_handle_t handle;
    esp_err_t err;
    
    err = nvs_open("sensor_storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error abriendo NVS: %s", esp_err_to_name(err));
        return false;
    }
    
    err = nvs_set_u16(handle, "head", sensor_buffer.head);
    err |= nvs_set_u16(handle, "tail", sensor_buffer.tail);
    err |= nvs_set_u16(handle, "count", sensor_buffer.count);
    err |= nvs_set_u8(handle, "full", sensor_buffer.buffer_full ? 1 : 0);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error guardando metadata");
        nvs_close(handle);
        return false;
    }
    
    for (uint16_t i = 0; i < sensor_buffer.count; i++) {
        char key[16];
        snprintf(key, sizeof(key), "rd%04d", i);
        
        uint16_t idx = (sensor_buffer.tail + i) % MAX_BUFFER_SIZE;
        err = nvs_set_blob(handle, key, &sensor_buffer.readings[idx], sizeof(stored_reading_t));
        
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error guardando lectura %d: %s", i, esp_err_to_name(err));
            break;
        }
    }
    
    err = nvs_commit(handle);
    nvs_close(handle);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error en commit NVS: %s", esp_err_to_name(err));
        return false;
    }
    
    ESP_LOGD(TAG, "Guardado en NVS: %d lecturas", sensor_buffer.count);
    return true;
}

static bool load_buffer_from_nvs(void) {
    nvs_handle_t handle;
    esp_err_t err;
    
    err = nvs_open("sensor_storage", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No hay datos previos en NVS");
        return false;
    }
    
    uint16_t head, tail, count;
    uint8_t full;
    
    err = nvs_get_u16(handle, "head", &head);
    err |= nvs_get_u16(handle, "tail", &tail);
    err |= nvs_get_u16(handle, "count", &count);
    err |= nvs_get_u8(handle, "full", &full);
    
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Error cargando metadata: %s", esp_err_to_name(err));
        nvs_close(handle);
        return false;
    }
    
    if (count > MAX_BUFFER_SIZE) {
        ESP_LOGW(TAG, "Datos corruptos: count=%d > MAX=%d", count, MAX_BUFFER_SIZE);
        nvs_close(handle);
        return false;
    }
    
    sensor_buffer.head = head;
    sensor_buffer.tail = tail;
    sensor_buffer.count = count;
    sensor_buffer.buffer_full = (full == 1);
    
    for (uint16_t i = 0; i < count; i++) {
        char key[16];
        snprintf(key, sizeof(key), "rd%03d", i);
        
        size_t required_size = sizeof(stored_reading_t);
        uint16_t idx = (tail + i) % MAX_BUFFER_SIZE;
        
        err = nvs_get_blob(handle, key, &sensor_buffer.readings[idx], &required_size);
        if (err != ESP_OK || required_size != sizeof(stored_reading_t)) {
            ESP_LOGE(TAG, "Error cargando lectura %d", i);
            nvs_close(handle);
            return false;
        }
    }
    
    nvs_close(handle);
    ESP_LOGI(TAG, "Cargado desde NVS: %d lecturas", sensor_buffer.count);
    return true;
}

// =============================================================================
// FUNCIONES PÚBLICAS
// =============================================================================

void data_buffer_init(void) {
    ESP_LOGI(TAG, "Inicializando buffer circular...");
    
    memset(&sensor_buffer, 0, sizeof(circular_buffer_t));
    sensor_buffer.head = 0;
    sensor_buffer.tail = 0;
    sensor_buffer.count = 0;
    sensor_buffer.buffer_full = false;
    
    if (load_buffer_from_nvs()) {
        ESP_LOGI(TAG, "Datos previos cargados: %d lecturas", sensor_buffer.count);
    }
    
    buffer_initialized = true;
    ESP_LOGI(TAG, "Buffer listo. Capacidad: %d lecturas", MAX_BUFFER_SIZE);
}

bool data_buffer_store_reading(bme680_data_t *bme_data, float rainfall_mm, float wind_speed_ms) {
    if (!buffer_initialized) {
        ESP_LOGE(TAG, "Buffer no inicializado");
        return false;
    }
    
    if (!bme_data) {
        ESP_LOGE(TAG, "Datos BME680 nulos");
        return false;
    }
    
    // Validar datos antes de almacenar
    if (bme_data->temperature < -50.0 || bme_data->temperature > 100.0 ||
        bme_data->humidity < 0.0 || bme_data->humidity > 100.0 ||
        rainfall_mm < 0.0 || rainfall_mm > 1000.0 ||
        wind_speed_ms < 0.0 || wind_speed_ms > 100.0) {
        ESP_LOGW(TAG, "⚠️ Datos inválidos, no se almacenan");
        return false;
    }
    
    // Verificar si es igual a la última lectura
    if (sensor_buffer.count > 0) {
        uint16_t last_idx = (sensor_buffer.head == 0) ? MAX_BUFFER_SIZE - 1 : sensor_buffer.head - 1;
        stored_reading_t *last = &sensor_buffer.readings[last_idx];
        
        if (fabs(bme_data->temperature - last->temperature) < 0.1 &&
            fabs(bme_data->humidity - last->humidity) < 0.1 &&
            fabs(rainfall_mm - last->rainfall_mm) < 0.1 &&
            fabs(wind_speed_ms - last->wind_speed_ms) < 0.1) {
            ESP_LOGD(TAG, "Lectura similar a la anterior, no se almacena duplicado");
            return false;
        }
    }
    
    // Verificar si hay que hacer espacio
    if (sensor_buffer.buffer_full) {
        ESP_LOGW(TAG, "Buffer lleno, eliminando lectura más antigua");
        sensor_buffer.tail = (sensor_buffer.tail + 1) % MAX_BUFFER_SIZE;
        sensor_buffer.count--;
        sensor_buffer.buffer_full = false;
    }
    
    // Obtener timestamp ACTUAL
    time_t current_time = time_get_current();
    bool time_was_synced = time_is_synced();
    
    // Preparar nueva lectura CON TIMESTAMP
    stored_reading_t new_reading;
    
    // Datos del sensor
    new_reading.temperature = bme_data->temperature;
    new_reading.humidity = bme_data->humidity;
    new_reading.pressure = bme_data->pressure;
    new_reading.gas_resistance = bme_data->gas_resistance;
    new_reading.air_quality = bme_data->air_quality;
    new_reading.rainfall_mm = rainfall_mm;
    new_reading.wind_speed_ms = wind_speed_ms;
    
    // Timestamp y metadata
    new_reading.timestamp = current_time;
    new_reading.time_synced = time_was_synced;
    
    // Guardar en buffer
    uint16_t write_idx = sensor_buffer.head;
    sensor_buffer.readings[write_idx] = new_reading;
    sensor_buffer.head = (sensor_buffer.head + 1) % MAX_BUFFER_SIZE;
    sensor_buffer.count++;
    
    // Actualizar estado de lleno
    sensor_buffer.buffer_full = (sensor_buffer.count >= MAX_BUFFER_SIZE);
    
    // Verificar consistencia
    if (sensor_buffer.count > MAX_BUFFER_SIZE) {
        ESP_LOGE(TAG, "❌ ERROR: count (%d) > MAX_BUFFER_SIZE", sensor_buffer.count);
        sensor_buffer.count = MAX_BUFFER_SIZE;
        sensor_buffer.buffer_full = true;
    }
    
    // Convertir timestamp a string legible
    struct tm *timeinfo = localtime(&current_time);
    char time_str[32];
    strftime(time_str, sizeof(time_str), "%H:%M:%S", timeinfo);
    
    ESP_LOGI(TAG, "💾 Almacenado en idx=%d. Total: %d/%d", 
             write_idx, sensor_buffer.count, MAX_BUFFER_SIZE);
    
    // DEBUG: Mostrar datos almacenados
    ESP_LOGD(TAG, "  Hora: %s (%s)", time_str, time_was_synced ? "NTP" : "estimado");
    ESP_LOGD(TAG, "  Temp: %.2fC, Hum: %.2f%%, Pres: %.2fhPa", 
             new_reading.temperature, new_reading.humidity, new_reading.pressure);
    ESP_LOGD(TAG, "  Lluvia: %.2fmm, Viento: %.2fm/s", 
             new_reading.rainfall_mm, new_reading.wind_speed_ms);
    ESP_LOGD(TAG, "  Timestamp Unix: %ld", (long)new_reading.timestamp);
    
    // Guardar periódicamente en flash (cada 10 lecturas)
    static uint8_t save_counter = 0;
    if (++save_counter >= 10) {
        if (save_buffer_to_nvs()) {
            ESP_LOGD(TAG, "Guardado automático en NVS completado");
        }
        save_counter = 0;
    }
    
    return true;
}

bool data_buffer_send_stored_readings(void) {
    if (!buffer_initialized || sensor_buffer.count == 0) {
        ESP_LOGI(TAG, "📭 No hay lecturas almacenadas para enviar");
        return true;
    }
    
    ESP_LOGI(TAG, "═══════════════════════════════════════════════");
    ESP_LOGI(TAG, "📦 INICIANDO ENVÍO DE DATOS ALMACENADOS");
    ESP_LOGI(TAG, "   Lecturas pendientes: %d/%d", sensor_buffer.count, MAX_BUFFER_SIZE);
    ESP_LOGI(TAG, "═══════════════════════════════════════════════");
    
    uint16_t sent = 0;
    uint16_t failed = 0;
    uint16_t original_count = sensor_buffer.count;
    
    // Configuración conservadora para evitar rate limiting
    uint16_t max_per_batch = 2;           // Solo 2 por lote
    uint32_t delay_between_ms = 3000;     // 3 segundos entre envíos
    static uint32_t last_batch_time = 0;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    
    // Rate limiting: esperar 30 segundos entre lotes
    if (last_batch_time > 0 && (now - last_batch_time) < 30) {
        uint32_t wait_seconds = 30 - (now - last_batch_time);
        ESP_LOGW(TAG, "⏳ Rate limiting activo. Esperando %"PRIu32" segundos...", 
                 wait_seconds);
        return false;
    }
    
    while (sensor_buffer.count > 0 && sent < max_per_batch) {
        // Verificar conexión MQTT ANTES de cada envío
        if (!mqtt_is_connected()) {
            ESP_LOGW(TAG, "⚠️ MQTT desconectado, esperando reconexión...");
            vTaskDelay(3000 / portTICK_PERIOD_MS);  // Esperar 3 segundos
            
            if (!mqtt_is_connected()) {
                ESP_LOGW(TAG, "❌ MQTT sigue desconectado, abortando envío");
                break;
            }
            
            ESP_LOGI(TAG, "✅ MQTT reconectado, continuando...");
            vTaskDelay(1000 / portTICK_PERIOD_MS);  // Pequeña pausa extra
        }
        
        // Obtener lectura CON SU TIMESTAMP ORIGINAL
        stored_reading_t *reading = &sensor_buffer.readings[sensor_buffer.tail];
        
        // Convertir timestamp a string legible
        struct tm *timeinfo = localtime(&reading->timestamp);
        char time_str[32];
        char date_str[32];
        strftime(time_str, sizeof(time_str), "%H:%M:%S", timeinfo);
        strftime(date_str, sizeof(date_str), "%d/%m/%Y", timeinfo);
        
        // Mostrar datos COMPLETOS de la lectura
        ESP_LOGI(TAG, "═══════════════════════════════════════════════");
        ESP_LOGI(TAG, "📤 ENVIANDO LECTURA ALMACENADA");
        ESP_LOGI(TAG, "   Secuencia: %d/%d (total pendiente: %d)", 
                 sent + 1, max_per_batch, original_count);
        ESP_LOGI(TAG, "   Fecha: %s %s", date_str, time_str);
        ESP_LOGI(TAG, "   Hora sincronizada: %s", 
                 reading->time_synced ? "✅ SÍ (NTP)" : "⚠️ NO (estimada)");
        ESP_LOGI(TAG, "   Timestamp Unix: %ld segundos", (long)reading->timestamp);
        ESP_LOGI(TAG, "═══════════════════════════════════════════════");
        
        // Mostrar todos los valores de sensores
        ESP_LOGI(TAG, "🌡️  TEMPERATURA: %.2f °C", reading->temperature);
        ESP_LOGI(TAG, "💧 HUMEDAD: %.2f %%", reading->humidity);
        ESP_LOGI(TAG, "📊 PRESIÓN: %.2f hPa", reading->pressure);
        ESP_LOGI(TAG, "🌀 RESISTENCIA GAS: %lu Ω", 
                 (unsigned long)reading->gas_resistance);
        ESP_LOGI(TAG, "🌬️  CALIDAD AIRE: %.2f /100", reading->air_quality);
        ESP_LOGI(TAG, "🌧️  LLUVIA ACUMULADA: %.2f mm", reading->rainfall_mm);
        ESP_LOGI(TAG, "💨 VELOCIDAD VIENTO: %.2f m/s (%.2f km/h)", 
                 reading->wind_speed_ms, reading->wind_speed_ms * 3.6);
        
        ESP_LOGI(TAG, "═══════════════════════════════════════════════");
        
        // Crear mensaje JSON con timestamp REAL de la lectura
        char json_message[512];
        
        // ThingsBoard espera timestamp en MILISEGUNDOS
        uint64_t timestamp_ms = (uint64_t)reading->timestamp * 1000ULL;
        
        int len = snprintf(json_message, sizeof(json_message),
            "{\"ts\":%" PRIu64 ","
            "\"temperature\":%.2f,"
            "\"humidity\":%.2f,"
            "\"pressure\":%.2f,"
            "\"gas_resistance\":%" PRIu32 ","
            "\"air_quality\":%.2f,"
            "\"rainfall_mm\":%.2f,"
            "\"wind_speed_ms\":%.2f,"
            "\"wind_speed_kmh\":%.2f,"
            "\"stored\":true,"
            "\"time_synced\":%s,"
            "\"original_time\":\"%s\","
            "\"original_date\":\"%s\","
            "\"buffer_index\":%d,"
            "\"remaining_in_buffer\":%d}",
            timestamp_ms,
            reading->temperature,
            reading->humidity,
            reading->pressure,
            reading->gas_resistance,
            reading->air_quality,
            reading->rainfall_mm,
            reading->wind_speed_ms,
            reading->wind_speed_ms * 3.6,
            reading->time_synced ? "true" : "false",
            time_str,
            date_str,
            sensor_buffer.tail,
            sensor_buffer.count - 1);
        
        if (len >= sizeof(json_message)) {
            ESP_LOGW(TAG, "⚠️ Mensaje JSON demasiado largo, truncado");
            json_message[sizeof(json_message) - 1] = '\0';
        }
        
        // Mostrar JSON que se enviará
        ESP_LOGI(TAG, "📄 JSON a enviar (%d bytes):", strlen(json_message));
        ESP_LOGI(TAG, "%s", json_message);
        
        // Enviar con reintentos
        bool success = false;
        int max_attempts = 3;
        
        for (int attempt = 0; attempt < max_attempts && !success; attempt++) {
            if (attempt > 0) {
                uint32_t backoff_ms = 1000 * (1 << attempt); // 2s, 4s, etc.
                ESP_LOGW(TAG, "🔄 Reintento %d/%d en %"PRIu32" ms...", 
                        attempt + 1, max_attempts, backoff_ms);
                vTaskDelay(backoff_ms / portTICK_PERIOD_MS);
            }
            
            ESP_LOGI(TAG, "📡 Enviando a ThingsBoard (intento %d)...", attempt + 1);
            success = send_mqtt_telemetry_with_timestamp(json_message);
            
            if (success) {
                ESP_LOGI(TAG, "✅ Envío exitoso en intento %d", attempt + 1);
            } else {
                ESP_LOGE(TAG, "❌ Fallo en intento %d", attempt + 1);
            }
        }
        
        if (success) {
            sent++;
            
            // Mostrar confirmación
            ESP_LOGI(TAG, "═══════════════════════════════════════════════");
            ESP_LOGI(TAG, "🎯 LECTURA ENVIADA EXITOSAMENTE");
            ESP_LOGI(TAG, "   Enviada: %d de %d", sent, original_count);
            ESP_LOGI(TAG, "   Hora original: %s %s", date_str, time_str);
            ESP_LOGI(TAG, "   Siguiente índice: %d", 
                     (sensor_buffer.tail + 1) % MAX_BUFFER_SIZE);
            ESP_LOGI(TAG, "═══════════════════════════════════════════════");
            
            // Actualizar buffer
            sensor_buffer.tail = (sensor_buffer.tail + 1) % MAX_BUFFER_SIZE;
            sensor_buffer.count--;
            sensor_buffer.buffer_full = false;
            
            // ESPERAR entre envíos para no saturar MQTT
            if (sensor_buffer.count > 0 && sent < max_per_batch) {
                ESP_LOGI(TAG, "⏳ Esperando %"PRIu32" ms antes del siguiente envío...", 
                         delay_between_ms);
                vTaskDelay(delay_between_ms / portTICK_PERIOD_MS);
            }
        } else {
            failed++;
            ESP_LOGE(TAG, "═══════════════════════════════════════════════");
            ESP_LOGE(TAG, "❌ ERROR EN ENVÍO DE LECTURA");
            ESP_LOGE(TAG, "   Hora: %s %s", date_str, time_str);
            ESP_LOGE(TAG, "   Fallos consecutivos: %d", failed);
            ESP_LOGE(TAG, "═══════════════════════════════════════════════");
            
            // Backoff exponencial después de fallos
            uint32_t backoff_ms = 3000 * (1 << failed); // 3s, 6s, 12s...
            if (backoff_ms > 15000) backoff_ms = 15000; // Máximo 15 segundos
            
            ESP_LOGI(TAG, "⏳ Backoff de %"PRIu32" ms después del fallo...", backoff_ms);
            vTaskDelay(backoff_ms / portTICK_PERIOD_MS);
            
            // Si falla 3 veces seguidas, abortar
            if (failed >= 3) {
                ESP_LOGE(TAG, "⚠️ 3 fallos consecutivos, abortando envío");
                break;
            }
        }
    }
    
    // Guardar estado solo si se envió algo
    if (sent > 0) {
        if (save_buffer_to_nvs()) {
            ESP_LOGI(TAG, "💾 Estado del buffer guardado en NVS");
        }
        last_batch_time = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    }
    
    // Mostrar resumen final
    ESP_LOGI(TAG, "═══════════════════════════════════════════════");
    ESP_LOGI(TAG, "📊 RESUMEN FINAL ENVÍO DATOS ALMACENADOS");
    ESP_LOGI(TAG, "   Total original: %d lecturas", original_count);
    ESP_LOGI(TAG, "   ✅ Enviadas exitosamente: %d", sent);
    ESP_LOGI(TAG, "   ❌ Falladas: %d", failed);
    ESP_LOGI(TAG, "   📭 Restantes en buffer: %d", sensor_buffer.count);
    
    if (sensor_buffer.count > 0) {
        float percentage = (sensor_buffer.count * 100.0f) / MAX_BUFFER_SIZE;
        ESP_LOGI(TAG, "   📈 Estado buffer: %d/%d (%.1f%%)",
                 sensor_buffer.count, MAX_BUFFER_SIZE, percentage);
        
        if (percentage > 80.0f) {
            ESP_LOGW(TAG, "   ⚠️ Buffer casi lleno (>80%%)");
        }
    }
    
    if (sent > 0) {
        ESP_LOGI(TAG, "   ⏱️  Próximo lote disponible en: 30 segundos");
    }
    
    ESP_LOGI(TAG, "═══════════════════════════════════════════════");
    
    return (failed == 0);
}

uint16_t data_buffer_get_count(void) {
    return sensor_buffer.count;
}

bool data_buffer_is_full(void) {
    return sensor_buffer.buffer_full;
}

void data_buffer_clear(void) {
    ESP_LOGI(TAG, "Limpiando buffer de datos");
    
    sensor_buffer.head = 0;
    sensor_buffer.tail = 0;
    sensor_buffer.count = 0;
    sensor_buffer.buffer_full = false;
    
    nvs_handle_t handle;
    if (nvs_open("sensor_storage", NVS_READWRITE, &handle) == ESP_OK) {
        nvs_erase_all(handle);
        nvs_commit(handle);
        nvs_close(handle);
    }
}

void data_buffer_print_status(void) {
    ESP_LOGI(TAG, "=== ESTADO DEL BUFFER ===");
    ESP_LOGI(TAG, "Inicializado: %s", buffer_initialized ? "SÍ" : "NO");
    ESP_LOGI(TAG, "Capacidad: %d lecturas", MAX_BUFFER_SIZE);
    ESP_LOGI(TAG, "Almacenadas: %d (%.1f%%)", 
             sensor_buffer.count, 
             (sensor_buffer.count * 100.0f) / MAX_BUFFER_SIZE);
    ESP_LOGI(TAG, "Lleno: %s", sensor_buffer.buffer_full ? "SÍ" : "NO");
    ESP_LOGI(TAG, "Head: %d, Tail: %d", sensor_buffer.head, sensor_buffer.tail);
}