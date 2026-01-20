#include "system_config.h"


static const char *TAG = "DATA_BUFFER";

// =============================================================================
// VARIABLES PRIVADAS
// =============================================================================
static circular_buffer_t sensor_buffer = {0};
static bool buffer_initialized = false;

// =============================================================================
// FUNCIONES PRIVADAS (UTILIDAD)
// =============================================================================

/**
 * @brief Guarda el buffer actual en memoria flash NVS
 * @return true si se guardó correctamente, false si hubo error
 */
static bool save_buffer_to_nvs(void) {
    if (sensor_buffer.count == 0) {
        return true;  // No hay nada que guardar
    }
    
    nvs_handle_t handle;
    esp_err_t err;
    
    // Abrir namespace NVS
    err = nvs_open("sensor_storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error abriendo NVS: %s", esp_err_to_name(err));
        return false;
    }
    
    // Guardar metadata
    err = nvs_set_u16(handle, "head", sensor_buffer.head);
    err |= nvs_set_u16(handle, "tail", sensor_buffer.tail);
    err |= nvs_set_u16(handle, "count", sensor_buffer.count);
    err |= nvs_set_u8(handle, "full", sensor_buffer.buffer_full ? 1 : 0);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error guardando metadata");
        nvs_close(handle);
        return false;
    }
    
    // Guardar cada lectura
    for (uint16_t i = 0; i < sensor_buffer.count; i++) {
        char key[16];
        snprintf(key, sizeof(key), "rd%03d", i);
        
        uint16_t idx = (sensor_buffer.tail + i) % MAX_BUFFER_SIZE;
        err = nvs_set_blob(handle, key, &sensor_buffer.readings[idx], sizeof(stored_reading_t));
        
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error guardando lectura %d", i);
            break;
        }
    }
    
    // Commit y cerrar
    err = nvs_commit(handle);
    nvs_close(handle);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error en commit: %s", esp_err_to_name(err));
        return false;
    }
    
    ESP_LOGD(TAG, "Guardado en NVS: %d lecturas", sensor_buffer.count);
    return true;
}

/**
 * @brief Carga el buffer desde memoria flash NVS
 * @return true si se cargó correctamente, false si no hay datos o error
 */
static bool load_buffer_from_nvs(void) {
    nvs_handle_t handle;
    esp_err_t err;
    
    // Abrir namespace NVS
    err = nvs_open("sensor_storage", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No hay datos previos en NVS");
        return false;
    }
    
    // Cargar metadata
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
    
    // Validar datos
    if (count > MAX_BUFFER_SIZE) {
        ESP_LOGW(TAG, "Datos corruptos: count=%d > MAX=%d", count, MAX_BUFFER_SIZE);
        nvs_close(handle);
        return false;
    }
    
    // Restaurar metadata
    sensor_buffer.head = head;
    sensor_buffer.tail = tail;
    sensor_buffer.count = count;
    sensor_buffer.buffer_full = (full == 1);
    
    // Cargar lecturas
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
// FUNCIONES PÚBLICAS (DECLARADAS EN system_config.h)
// =============================================================================

// 1. INICIALIZACIÓN
void data_buffer_init(void) {
    ESP_LOGI(TAG, "Inicializando buffer circular...");
    
    // Inicializar estructura
    memset(&sensor_buffer, 0, sizeof(circular_buffer_t));
    sensor_buffer.head = 0;
    sensor_buffer.tail = 0;
    sensor_buffer.count = 0;
    sensor_buffer.buffer_full = false;
    
    // Cargar datos guardados
    if (load_buffer_from_nvs()) {
        ESP_LOGI(TAG, "Datos previos cargados: %d lecturas", sensor_buffer.count);
    }
    
    buffer_initialized = true;
    ESP_LOGI(TAG, "Buffer listo. Capacidad: %d lecturas", MAX_BUFFER_SIZE);
}

// 2. ALMACENAR LECTURA
bool data_buffer_store_reading(bme680_data_t *bme_data, float rainfall_mm, float wind_speed_ms) {
    if (!buffer_initialized) {
        ESP_LOGE(TAG, "Buffer no inicializado");
        return false;
    }
    
    if (!bme_data) {
        ESP_LOGE(TAG, "Datos BME680 nulos");
        return false;
    }
    
    // Verificar si hay que hacer espacio
    if (sensor_buffer.buffer_full) {
        ESP_LOGW(TAG, "Buffer lleno, eliminando lectura más antigua");
        sensor_buffer.tail = (sensor_buffer.tail + 1) % MAX_BUFFER_SIZE;
        sensor_buffer.count--;
        sensor_buffer.buffer_full = false;
    }
    
    // Preparar nueva lectura
    stored_reading_t new_reading;
    new_reading.temperature = bme_data->temperature;
    new_reading.humidity = bme_data->humidity;
    new_reading.pressure = bme_data->pressure;
    new_reading.gas_resistance = bme_data->gas_resistance;
    new_reading.air_quality = bme_data->air_quality;
    new_reading.rainfall_mm = rainfall_mm;
    new_reading.wind_speed_ms = wind_speed_ms;
    new_reading.timestamp = (uint32_t)(esp_timer_get_time() / 1000000ULL); // Segundos desde boot
    
    // Guardar en buffer
    sensor_buffer.readings[sensor_buffer.head] = new_reading;
    sensor_buffer.head = (sensor_buffer.head + 1) % MAX_BUFFER_SIZE;
    sensor_buffer.count++;
    
    // Actualizar estado de lleno
    sensor_buffer.buffer_full = (sensor_buffer.count >= MAX_BUFFER_SIZE);
    
    // Guardar periódicamente en flash (cada 5 lecturas)
    static uint8_t save_counter = 0;
    if (++save_counter >= 5) {
        save_buffer_to_nvs();
        save_counter = 0;
    }
    
    ESP_LOGD(TAG, "Lectura almacenada. Total: %d/%d", sensor_buffer.count, MAX_BUFFER_SIZE);
    return true;
}

// 3. ENVIAR LECTURAS ALMACENADAS
bool data_buffer_send_stored_readings(void) {
    if (!buffer_initialized || sensor_buffer.count == 0) {
        ESP_LOGI(TAG, "No hay lecturas almacenadas");
        return false;
    }
    
    ESP_LOGI(TAG, "Enviando %d lecturas almacenadas...", sensor_buffer.count);
    
    uint16_t sent = 0;
    uint16_t failed = 0;
    
    while (sensor_buffer.count > 0) {
        // Obtener lectura más antigua
        stored_reading_t *reading = &sensor_buffer.readings[sensor_buffer.tail];
        
        // Crear mensaje JSON
        char json_message[512];
        snprintf(json_message, sizeof(json_message),
            "{\"timestamp\":%lu,"
            "\"temperature\":%.2f,"
            "\"humidity\":%.2f,"
            "\"pressure\":%.2f,"
            "\"gas_resistance\":%lu,"
            "\"air_quality\":%.2f,"
            "\"rainfall_mm\":%.2f,"
            "\"wind_speed_ms\":%.2f,"
            "\"wind_speed_kmh\":%.2f,"
            "\"stored\":true}",
            (unsigned long)reading->timestamp,
            reading->temperature,
            reading->humidity,
            reading->pressure,
            (unsigned long)reading->gas_resistance,
            reading->air_quality,
            reading->rainfall_mm,
            reading->wind_speed_ms,
            reading->wind_speed_ms * 3.6);
        
        // Enviar por MQTT (necesitas implementar esta función)
        bool success = send_mqtt_telemetry_with_timestamp(json_message);
        
        if (success) {
            sent++;
            // Eliminar del buffer
            sensor_buffer.tail = (sensor_buffer.tail + 1) % MAX_BUFFER_SIZE;
            sensor_buffer.count--;
            sensor_buffer.buffer_full = false;
        } else {
            failed++;
            ESP_LOGE(TAG, "Error enviando lectura, abortando");
            break;
        }
        
        // Pequeña pausa
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    // Guardar estado actual
    save_buffer_to_nvs();
    
    ESP_LOGI(TAG, "Envío completado: %d enviadas, %d falladas, %d restantes", 
             sent, failed, sensor_buffer.count);
    
    return (failed == 0);
}

// 4. CONSULTAS
uint16_t data_buffer_get_count(void) {
    return sensor_buffer.count;
}

bool data_buffer_is_full(void) {
    return sensor_buffer.buffer_full;
}

// 5. LIMPIAR BUFFER
void data_buffer_clear(void) {
    ESP_LOGI(TAG, "Limpiando buffer de datos");
    
    sensor_buffer.head = 0;
    sensor_buffer.tail = 0;
    sensor_buffer.count = 0;
    sensor_buffer.buffer_full = false;
    
    // También limpiar NVS
    nvs_handle_t handle;
    if (nvs_open("sensor_storage", NVS_READWRITE, &handle) == ESP_OK) {
        nvs_erase_all(handle);
        nvs_commit(handle);
        nvs_close(handle);
    }
}

// 6. DIAGNÓSTICO
void data_buffer_print_status(void) {
    ESP_LOGI(TAG, "=== ESTADO DEL BUFFER ===");
    ESP_LOGI(TAG, "Inicializado: %s", buffer_initialized ? "SÍ" : "NO");
    ESP_LOGI(TAG, "Capacidad: %d lecturas", MAX_BUFFER_SIZE);
    ESP_LOGI(TAG, "Almacenadas: %d (%.1f%%)", 
             sensor_buffer.count, 
             (sensor_buffer.count * 100.0f) / MAX_BUFFER_SIZE);
    ESP_LOGI(TAG, "Lleno: %s", sensor_buffer.buffer_full ? "SÍ" : "NO");
    ESP_LOGI(TAG, "Head: %d, Tail: %d", sensor_buffer.head, sensor_buffer.tail);
    
    if (sensor_buffer.count > 0) {
        ESP_LOGI(TAG, "=== ÚLTIMAS LECTURAS ===");
        uint8_t to_show = (sensor_buffer.count < 3) ? sensor_buffer.count : 3;
        
        for (uint8_t i = 0; i < to_show; i++) {
            uint16_t idx = (sensor_buffer.tail + i) % MAX_BUFFER_SIZE;
            stored_reading_t *r = &sensor_buffer.readings[idx];
            
            ESP_LOGI(TAG, "[%d] T:%.1f°C H:%.1f%% P:%.1fhPa Lluvia:%.2fmm",
                     i + 1,
                     r->temperature,
                     r->humidity,
                     r->pressure,
                     r->rainfall_mm);
        }
    }
}