#include "system_config.h"

static const char *TAG = "MAIN_SYSTEM";

void app_main(void) {
    ESP_LOGI(TAG, "🚀 Iniciando Estación Meteorológica...");
    
    // 1. INICIALIZAR NVS (para buffer y WiFi)
    ESP_LOGI(TAG, "📁 Inicializando NVS...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // 2. INICIALIZAR BUFFER DE DATOS (¡PRIMERO!)
    ESP_LOGI(TAG, "💾 Inicializando buffer de datos...");
    data_buffer_init();
    ESP_LOGI(TAG, "Buffer disponible: %d/%d lecturas", 
             data_buffer_get_count(), MAX_BUFFER_SIZE);
    
    // 3. INICIALIZAR BME680
    ESP_LOGI(TAG, "🌡️ Inicializando sensor BME680...");
    bme680_init();
    if (bme680_configure_sensor() == ESP_OK) {
        ESP_LOGI(TAG, "✅ BME680 inicializado correctamente");
        
        // Lectura inicial de prueba
        bme680_data_t sensor_data;
        if (bme680_read_all_data(&sensor_data) == ESP_OK) {
            ESP_LOGI(TAG, "📊 Lectura inicial BME680:");
            ESP_LOGI(TAG, "  Temperatura: %.2f°C", sensor_data.temperature);
            ESP_LOGI(TAG, "  Humedad: %.1f%%", sensor_data.humidity);
            ESP_LOGI(TAG, "  Presión: %.2f hPa", sensor_data.pressure);
            ESP_LOGI(TAG, "  Gas: %lu Ω", (unsigned long)sensor_data.gas_resistance);
            ESP_LOGI(TAG, "  Calidad Aire: %.1f/100", sensor_data.air_quality);
            ESP_LOGI(TAG, "  Gas raw: %d", sensor_data.raw_gas);
        }
    } else {
        ESP_LOGE(TAG, "❌ Error inicializando BME680");
    }
    
    // 4. INICIALIZAR WIFI
    ESP_LOGI(TAG, "📡 Inicializando WiFi...");
    wifi_init_sta();
    
    // 5. INICIALIZAR SENSORES METEOROLÓGICOS
    ESP_LOGI(TAG, "🌧️ Inicializando sensores meteorológicos...");
    init_sensors();
    
    // 6. CONFIGURAR MQTT SI HAY WIFI
    if (wifi_is_connected()) {
        ESP_LOGI(TAG, "✅ Modo STA - Conectado a WiFi");
        mqtt_init();
    } else {
        ESP_LOGW(TAG, "📡 Modo AP - Servidor de configuración activo");
        ESP_LOGI(TAG, "   SSID: %s", wifi_get_ap_ssid());
        ESP_LOGI(TAG, "   Contraseña: %s", AP_PASSWORD);
        ESP_LOGI(TAG, "   IP: 192.168.4.1");
    }
    
    // 7. MOSTRAR ESTADO INICIAL COMPLETO
    ESP_LOGI(TAG, "=== ESTADO INICIAL DEL SISTEMA ===");
    ESP_LOGI(TAG, "WiFi: %s", wifi_is_connected() ? "CONECTADO" : "DESCONECTADO");
    ESP_LOGI(TAG, "Buffer: %d lecturas almacenadas", data_buffer_get_count());
    ESP_LOGI(TAG, "BME680: %s", bme680_is_connected() ? "CONECTADO" : "DESCONECTADO");
    ESP_LOGI(TAG, "==================================");
    
    // 8. LOOP PRINCIPAL
    int cycle_count = 0;
    bool was_connected = wifi_is_connected();  // Estado anterior de conexión
    
    while (1) {
        cycle_count++;
        
        // Determinar estado actual de conexión
        bool is_connected = wifi_is_connected() && mqtt_is_connected();
        
        // Leer sensores (siempre se hace, conectado o no)
        bme680_data_t bme_data;
        esp_err_t bme_result = bme680_read_all_data(&bme_data);
        float rainfall_mm = read_pluviometro_value();
        float wind_speed_ms = read_anemometro_value();
        
        if (is_connected) {
            // ========== MODO CONECTADO ==========
            ESP_LOGI(TAG, "=== CICLO %d (CONECTADO) ===", cycle_count);
            
            // Detectar reconexión (si antes estaba desconectado y ahora conectado)
            if (!was_connected) {
                ESP_LOGI(TAG, "🔄 ¡RECONEXIÓN DETECTADA!");
                
                // Enviar datos almacenados en buffer primero
                if (data_buffer_get_count() > 0) {
                    ESP_LOGI(TAG, "📤 Enviando %d lecturas almacenadas...", 
                             data_buffer_get_count());
                    
                    // Mostrar estado antes de enviar
                    data_buffer_print_status();
                    
                    // Enviar todas las lecturas almacenadas
                    bool send_success = data_buffer_send_stored_readings();
                    
                    if (send_success) {
                        ESP_LOGI(TAG, "✅ Datos almacenados enviados correctamente");
                    } else {
                        ESP_LOGW(TAG, "⚠️ Algunos datos no se pudieron enviar");
                    }
                } else {
                    ESP_LOGI(TAG, "✅ No hay datos almacenados pendientes");
                }
            }
            
            // Enviar lectura actual (además de las almacenadas)
            if (bme_result == ESP_OK) {
                send_mqtt_telemetry(&bme_data, rainfall_mm, wind_speed_ms);
                ESP_LOGI(TAG, "📤 Lectura actual enviada a la nube");
            } else {
                ESP_LOGE(TAG, "❌ Error leyendo BME680, no se envía lectura actual");
            }
            
        } else {
            // ========== MODO DESCONECTADO ==========
            ESP_LOGI(TAG, "=== CICLO %d (DESCONECTADO) ===", cycle_count);
            
            // Solo almacenar si la lectura del BME680 fue exitosa
            if (bme_result == ESP_OK) {
                // Almacenar lectura en buffer local
                bool stored = data_buffer_store_reading(&bme_data, rainfall_mm, wind_speed_ms);
                
                if (stored) {
                    ESP_LOGI(TAG, "💾 Almacenado en buffer local");
                    ESP_LOGI(TAG, "   Total almacenado: %d/%d lecturas", 
                             data_buffer_get_count(), MAX_BUFFER_SIZE);
                    
                    // Advertencia si el buffer está casi lleno
                    if (data_buffer_is_full()) {
                        ESP_LOGW(TAG, "⚠️ ¡BUFFER LLENO! Las lecturas más antiguas se sobrescribirán");
                    } else if (data_buffer_get_count() > (MAX_BUFFER_SIZE * 0.8)) {
                        ESP_LOGW(TAG, "⚠️ Buffer al 80%% de capacidad");
                    }
                } else {
                    ESP_LOGE(TAG, "❌ Error almacenando en buffer");
                }
            } else {
                ESP_LOGW(TAG, "⚠️ No se almacena: Error en lectura del sensor");
            }
            
            // Mostrar lectura local (solo para monitorización)
            ESP_LOGI(TAG, "📊 Lectura local:");
            ESP_LOGI(TAG, "   Temperatura: %.1f°C", bme_data.temperature);
            ESP_LOGI(TAG, "   Humedad: %.1f%%", bme_data.humidity);
            ESP_LOGI(TAG, "   Presión: %.1f hPa", bme_data.pressure);
            ESP_LOGI(TAG, "   Lluvia: %.2f mm", rainfall_mm);
            ESP_LOGI(TAG, "   Viento: %.1f m/s", wind_speed_ms);
        }
        
        // ========== TAREAS PERIÓDICAS ==========
        
        // Mostrar estado del buffer cada 10 ciclos
        if (cycle_count % 10 == 0) {
            ESP_LOGI(TAG, "--- INFORME PERIÓDICO [Ciclo %d] ---", cycle_count);
            ESP_LOGI(TAG, "Estado WiFi: %s", 
                     wifi_is_connected() ? "CONECTADO" : "DESCONECTADO");
            ESP_LOGI(TAG, "Estado MQTT: %s", 
                     mqtt_is_connected() ? "CONECTADO" : "DESCONECTADO");
            ESP_LOGI(TAG, "Lecturas en buffer: %d/%d", 
                     data_buffer_get_count(), MAX_BUFFER_SIZE);
            
            // Mostrar detalles del buffer si tiene datos
            if (data_buffer_get_count() > 0) {
                data_buffer_print_status();
            }
            
            // Verificar estado del BME680
            if (!bme680_is_connected()) {
                ESP_LOGW(TAG, "⚠️ Advertencia: BME680 no detectado");
            }
            
            ESP_LOGI(TAG, "--------------------------------------");
        }
        
        // Intentar reconexión WiFi si está desconectado (cada 30 ciclos)
        if (!wifi_is_connected() && (cycle_count % 30 == 0)) {
            ESP_LOGI(TAG, "🔄 Intentando reconexión WiFi...");
            // Nota: Tu sistema WiFi ya maneja reconexiones automáticas
        }
        
        // Guardar buffer en flash periódicamente (cada 20 ciclos)
        if (cycle_count % 20 == 0 && data_buffer_get_count() > 0) {
            ESP_LOGI(TAG, "💾 Guardando buffer en flash...");
            // El buffer ya se guarda automáticamente, pero podemos forzar un guardado
            // La función save_buffer_to_nvs() es privada, pero se llama automáticamente
        }
        
        // Actualizar estado anterior de conexión
        was_connected = is_connected;
        
        // ========== ESPERA ENTRE LECTURAS ==========
        
        // Mostrar tiempo restante antes de siguiente lectura
        int wait_seconds = 5;
        ESP_LOGI(TAG, "⏱️ Esperando %d segundos para próxima lectura...", wait_seconds);
        
        // Esperar entre lecturas (5 segundos por defecto)
        vTaskDelay(wait_seconds * 1000 / portTICK_PERIOD_MS);
    }
}

// =============================================================================
// FUNCIÓN DE EMERGENCIA (puede ser llamada desde otras partes si es necesario)
// =============================================================================
/*
void emergency_data_save(void) {
    // Esta función podría llamarse antes de un reinicio forzado
    ESP_LOGW(TAG, "⚠️ GUARDADO DE EMERGENCIA DE DATOS");
    
    if (data_buffer_get_count() > 0) {
        ESP_LOGI(TAG, "Guardando %d lecturas antes de reinicio...", 
                 data_buffer_get_count());
        // Forzar guardado en flash
        // Nota: Necesitarías hacer pública la función save_buffer_to_nvs()
        // o añadir una función pública para esto
    }
}
*/