#include "system_config.h"


static const char *TAG = "WEB_SERVER";
static httpd_handle_t web_server = NULL;

// =============================================================================
// FUNCIONES AUXILIARES PRIVADAS
// =============================================================================

/**
 * @brief Decodifica strings codificados en URL
 */
static void url_decode(const char* src, char* dst, size_t dst_size) {
    if (!src || !dst || dst_size == 0) return;
    
    size_t i = 0, j = 0;
    
    while (src[i] != '\0' && j < dst_size - 1) {
        if (src[i] == '+') {
            dst[j++] = ' ';
        } else if (src[i] == '%' && src[i+1] && src[i+2]) {
            char hex[3] = {src[i+1], src[i+2], '\0'};
            char *endptr;
            long int val = strtol(hex, &endptr, 16);
            
            if (*endptr == '\0') {
                dst[j++] = (char)val;
                i += 2;
            } else {
                dst[j++] = src[i];
            }
        } else {
            dst[j++] = src[i];
        }
        i++;
    }
    dst[j] = '\0';
}

/**
 * @brief Parsea datos application/x-www-form-urlencoded
 */
static bool parse_form_data(const char* data, size_t data_len, 
                           char* ssid, size_t ssid_size,
                           char* password, size_t pass_size) {
    if (!data || data_len == 0) return false;
    
    char *data_copy = malloc(data_len + 1);
    if (!data_copy) return false;
    
    memcpy(data_copy, data, data_len);
    data_copy[data_len] = '\0';
    
    ssid[0] = '\0';
    password[0] = '\0';
    
    char *token = strtok(data_copy, "&");
    while (token != NULL) {
        char *equals = strchr(token, '=');
        if (equals) {
            *equals = '\0';
            char *key = token;
            char *value = equals + 1;
            
            if (strcmp(key, "ssid") == 0) {
                url_decode(value, ssid, ssid_size);
            } else if (strcmp(key, "password") == 0) {
                url_decode(value, password, pass_size);
            }
        }
        token = strtok(NULL, "&");
    }
    
    free(data_copy);
    return (strlen(ssid) > 0);
}

/**
 * @brief Guarda credenciales WiFi en NVS
 */
static bool save_wifi_credentials(const char* ssid, const char* password) {
    if (!ssid || strlen(ssid) == 0) {
        ESP_LOGE(TAG, "SSID vacío, no se guarda");
        return false;
    }
    
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("wifi_config", NVS_READWRITE, &nvs_handle);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error abriendo NVS: %s", esp_err_to_name(err));
        return false;
    }
    
    err = nvs_set_str(nvs_handle, "ssid", ssid);
    if (err == ESP_OK) {
        err = nvs_set_str(nvs_handle, "password", password);
    }
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error guardando en NVS: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }
    
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error en commit NVS: %s", esp_err_to_name(err));
        return false;
    }
    
    ESP_LOGI(TAG, "✅ Credenciales guardadas: SSID='%s'", ssid);
    return true;
}

// =============================================================================
// HANDLERS HTTP
// =============================================================================

/**
 * @brief Handler para la página principal (/)
 */
static esp_err_t root_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "📄 Petición a página principal desde IP");
    
    // HTML MINIMALISTA para testing
    const char* minimal_html = 
        "<html><head><title>ESP32 Config</title></head>"
        "<body>"
        "<h1>ESP32 Config WiFi</h1>"
        "<p>AP: ESP32_E704</p>"
        "<form action='/save' method='post'>"
        "SSID: <input name='ssid'><br>"
        "Pass: <input type='password' name='password'><br>"
        "<input type='submit' value='Save'>"
        "</form>"
        "</body></html>";
    
    ESP_LOGI(TAG, "Enviando respuesta HTML (%d bytes)", strlen(minimal_html));
    httpd_resp_set_type(req, "text/html");
    esp_err_t ret = httpd_resp_send(req, minimal_html, strlen(minimal_html));
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error enviando respuesta: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Respuesta enviada OK");
    }
    
    return ret;
}

/**
 * @brief Handler para guardar configuración (/save)
 */
static esp_err_t save_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "📥 Recibiendo configuración WiFi...");
    
    // Validar método
    if (req->method != HTTP_POST) {
        ESP_LOGE(TAG, "Método no permitido: %d (esperaba POST)", req->method);
        httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Método no permitido");
        return ESP_FAIL;
    }
    
    // Validar contenido
    if (req->content_len <= 0) {
        ESP_LOGE(TAG, "Contenido vacío o sin Content-Length");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Sin datos en el formulario");
        return ESP_FAIL;
    }
    
    // Limitar tamaño máximo razonable (2KB debería ser suficiente para SSID + password)
    if (req->content_len > 2048) {
        ESP_LOGE(TAG, "Datos POST demasiado grandes: %d bytes (máx: 2048)", req->content_len);
        httpd_resp_send_err(req, HTTPD_431_REQ_HDR_FIELDS_TOO_LARGE, "Datos demasiado grandes");
        return ESP_FAIL;
    }
    
    char ssid[33] = {0};      // SSID máximo 32 chars + null terminator
    char password[65] = {0};  // Password máximo 64 chars + null terminator
    
    // Buffer dinámico para los datos POST
    char* content = malloc(req->content_len + 1);
    if (!content) {
        ESP_LOGE(TAG, "❌ Error de memoria al allocar %d bytes", req->content_len + 1);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Error interno del servidor");
        return ESP_FAIL;
    }
    
    
    // Leer todos los datos POST
    int total_received = 0;
    while (total_received < req->content_len) {
        int ret = httpd_req_recv(req, content + total_received, req->content_len - total_received);
        
        if (ret < 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                ESP_LOGE(TAG, "Timeout recibiendo datos POST");
                free(content);
                httpd_resp_send_408(req);
                return ESP_FAIL;
            }
            ESP_LOGE(TAG, "Error recibiendo datos POST: %d", ret);
            free(content);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Error recibiendo datos");
            return ESP_FAIL;
        }
        
        if (ret == 0) {
            // Conexión cerrada por cliente
            ESP_LOGE(TAG, "Conexión cerrada por cliente durante recepción");
            free(content);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Conexión interrumpida");
            return ESP_FAIL;
        }
        
        total_received += ret;
        ESP_LOGD(TAG, "Recibidos %d bytes (total: %d/%d)", ret, total_received, req->content_len);
    }
    
    // Terminar string
    content[total_received] = '\0';
    ESP_LOGD(TAG, "Datos POST completos (%d bytes): %s", total_received, content);
    
    // Parsear datos del formulario
    if (!parse_form_data(content, total_received, ssid, sizeof(ssid), password, sizeof(password))) {
        ESP_LOGE(TAG, "❌ Error parseando datos del formulario");
        free(content);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Datos del formulario inválidos");
        return ESP_FAIL;
    }
    
    free(content);  // Liberar buffer temporal
    
    // Validar SSID
    if (strlen(ssid) == 0) {
        ESP_LOGE(TAG, "SSID vacío");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SSID no puede estar vacío");
        return ESP_FAIL;
    }
    
    if (strlen(ssid) > 32) {
        ESP_LOGE(TAG, "SSID demasiado largo: %d caracteres", strlen(ssid));
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SSID demasiado largo (máx 32 chars)");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "📝 Configuración recibida - SSID: '%s', Password: '%s'", 
             ssid, strlen(password) > 0 ? "[PROTEGIDA]" : "(sin contraseña)");
    
    // Guardar en NVS
    if (!save_wifi_credentials(ssid, password)) {
        ESP_LOGE(TAG, "❌ Error guardando credenciales en NVS");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Error guardando configuración");
        return ESP_FAIL;
    }
    
    // Página de éxito
    char response[1024];
    const char* pass_display = strlen(password) > 0 ? "********" : "(sin contraseña)";
    
    snprintf(response, sizeof(response), 
             "<!DOCTYPE html>\n"
             "<html>\n"
             "<head>\n"
             "    <meta charset=\"UTF-8\">\n"
             "    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n"
             "    <meta http-equiv=\"refresh\" content=\"5;url=/\">\n"
             "    <title>Configuración Guardada</title>\n"
             "    <style>\n"
             "        body { font-family: Arial, sans-serif; text-align: center; padding: 50px; }\n"
             "        .success { color: green; font-size: 24px; }\n"
             "        .info { background: #f0f0f0; padding: 20px; margin: 20px; border-radius: 10px; }\n"
             "    </style>\n"
             "</head>\n"
             "<body>\n"
             "    <h1 class=\"success\">✅ Configuración Guardada</h1>\n"
             "    <div class=\"info\">\n"
             "        <p><strong>SSID:</strong> %s</p>\n"
             "        <p><strong>Contraseña:</strong> %s</p>\n"
             "    </div>\n"
             "    <p>El dispositivo se reiniciará en 5 segundos...</p>\n"
             "    <p>Intentará conectar a: <strong>%s</strong></p>\n"
             "    <p><em>Si falla, volverá a modo Access Point.</em></p>\n"
             "</body>\n"
             "</html>",
             ssid, pass_display, ssid);
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, response, strlen(response));
    
    // Programar reinicio (dar tiempo para que se envíe la respuesta)
    ESP_LOGI(TAG, "🔄 Programando reinicio en 3 segundos...");
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "🚀 Reiniciando para aplicar configuración WiFi...");
    esp_restart();
    
    return ESP_OK;
}
// =============================================================================
// FUNCIONES PÚBLICAS
// =============================================================================

void web_server_start(void) {
    if (web_server != NULL) {
        ESP_LOGW(TAG, "⚠️ Servidor web ya iniciado");
        return;
    }
    
    ESP_LOGI(TAG, "🌐 Iniciando servidor web de configuración...");
    
    // Configuración del servidor HTTP con buffers aumentados
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.ctrl_port = 32768;
    config.max_open_sockets = 4;          // Aumentado para múltiples clientes
    config.lru_purge_enable = true;
    config.stack_size = 12288;            // 12KB stack (crítico para HTML grande)
    config.max_uri_handlers = 15;         // Espacio para más handlers si es necesario
    config.recv_wait_timeout = 15;        // 15 segundos timeout recepción
    config.send_wait_timeout = 15;        // 15 segundos timeout envío
    config.global_user_ctx = NULL;
    config.global_user_ctx_free_fn = NULL;
    config.open_fn = NULL;
    config.close_fn = NULL;
    
    // En versiones más nuevas de IDF, estos campos podrían estar disponibles:
    #ifdef CONFIG_HTTPD_RECV_BUF_LEN
    config.recv_buf_len = 4096;           // Buffer de recepción 4KB
    #endif
    
    #ifdef CONFIG_HTTPD_SEND_BUF_LEN  
    config.send_buf_len = 8192;           // Buffer de envío 8KB
    #endif
    
    #ifdef CONFIG_HTTPD_TASK_PRIORITY
    config.task_priority = 5;             // Prioridad media
    #endif
    
    #ifdef CONFIG_HTTPD_TASK_AFFINITY
    config.core_id = tskNO_AFFINITY;      // Cualquier core
    #endif
    
    // Iniciar servidor
    esp_err_t ret = httpd_start(&web_server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Error iniciando servidor web: %s", esp_err_to_name(ret));
        return;
    }
    
    // Registrar handlers
    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_handler,
        .user_ctx = NULL
    };
    
    httpd_uri_t save_uri = {
        .uri = "/save",
        .method = HTTP_POST,
        .handler = save_handler,
        .user_ctx = NULL
    };
    
    // Registrar todos los handlers
    httpd_register_uri_handler(web_server, &root_uri);
    httpd_register_uri_handler(web_server, &save_uri);
    
    ESP_LOGI(TAG, "✅ Servidor web iniciado correctamente");
    ESP_LOGI(TAG, "   URL: http://192.168.4.1");
    ESP_LOGI(TAG, "   Stack size: %d bytes", config.stack_size);
    ESP_LOGI(TAG, "   Max sockets: %d", config.max_open_sockets);
}

void web_server_stop(void) {
    if (web_server != NULL) {
        ESP_LOGI(TAG, "🛑 Deteniendo servidor web...");
        httpd_stop(web_server);
        web_server = NULL;
        ESP_LOGI(TAG, "✅ Servidor web detenido");
    }
}

bool web_server_is_running(void) {
    return (web_server != NULL);
}

const char* web_server_get_ip(void) {
    return "192.168.4.1";
}