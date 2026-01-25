#include "system_config.h"

static const char *TAG = "WIFI_MANAGER";

static EventGroupHandle_t s_wifi_event_group;
static int wifi_conectado = 0;
static bool credentials_failed = false;
static char ap_ssid[32];

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// =============================================================================
// CONFIGURACIÓN AP
// =============================================================================
#define AP_PASSWORD "config123"

// =============================================================================
// GENERAR SSID DEL AP
// =============================================================================
static void generate_ap_ssid(void) {
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_STA));
    snprintf(ap_ssid, sizeof(ap_ssid), "ESP32_%02X%02X", mac[4], mac[5]);
    ESP_LOGI(TAG, "SSID del AP: %s", ap_ssid);
}

// =============================================================================
// MANEJADOR DE EVENTOS WiFi (MEJORADO)
// =============================================================================
static void wifi_event_handler(void* arg, esp_event_base_t event_base, 
                              int32_t event_id, void* event_data) {
    static int retry_num = 0;
    
    ESP_LOGI(TAG, "Evento WiFi: %s, ID: %d", event_base, (int)event_id);
    
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi iniciado - Conectando...");
        esp_wifi_connect();
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_conectado = 0;
        
        wifi_event_sta_disconnected_t* disconnected = (wifi_event_sta_disconnected_t*) event_data;
        ESP_LOGE(TAG, "WiFi desconectado. Razon: %d", disconnected->reason);
        
        // Detectar fallo de credenciales
        if (disconnected->reason == WIFI_REASON_AUTH_EXPIRE || 
            disconnected->reason == WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT || 
            disconnected->reason == WIFI_REASON_AUTH_FAIL) {
            ESP_LOGW(TAG, "Credenciales incorrectas!");
            credentials_failed = true;
        }
        
        if (retry_num < MAX_INTENTOS && !credentials_failed) {
            esp_wifi_connect();
            retry_num++;
            ESP_LOGI(TAG, "Reintento WiFi %d/%d", retry_num, MAX_INTENTOS);
        } else {
            ESP_LOGE(TAG, "Fallo permanente WiFi");
            if (s_wifi_event_group) {
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            }
        }
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        wifi_conectado = 1;
        credentials_failed = false;
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "WiFi conectado! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_num = 0;
        if (s_wifi_event_group) {
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Dispositivo conectado al AP: " MACSTR, MAC2STR(event->mac));
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "Dispositivo desconectado del AP: " MACSTR, MAC2STR(event->mac));
    }
}

// =============================================================================
// INICIAR MODO ACCESS POINT
// =============================================================================
static void wifi_start_ap(void) {
    ESP_LOGI(TAG, "Iniciando Access Point...");
    
    // Detener WiFi actual
    esp_wifi_stop();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Configurar modo AP
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "",
            .ssid_len = 0,
            .channel = 1,
            .password = AP_PASSWORD,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .pmf_cfg = {
                .required = true,
            },
        },
    };
    
    strncpy((char*)wifi_config.ap.ssid, ap_ssid, sizeof(wifi_config.ap.ssid));
    wifi_config.ap.ssid_len = strlen(ap_ssid);
    
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "✅ AP iniciado: %s (Contraseña: %s)", ap_ssid, AP_PASSWORD);
    ESP_LOGI(TAG, "🌐 IP del AP: 192.168.4.1");
}
// =============================================================================
// FUNCIONES AUXILIARES (AGREGAR ANTES DE LOS HANDLERS)
// =============================================================================

/**
 * @brief Decodifica strings codificados en URL
 * @param src String codificado
 * @param dst Buffer de destino
 * @param dst_size Tamaño del buffer
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
 * @param data Datos a parsear
 * @param data_len Longitud de los datos
 * @param ssid Buffer para SSID
 * @param ssid_size Tamaño buffer SSID
 * @param password Buffer para password
 * @param pass_size Tamaño buffer password
 * @return true si se parseó correctamente
 */
static bool parse_form_data(const char* data, size_t data_len, 
                           char* ssid, size_t ssid_size,
                           char* password, size_t pass_size) {
    if (!data || data_len == 0) return false;
    
    // Crear copia para strtok
    char *data_copy = malloc(data_len + 1);
    if (!data_copy) return false;
    
    memcpy(data_copy, data, data_len);
    data_copy[data_len] = '\0';
    
    // Inicializar buffers
    ssid[0] = '\0';
    password[0] = '\0';
    
    // Parsear parámetros
    char *token = strtok(data_copy, "&");
    while (token != NULL) {
        char *equals = strchr(token, '=');
        if (equals) {
            *equals = '\0';  // Separar key=value
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

// =============================================================================
// HANDLER CORREGIDO PARA /save
// =============================================================================

static esp_err_t save_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Recibiendo configuración WiFi...");
    
    char ssid[33] = {0};        // SSID máximo 32 caracteres + null
    char password[65] = {0};    // Password máximo 64 caracteres + null
    
    // Buffer para recibir datos POST
    char content[256];
    int ret;
    
    // Leer datos del cuerpo de la solicitud
    int total_len = req->content_len;
    int cur_len = 0;
    
    if (total_len >= sizeof(content)) {
        ESP_LOGE(TAG, "Datos POST demasiado grandes: %d bytes", total_len);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Datos demasiado grandes");
        return ESP_FAIL;
    }
    
    while (cur_len < total_len) {
        ret = httpd_req_recv(req, content + cur_len, total_len - cur_len);
        if (ret <= 0) {
            ESP_LOGE(TAG, "Error recibiendo datos POST");
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Error recibiendo datos");
            return ESP_FAIL;
        }
        cur_len += ret;
    }
    content[cur_len] = '\0';
    
    ESP_LOGD(TAG, "Datos POST recibidos (%d bytes): %s", cur_len, content);
    
    // Parsear datos del formulario
    if (!parse_form_data(content, cur_len, ssid, sizeof(ssid), password, sizeof(password))) {
        ESP_LOGE(TAG, "Error parseando datos del formulario");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Datos inválidos");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Configuración recibida: SSID='%s', PASS='%s'", ssid, password);
    
    // Validar SSID
    if (strlen(ssid) == 0) {
        ESP_LOGE(TAG, "SSID vacío");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SSID no puede estar vacío");
        return ESP_FAIL;
    }
    
    // Guardar en NVS
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("wifi_config", NVS_READWRITE, &nvs_handle);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error abriendo NVS: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Error interno");
        return ESP_FAIL;
    }
    
    // Guardar credenciales
    err = nvs_set_str(nvs_handle, "ssid", ssid);
    if (err == ESP_OK) {
        err = nvs_set_str(nvs_handle, "password", password);
    }
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error guardando en NVS: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Error guardando configuración");
        return ESP_FAIL;
    }
    
    // Commit y cerrar
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error en commit NVS: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Error guardando configuración");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Configuración WiFi guardada exitosamente en NVS");
    
    // Página de respuesta HTML
    static const char* success_page = 
"<!DOCTYPE html>\n"
"<html>\n"
"<head>\n"
"    <meta charset=\"UTF-8\">\n"
"    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n"
"    <title>Configuración Estación Meteorológica</title>\n"
"    <style>\n"
"        body {\n"
"            font-family: Arial, sans-serif;\n"
"            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);\n"
"            margin: 0;\n"
"            padding: 20px;\n"
"            min-height: 100vh;\n"
"            display: flex;\n"
"            justify-content: center;\n"
"            align-items: center;\n"
"        }\n"
"        \n"
"        .container {\n"
"            background: white;\n"
"            padding: 40px;\n"
"            border-radius: 15px;\n"
"            box-shadow: 0 20px 60px rgba(0,0,0,0.3);\n"
"            width: 100%;\n"
"            max-width: 400px;\n"
"        }\n"
"        \n"
"        h1 {\n"
"            color: #333;\n"
"            text-align: center;\n"
"            margin-bottom: 30px;\n"
"            font-size: 24px;\n"
"        }\n"
"        \n"
"        .status {\n"
"            background: #f8f9fa;\n"
"            padding: 15px;\n"
"            border-radius: 8px;\n"
"            margin-bottom: 20px;\n"
"            text-align: center;\n"
"            border-left: 4px solid #667eea;\n"
"        }\n"
"        \n"
"        .status h3 {\n"
"            margin: 0 0 10px 0;\n"
"            color: #555;\n"
"        }\n"
"        \n"
"        .form-group {\n"
"            margin-bottom: 20px;\n"
"        }\n"
"        \n"
"        label {\n"
"            display: block;\n"
"            margin-bottom: 8px;\n"
"            color: #555;\n"
"            font-weight: bold;\n"
"        }\n"
"        \n"
"        input[type=\"text\"],\n"
"        input[type=\"password\"] {\n"
"            width: 100%;\n"
"            padding: 12px;\n"
"            border: 2px solid #e0e0e0;\n"
"            border-radius: 8px;\n"
"            font-size: 16px;\n"
"            transition: border 0.3s;\n"
"            box-sizing: border-box;\n"
"        }\n"
"        \n"
"        input[type=\"text\"]:focus,\n"
"        input[type=\"password\"]:focus {\n"
"            border-color: #667eea;\n"
"            outline: none;\n"
"        }\n"
"        \n"
"        .btn {\n"
"            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);\n"
"            color: white;\n"
"            border: none;\n"
"            padding: 15px;\n"
"            border-radius: 8px;\n"
"            font-size: 16px;\n"
"            font-weight: bold;\n"
"            cursor: pointer;\n"
"            width: 100%;\n"
"            transition: transform 0.2s;\n"
"        }\n"
"        \n"
"        .btn:hover {\n"
"            transform: translateY(-2px);\n"
"        }\n"
"        \n"
"        .btn:active {\n"
"            transform: translateY(0);\n"
"        }\n"
"        \n"
"        .info {\n"
"            margin-top: 20px;\n"
"            padding: 15px;\n"
"            background: #e8f4fd;\n"
"            border-radius: 8px;\n"
"            border-left: 4px solid #2196F3;\n"
"        }\n"
"        \n"
"        .info p {\n"
"            margin: 5px 0;\n"
"            font-size: 14px;\n"
"            color: #555;\n"
"        }\n"
"    </style>\n"
"</head>\n"
"<body>\n"
"    <div class=\"container\">\n"
"        <h1>🌤️ Estación Meteorológica ESP32</h1>\n"
"        \n"
"        <div class=\"status\">\n"
"            <h3>📡 Modo: Access Point</h3>\n"
"            <p>Conectado a: <strong>%s</strong></p>\n"
"            <p>IP: <strong>192.168.4.1</strong></p>\n"
"        </div>\n"
"        \n"
"        <form action=\"/save\" method=\"post\" onsubmit=\"return validateForm()\">\n"
"            <div class=\"form-group\">\n"
"                <label for=\"ssid\">📶 Nombre de red WiFi (SSID):</label>\n"
"                <input type=\"text\" id=\"ssid\" name=\"ssid\" \n"
"                       placeholder=\"Ej: MiCasa_WiFi\" required>\n"
"            </div>\n"
"            \n"
"            <div class=\"form-group\">\n"
"                <label for=\"password\">🔑 Contraseña WiFi:</label>\n"
"                <input type=\"password\" id=\"password\" name=\"password\" \n"
"                       placeholder=\"Opcional para redes abiertas\">\n"
"            </div>\n"
"            \n"
"            <button type=\"submit\" class=\"btn\">💾 Guardar y Conectar</button>\n"
"        </form>\n"
"        \n"
"        <div class=\"info\">\n"
"            <p><strong>⚠️ Importante:</strong></p>\n"
"            <p>1. El ESP32 se reiniciará después de guardar</p>\n"
"            <p>2. Se intentará conectar a la red WiFi especificada</p>\n"
"            <p>3. Si falla, volverá a modo Access Point</p>\n"
"        </div>\n"
"    </div>\n"
"    \n"
"    <script>\n"
"        function validateForm() {\n"
"            const ssid = document.getElementById('ssid').value;\n"
"            if (!ssid.trim()) {\n"
"                alert('Por favor, introduce el nombre de la red WiFi');\n"
"                return false;\n"
"            }\n"
"            return true;\n"
"        }\n"
"        \n"
"        // Mostrar SSID actual en consola\n"
"        console.log('Configuración WiFi - ESP32');\n"
"    </script>\n"
"</body>\n"
"</html>\n";
    
    // Generar respuesta dinámica
    char response[1024];
    snprintf(response, sizeof(response), success_page, ssid, 
             strlen(password) > 0 ? "********" : "(sin contraseña)");
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, response, strlen(response));
    
    // Reiniciar después de dar tiempo para enviar la respuesta
    ESP_LOGI(TAG, "Programando reinicio en 2 segundos...");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Reiniciando para aplicar configuración WiFi...");
    esp_restart();
    
    return ESP_OK;
}

// =============================================================================
// CONECTAR A RED WiFi (STA)
// =============================================================================
static void wifi_connect_sta(void) {
    ESP_LOGI(TAG, "🔌 Conectando a: %s", WIFI_SSID);
    
    // Configurar modo STA
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    
    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
}

// =============================================================================
// INICIALIZACIÓN PRINCIPAL WiFi
// =============================================================================
void wifi_init_sta(void) {
    ESP_LOGI(TAG, "🔌 Inicializando WiFi Manager...");
    
    s_wifi_event_group = xEventGroupCreate();
    
    // Inicializar stack de red
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Crear interfaces para ambos modos
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    assert(sta_netif && ap_netif);
    
    // Configurar WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Registrar manejadores de eventos
    esp_event_handler_instance_t instance_any_id, instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler, NULL, &instance_got_ip));
    
    // Generar SSID del AP
    generate_ap_ssid();
    
    // Intentar conexión STA primero
    wifi_connect_sta();
    
    ESP_LOGI(TAG, "⏳ Esperando conexión WiFi...");
    
    // Esperar conexión con timeout
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE, pdFALSE, pdMS_TO_TICKS(30000));
    
    // Limpiar manejadores de eventos
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    
    if (s_wifi_event_group) {
        vEventGroupDelete(s_wifi_event_group);
        s_wifi_event_group = NULL;
    }
    
    // Decidir modo de operación
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi CONECTADO! Modo STA activo");
    } else if (bits & WIFI_FAIL_BIT || credentials_failed) {
        ESP_LOGW(TAG, "Iniciando modo Access Point...");
        if (credentials_failed) {
            ESP_LOGE(TAG, "Razón: Credenciales incorrectas");
        } else {
            ESP_LOGE(TAG, "Razón: Timeout de conexión");
        }
        wifi_start_ap();
    } else {
        ESP_LOGE(TAG, "TIMEOUT WiFi - Iniciando modo AP");
        wifi_start_ap();
    }
}

// =============================================================================
// FUNCIONES PÚBLICAS
// =============================================================================
bool wifi_is_connected(void) {
    return wifi_conectado;
}

bool wifi_is_ap_mode(void) {
    wifi_mode_t mode;
    esp_wifi_get_mode(&mode);
    return (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA);
}

const char* wifi_get_ap_ssid(void) {
    return ap_ssid;
}