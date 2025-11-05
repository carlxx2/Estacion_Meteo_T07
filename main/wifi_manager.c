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
        ESP_LOGI(TAG, "📶 WiFi iniciado - Conectando...");
        esp_wifi_connect();
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_conectado = 0;
        
        wifi_event_sta_disconnected_t* disconnected = (wifi_event_sta_disconnected_t*) event_data;
        ESP_LOGE(TAG, "📶 WiFi desconectado. Razón: %d", disconnected->reason);
        
        // Detectar fallo de credenciales
        if (disconnected->reason == WIFI_REASON_AUTH_EXPIRE || 
            disconnected->reason == WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT || 
            disconnected->reason == WIFI_REASON_AUTH_FAIL) {
            ESP_LOGW(TAG, "🔐 Credenciales incorrectas!");
            credentials_failed = true;
        }
        
        if (retry_num < MAX_INTENTOS && !credentials_failed) {
            esp_wifi_connect();
            retry_num++;
            ESP_LOGI(TAG, "🔄 Reintento WiFi %d/%d", retry_num, MAX_INTENTOS);
        } else {
            ESP_LOGE(TAG, "❌ Fallo permanente WiFi");
            if (s_wifi_event_group) {
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            }
        }
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        wifi_conectado = 1;
        credentials_failed = false;
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "✅ WiFi conectado! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_num = 0;
        if (s_wifi_event_group) {
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "📱 Dispositivo conectado al AP: " MACSTR, MAC2STR(event->mac));
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "📱 Dispositivo desconectado del AP: " MACSTR, MAC2STR(event->mac));
    }
}

// =============================================================================
// INICIAR MODO ACCESS POINT
// =============================================================================
static void wifi_start_ap(void) {
    ESP_LOGI(TAG, "📡 Iniciando Access Point...");
    
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
            .authmode = WIFI_AUTH_WPA2_PSK,
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
        ESP_LOGI(TAG, "✅ WiFi CONECTADO! Modo STA activo");
    } else if (bits & WIFI_FAIL_BIT || credentials_failed) {
        ESP_LOGW(TAG, "🔄 Iniciando modo Access Point...");
        if (credentials_failed) {
            ESP_LOGE(TAG, "❌ Razón: Credenciales incorrectas");
        } else {
            ESP_LOGE(TAG, "❌ Razón: Timeout de conexión");
        }
        wifi_start_ap();
    } else {
        ESP_LOGE(TAG, "⏰ TIMEOUT WiFi - Iniciando modo AP");
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