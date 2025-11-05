#include "system_config.h"

static const char *TAG = "WIFI_MANAGER";

static EventGroupHandle_t s_wifi_event_group;
static int wifi_conectado = 0;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

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
        
        if (retry_num < MAX_INTENTOS) {
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
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "✅ WiFi conectado! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_num = 0;
        if (s_wifi_event_group) {
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
    }
}

void wifi_init_sta(void) {
    ESP_LOGI(TAG, "🔌 Inicializando WiFi...");
    ESP_LOGI(TAG, "Conectando a: %s", WIFI_SSID);
    
    s_wifi_event_group = xEventGroupCreate();
    
    // Inicializar stack de red
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);
    
    // Configurar WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Registrar manejadores
    esp_event_handler_instance_t instance_any_id, instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler, NULL, &instance_got_ip));
    
    // Configurar credenciales
    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "Esperando conexión WiFi...");
    
    // Esperar conexión con timeout
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE, pdFALSE, pdMS_TO_TICKS(30000));
    
    // Limpiar
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    
    if (s_wifi_event_group) {
        vEventGroupDelete(s_wifi_event_group);
        s_wifi_event_group = NULL;
    }
    
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "✅ WiFi CONECTADO!");
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "❌ FALLO WiFi - Verifica SSID/Password");
    } else {
        ESP_LOGE(TAG, "⏰ TIMEOUT WiFi");
    }
}

bool wifi_is_connected(void) {
    return wifi_conectado;
}