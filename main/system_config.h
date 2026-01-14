// =============================================================================
// CONFIGURACIÓN - ¡ACTUALIZA CON TUS DATOS REALES!
// =============================================================================
#define WIFI_SSID      "SBC"
#define WIFI_PASS      "SBCwifi$"      
#define MAX_INTENTOS   10
#define AP_PASSWORD    "config123"
#define THINGSBOARD_MQTT_URI "mqtt://demo.thingsboard.io"
#define THINGSBOARD_ACCESS_TOKEN "om8tccnqiv43ukzaprgp"
#define GITHUB_FIRMWARE_URL "https://raw.githubusercontent.com/carlxx2/Estacion_Meteo_T07/main/firmware/firmware.bin"

// ==================== CONFIGURACIÓN ADC ====================
#define PLUVIOMETRO_ADC_CHANNEL    ADC1_CHANNEL_4   // GPIO32
#define ANEMOMETRO_ADC_CHANNEL     ADC1_CHANNEL_5   // GPIO33

// =============================================================================
// CONFIGURACIÓN BME680 - MEJORADA
// =============================================================================
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000
#define BME680_ADDR 0x77

// Calibración lineal (ajusta según tus mediciones de referencia)
#define BME680_TEMP_SCALE 1.0f
#define BME680_TEMP_OFFSET_C 0.0f
#define BME680_HUM_SCALE 1.0f
#define BME680_HUM_OFFSET_PCT 0.0f

// Modos de operación (de gschorcht)
typedef enum {
    BME680_SLEEP_MODE = 0,
    BME680_FORCED_MODE,
    BME680_PARALLEL_MODE,
    BME680_NORMAL_MODE = 3
} bme680_mode_t;

// Oversampling rates (de gschorcht)
typedef enum {
    BME680_OSR_NONE = 0,
    BME680_OSR_1X,
    BME680_OSR_2X,
    BME680_OSR_4X,
    BME680_OSR_8X,
    BME680_OSR_16X
} bme680_osr_t;

// =============================================================================
// ESTRUCTURAS DE DATOS - MEJORADAS
// =============================================================================

// Estructura para datos del sensor BME680 (manteniendo tu formato)
typedef struct {
    float temperature;
@@ -118,26 +124,26 @@ bool mqtt_is_connected(void);
// OTA Updater (sin cambios)
void check_ota_updates(void);
void verify_github_url(void);

// Sensor Reader (actualizada sin LDR)
void init_sensors(void);
float read_pluviometro_value(void);
float read_anemometro_value(void);

// BME680 Sensor - FUNCIONES MEJORADAS
void bme680_init(void);
esp_err_t bme680_read_all_data(bme680_data_t *sensor_data);
bool bme680_is_connected(void);
void bme680_start_reading_task(void);
esp_err_t bme680_configure_sensor(void);
void i2c_diagnostic(void);

// NUEVAS FUNCIONES DE gschorcht INTEGRADAS
esp_err_t bme680_set_oversampling_rates(bme680_osr_t osr_t, bme680_osr_t osr_p, bme680_osr_t osr_h);
esp_err_t bme680_set_heater_profile(uint16_t temperature, uint16_t duration);
esp_err_t bme680_set_operation_mode(bme680_mode_t mode);
esp_err_t bme680_set_ambient_temperature(int16_t temperature);
bme680_mode_t bme680_get_operation_mode(void);
esp_err_t bme680_apply_config(bme680_config_t *config);
