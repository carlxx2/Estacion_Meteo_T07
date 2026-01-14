#include "system_config.h"
#include <math.h>

static const char *TAG = "BME680_SENSOR";

// ==================== ESTRUCTURAS MEJORADAS ====================

// Estructura de calibración
typedef struct {
    uint16_t par_t1;
    int16_t par_t2;
    int8_t par_t3;
    
    uint16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int16_t par_p4;
    int16_t par_p5;
    int8_t par_p6;
    int8_t par_p7;
    int16_t par_p8;
    int16_t par_p9;
    uint8_t par_p10;
    
    uint16_t par_h1;
    uint16_t par_h2;
    int8_t par_h3;
    int8_t par_h4;
    int8_t par_h5;
    uint8_t par_h6;
    int8_t par_h7;
    
    uint8_t par_gh1;
    int16_t par_gh2;
    int8_t par_gh3;
    int8_t par_gh4;
    
    float t_fine;
} bme680_calib_data_t;

// Nueva estructura de dispositivo (inspirada en gschorcht)
typedef struct {
    uint8_t i2c_addr;
    bool initialized;
    bme680_calib_data_t calib;
    bme680_osr_t osr_t;
    bme680_osr_t osr_p;
    bme680_osr_t osr_h;
    bme680_mode_t mode;
    uint16_t heater_temp;
    uint16_t heater_duration;
    int16_t ambient_temp;
} bme680_device_t;

static bme680_device_t bme680_dev = {
    .i2c_addr = BME680_ADDR,
    .initialized = false,
    .osr_t = BME680_OSR_2X,
    .osr_p = BME680_OSR_4X,
    .osr_h = BME680_OSR_2X,
    .mode = BME680_FORCED_MODE,
    .heater_temp = 200,
    .heater_duration = 100,
    .ambient_temp = 25
};

static bme680_data_t last_sensor_data = {0};
static uint8_t bme680_current_addr = BME680_ADDR;

// ==================== DIAGNÓSTICO I2C (TU CÓDIGO) ====================

void i2c_diagnostic(void) {
    ESP_LOGI(TAG, "Iniciando diagnostico I2C completo...");
    
    // 1. Escanear bus I2C completo
    ESP_LOGI(TAG, "Escaneando bus I2C...");
    int devices_found = 0;
    
    for (int address = 1; address < 127; address++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Dispositivo encontrado en: 0x%02X", address);
            devices_found++;
        }
    }
    
    if (devices_found == 0) {
        ESP_LOGE(TAG, "NO se encontraron dispositivos I2C. Verifica conexiones.");
    } else {
        ESP_LOGI(TAG, "Total dispositivos encontrados: %d", devices_found);
    }
    
    // 2. Probar direcciones específicas del BME680
    uint8_t test_addresses[] = {0x76, 0x77};
    ESP_LOGI(TAG, "Probando direcciones especificas del BME680...");
    
    for (int i = 0; i < sizeof(test_addresses)/sizeof(test_addresses[0]); i++) {
        ESP_LOGI(TAG, "Probando BME680 en: 0x%02X", test_addresses[i]);
        
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (test_addresses[i] << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "BME680 RESPONDE en: 0x%02X", test_addresses[i]);
            
            // Leer Chip ID para confirmar
            uint8_t chip_id;
            
            i2c_cmd_handle_t cmd_read = i2c_cmd_link_create();
            i2c_master_start(cmd_read);
            i2c_master_write_byte(cmd_read, (test_addresses[i] << 1) | I2C_MASTER_WRITE, true);
            i2c_master_write_byte(cmd_read, 0xD0, true);
            i2c_master_start(cmd_read);
            i2c_master_write_byte(cmd_read, (test_addresses[i] << 1) | I2C_MASTER_READ, true);
            i2c_master_read_byte(cmd_read, &chip_id, I2C_MASTER_NACK);
            i2c_master_stop(cmd_read);
            
            esp_err_t read_ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_read, 100 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd_read);
            
            if (read_ret == ESP_OK) {
                ESP_LOGI(TAG, "Chip ID: 0x%02X", chip_id);
                if (chip_id == 0x61) {
                    ESP_LOGI(TAG, "CONFIRMADO: BME680 en 0x%02X", test_addresses[i]);
                    bme680_current_addr = test_addresses[i];
                } else {
                    ESP_LOGW(TAG, "Chip ID incorrecto. Esperado: 0x61");
                }
            } else {
                ESP_LOGE(TAG, "Error leyendo Chip ID: %s", esp_err_to_name(read_ret));
            }
        } else {
            ESP_LOGW(TAG, "BME680 NO responde en: 0x%02X - Error: %s", 
                     test_addresses[i], esp_err_to_name(ret));
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// ==================== FUNCIONES I2C BÁSICAS (TUS FUNCIONES) ====================

static esp_err_t bme680_write_byte(uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (bme680_current_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

static esp_err_t bme680_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (bme680_current_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (bme680_current_addr << 1) | I2C_MASTER_READ, true);
    
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

// ==================== NUEVAS FUNCIONES DE CONFIGURACIÓN (gschorcht) ====================

esp_err_t bme680_set_oversampling_rates(bme680_osr_t osr_t, bme680_osr_t osr_p, bme680_osr_t osr_h) {
    if (!bme680_dev.initialized) {
        ESP_LOGE(TAG, "BME680 no inicializado");
        return ESP_FAIL;
    }
    
    bme680_dev.osr_t = osr_t;
    bme680_dev.osr_p = osr_p;
    bme680_dev.osr_h = osr_h;
    
    uint8_t ctrl_hum = osr_h & 0x07;
    uint8_t ctrl_meas = ((osr_t & 0x07) << 5) | ((osr_p & 0x07) << 2) | (bme680_dev.mode & 0x03);
    
    ESP_ERROR_CHECK(bme680_write_byte(0x72, ctrl_hum));
    ESP_ERROR_CHECK(bme680_write_byte(0x74, ctrl_meas));
    
    ESP_LOGI(TAG, "⚙️ Oversampling configurado: Temp=x%d, Pres=x%d, Hum=x%d", 
            1 << (osr_t - 1), 1 << (osr_p - 1), 1 << (osr_h - 1));
    
    return ESP_OK;
}

esp_err_t bme680_set_heater_profile(uint16_t temperature, uint16_t duration) {
    if (!bme680_dev.initialized) {
        ESP_LOGE(TAG, "BME680 no inicializado");
        return ESP_FAIL;
    }
    
    bme680_dev.heater_temp = temperature;
    bme680_dev.heater_duration = duration;
    
    // Configurar temperatura del calentador
    uint8_t res_heat = 0x00; // Deberías calcular esto correctamente según el datasheet
    ESP_ERROR_CHECK(bme680_write_byte(0x5A, res_heat));
    
    // Configurar duración
    uint8_t gas_wait = duration / 10; // Convertir a unidades de 10ms
    ESP_ERROR_CHECK(bme680_write_byte(0x64, gas_wait));
    
    ESP_LOGI(TAG, "🔥 Heater configurado: Temp=%d°C, Duración=%dms", temperature, duration);
    
    return ESP_OK;
}

esp_err_t bme680_set_operation_mode(bme680_mode_t mode) {
    if (!bme680_dev.initialized) {
        ESP_LOGE(TAG, "BME680 no inicializado");
        return ESP_FAIL;
    }
    
    bme680_dev.mode = mode;
    
    uint8_t ctrl_meas = ((bme680_dev.osr_t & 0x07) << 5) | 
                        ((bme680_dev.osr_p & 0x07) << 2) | 
                        (mode & 0x03);
    
    ESP_ERROR_CHECK(bme680_write_byte(0x74, ctrl_meas));
    
    return ESP_OK;
}

esp_err_t bme680_set_ambient_temperature(int16_t temperature) {
    if (!bme680_dev.initialized) {
        ESP_LOGE(TAG, "BME680 no inicializado");
        return ESP_FAIL;
    }
    
    bme680_dev.ambient_temp = temperature;
    
    ESP_LOGI(TAG, "🌡️ Temperatura ambiente configurada: %d°C", temperature);
    
    return ESP_OK;
}

bme680_mode_t bme680_get_operation_mode(void) {
    return bme680_dev.mode;
}

esp_err_t bme680_apply_config(bme680_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;
    
    ESP_ERROR_CHECK(bme680_set_oversampling_rates(config->osr_temperature, 
                                                  config->osr_pressure, 
                                                  config->osr_humidity));
    ESP_ERROR_CHECK(bme680_set_heater_profile(config->heater_temperature, 
                                              config->heater_duration));
    ESP_ERROR_CHECK(bme680_set_operation_mode(config->operation_mode));
    ESP_ERROR_CHECK(bme680_set_ambient_temperature(config->ambient_temperature));
    
    return ESP_OK;
}

// ==================== FUNCIONES DE CALIBRACIÓN ====================

static esp_err_t bme680_read_calibration_data(void) {
    ESP_LOGI(TAG, "📖 Iniciando lectura de calibración...");
    
    // Leer bloques de calibración
    uint8_t calib_data1[25];
    uint8_t calib_data2[16];
    
    esp_err_t ret = bme680_read_bytes(0x89, calib_data1, 25);
    if (ret != ESP_OK) return ret;
    
    ret = bme680_read_bytes(0xE1, calib_data2, 16);
    if (ret != ESP_OK) return ret;
    
    // Parsear calibración temperatura
    bme680_dev.calib.par_t1 = (uint16_t)(calib_data1[33 - 0x89] << 8 | calib_data1[32 - 0x89]);
    bme680_dev.calib.par_t2 = (int16_t)(calib_data1[1] << 8 | calib_data1[0]);
    bme680_dev.calib.par_t3 = (int8_t)calib_data1[2];
    
    // Parsear calibración presión
    bme680_dev.calib.par_p1 = (uint16_t)(calib_data1[5] << 8 | calib_data1[4]);
    bme680_dev.calib.par_p2 = (int16_t)(calib_data1[7] << 8 | calib_data1[6]);
    bme680_dev.calib.par_p3 = (int8_t)calib_data1[8];
    bme680_dev.calib.par_p4 = (int16_t)(calib_data1[11] << 8 | calib_data1[10]);
    bme680_dev.calib.par_p5 = (int16_t)(calib_data1[13] << 8 | calib_data1[12]);
    bme680_dev.calib.par_p6 = (int8_t)calib_data1[15];
    bme680_dev.calib.par_p7 = (int8_t)calib_data1[14];
    bme680_dev.calib.par_p8 = (int16_t)(calib_data1[19] << 8 | calib_data1[18]);
    bme680_dev.calib.par_p9 = (int16_t)(calib_data1[21] << 8 | calib_data1[20]);
    bme680_dev.calib.par_p10 = calib_data1[22];
    
    // Parsear calibración humedad
    bme680_dev.calib.par_h1 = (uint16_t)(((uint16_t)calib_data2[1] << 4) | (calib_data2[0] & 0x0F));
    bme680_dev.calib.par_h2 = (uint16_t)(((uint16_t)calib_data2[2] << 4) | (calib_data2[1] >> 4));
    bme680_dev.calib.par_h3 = (int8_t)calib_data2[3];
    bme680_dev.calib.par_h4 = (int8_t)calib_data2[4];
    bme680_dev.calib.par_h5 = (int8_t)calib_data2[5];
    bme680_dev.calib.par_h6 = (uint8_t)calib_data2[6];
    bme680_dev.calib.par_h7 = (int8_t)calib_data2[7];
    
    // Parsear calibración gas
    bme680_dev.calib.par_gh1 = (uint8_t)calib_data2[8];
    bme680_dev.calib.par_gh2 = (int16_t)(calib_data2[10] << 8 | calib_data2[9]);
    bme680_dev.calib.par_gh3 = (int8_t)calib_data2[11];
    
    ESP_LOGI(TAG, "✅ Calibración leída correctamente");
    
    return ESP_OK;
}

static float bme680_compensate_temperature(uint32_t adc_temp) {
    float var1 = ((float)adc_temp / 16384.0f - (float)bme680_dev.calib.par_t1 / 1024.0f) *
                 (float)bme680_dev.calib.par_t2;
    
    float var2 = (((float)adc_temp / 131072.0f - (float)bme680_dev.calib.par_t1 / 8192.0f) *
                  ((float)adc_temp / 131072.0f - (float)bme680_dev.calib.par_t1 / 8192.0f)) *
                 ((float)bme680_dev.calib.par_t3 * 16.0f);
    
    bme680_dev.calib.t_fine = var1 + var2;
    
    return bme680_dev.calib.t_fine / 5120.0f;
}

static float bme680_compensate_pressure(uint32_t adc_press) {
    float var1 = (bme680_dev.calib.t_fine / 2.0f) - 64000.0f;
    float var2 = var1 * var1 * ((float)bme680_dev.calib.par_p6 / 131072.0f);
    var2 = var2 + var1 * ((float)bme680_dev.calib.par_p5 * 2.0f);
    var2 = (var2 / 4.0f) + ((float)bme680_dev.calib.par_p4 * 65536.0f);
    var1 = (((float)bme680_dev.calib.par_p3 * var1 * var1 / 16384.0f) + 
            ((float)bme680_dev.calib.par_p2 * var1)) / 524288.0f;
    var1 = (1.0f + (var1 / 32768.0f)) * (float)bme680_dev.calib.par_p1;
    
    if (var1 == 0.0f) return 0.0f; // Evitar división por cero
    
    float pressure = 1048576.0f - (float)adc_press;
    pressure = (pressure - (var2 / 4096.0f)) * 6250.0f / var1;
    var1 = ((float)bme680_dev.calib.par_p9 * pressure * pressure) / 2147483648.0f;
    var2 = pressure * ((float)bme680_dev.calib.par_p8 / 32768.0f);
    
    pressure = pressure + (var1 + var2 + ((float)bme680_dev.calib.par_p7 * 128.0f)) / 16.0f;
    
    return pressure / 100.0f; // Convertir a hPa
}

static float bme680_compensate_humidity(uint16_t adc_hum) {
    float temp_comp = bme680_dev.calib.t_fine / 5120.0f;
    
    float var1 = (float)adc_hum - (((float)bme680_dev.calib.par_h1 * 16.0f) + 
                                   ((float)bme680_dev.calib.par_h3 / 2.0f * temp_comp));
    float var2 = var1 * ((float)bme680_dev.calib.par_h2 / 262144.0f * 
                         (1.0f + ((float)bme680_dev.calib.par_h4 / 16384.0f * temp_comp) + 
                          ((float)bme680_dev.calib.par_h5 / 1048576.0f * temp_comp * temp_comp)));
    float var3 = (float)bme680_dev.calib.par_h6 / 16384.0f;
    float var4 = (float)bme680_dev.calib.par_h7 / 2097152.0f;
    
    float humidity = var2 + (var3 + var4 * temp_comp) * var2 * var2;
    
    if (humidity > 100.0f) humidity = 100.0f;
    else if (humidity < 0.0f) humidity = 0.0f;
    
    return humidity;
}

static uint32_t bme680_compensate_gas(uint16_t adc_gas, uint8_t gas_range) {
    // Implementación simplificada - requiere tablas de lookup del datasheet
    uint32_t gas_resistance = adc_gas * 100; // Placeholder
    return gas_resistance;
}

static float bme680_calculate_air_quality(uint32_t gas_resistance, float humidity) {
    // Algoritmo simplificado para calidad del aire
    if (gas_resistance == 0) return 0.0f;
    
    float air_quality = (gas_resistance / 1000.0f) * (1.0f - (humidity / 100.0f));
    if (air_quality < 0) air_quality = 0;
    if (air_quality > 100) air_quality = 100;
    
    return air_quality;
}

// ==================== INICIALIZACIÓN DEL SENSOR ====================

void bme680_init(void) {
    ESP_LOGI(TAG, "🚀 Inicializando BME680...");
    
    // Inicializar I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0));
    
    // Diagnóstico I2C
    i2c_diagnostic();
    
    // Resetear sensor
    ESP_LOGI(TAG, "🔄 Reseteando BME680...");
    ESP_ERROR_CHECK(bme680_write_byte(0xE0, 0xB6));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // Leer datos de calibración
    ESP_LOGI(TAG, "📖 Leyendo datos de calibración...");
    esp_err_t ret = bme680_read_calibration_data();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Error leyendo calibración: %s", esp_err_to_name(ret));
        return;
    }
    
    bme680_dev.initialized = true;
    
    // Configurar sensor
    ESP_ERROR_CHECK(bme680_configure_sensor());
    
    ESP_LOGI(TAG, "✅ BME680 inicializado correctamente");
}

esp_err_t bme680_configure_sensor(void) {
    // Configurar oversampling
    ESP_ERROR_CHECK(bme680_set_oversampling_rates(bme680_dev.osr_t, bme680_dev.osr_p, bme680_dev.osr_h));
    
    // Configurar heater
    ESP_ERROR_CHECK(bme680_set_heater_profile(bme680_dev.heater_temp, bme680_dev.heater_duration));
    
    // Configurar modo
    ESP_ERROR_CHECK(bme680_set_operation_mode(bme680_dev.mode));
    
    return ESP_OK;
}

esp_err_t bme680_read_all_data(bme680_data_t *sensor_data) {
    // Verificar si el sensor está conectado
    if (!bme680_is_connected()) {
        ESP_LOGW(TAG, "⚠️ BME680 no detectado. Retornando valores por defecto.");
        
        sensor_data->temperature = -999.0f;
        sensor_data->humidity = -1.0f;
        sensor_data->pressure = -1.0f;
        sensor_data->gas_resistance = 0;
        sensor_data->air_quality = 0.0f;
        sensor_data->raw_gas = 0;
        
        return ESP_ERR_NOT_FOUND;
    }
    
    // Si está en modo forzado, asegurarse de iniciar medición
    if (bme680_dev.mode == BME680_FORCED_MODE) {
        bme680_set_operation_mode(BME680_FORCED_MODE);
        vTaskDelay(100 / portTICK_PERIOD_MS); // Esperar a que la medición esté lista
    }
    
    uint8_t sensor_data_raw[15];
    uint8_t gas_range;
    
    // Leer todos los datos del sensor (registros 0x1D a 0x2B)
    esp_err_t ret = bme680_read_bytes(0x1D, sensor_data_raw, 15);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Error leyendo datos del sensor: %s", esp_err_to_name(ret));
        
        sensor_data->temperature = -999.0f;
        sensor_data->humidity = -1.0f;
        sensor_data->pressure = -1.0f;
        sensor_data->gas_resistance = 0;
        sensor_data->air_quality = 0.0f;
        sensor_data->raw_gas = 0;
        
        return ret;
    }
    
    // Extraer valores ADC de los datos brutos
    uint32_t temp_adc = (uint32_t)((sensor_data_raw[5] << 12) | (sensor_data_raw[6] << 4) | (sensor_data_raw[7] >> 4));
    uint32_t press_adc = (uint32_t)((sensor_data_raw[2] << 12) | (sensor_data_raw[3] << 4) | (sensor_data_raw[4] >> 4));
    uint16_t hum_adc = (uint16_t)((sensor_data_raw[8] << 8) | sensor_data_raw[9]);
    uint16_t gas_adc = (uint16_t)((sensor_data_raw[13] << 2) | (sensor_data_raw[14] >> 6));
    gas_range = sensor_data_raw[14] & 0x0F;
    
    // Verificar si los valores ADC son razonables
    if (temp_adc == 0 || temp_adc > 0xFFFFF) {
        ESP_LOGW(TAG, "⚠️ Valor ADC de temperatura fuera de rango: %lu", (unsigned long)temp_adc);
        sensor_data->temperature = -999.0f;
    } else {
        // Después de extraer temp_adc:
        ESP_LOGI(TAG, "🌡️ ADC crudo temperatura: %lu (0x%06lX)", 
         (unsigned long)temp_adc, (unsigned long)temp_adc);

        float raw_temp = bme680_compensate_temperature(temp_adc);
        float calibrated_temp = (raw_temp * BME680_TEMP_SCALE) + BME680_TEMP_OFFSET_C;
        ESP_LOGI(TAG, "🌡️ Temperatura compensada: %.2f°C (calibrada: %.2f°C)", raw_temp, calibrated_temp);
        sensor_data->temperature = calibrated_temp;
    }
    
    if (press_adc == 0 || press_adc > 0xFFFFF) {
        ESP_LOGW(TAG, "⚠️ Valor ADC de presión fuera de rango: %lu", (unsigned long)press_adc);
        sensor_data->pressure = -1.0f;
    } else {
        sensor_data->pressure = bme680_compensate_pressure(press_adc);
    }
    
    if (hum_adc == 0 || hum_adc > 0xFFFF) {
        ESP_LOGW(TAG, "⚠️ Valor ADC de humedad fuera de rango: %u", hum_adc);
        sensor_data->humidity = -1.0f;
    } else {
        float raw_humidity = bme680_compensate_humidity(hum_adc);
        sensor_data->humidity = (raw_humidity * BME680_HUM_SCALE) + BME680_HUM_OFFSET_PCT;
    }
    
    // Procesar datos de gas
    sensor_data->raw_gas = gas_adc;
    if (gas_adc > 0) {
        sensor_data->gas_resistance = bme680_compensate_gas(gas_adc, gas_range);
        sensor_data->air_quality = bme680_calculate_air_quality(sensor_data->gas_resistance, sensor_data->humidity);
    } else {
        sensor_data->gas_resistance = 0;
        sensor_data->air_quality = 0.0f;
        ESP_LOGD(TAG, "🔇 Sensor de gas no activo o sin datos");
    }
    
    // Validar rangos razonables de los datos compensados
    if (sensor_data->temperature < -40.0f || sensor_data->temperature > 85.0f) {
        ESP_LOGW(TAG, "⚠️ Temperatura fuera de rango operativo: %.2f", sensor_data->temperature);
    }
    
    if (sensor_data->humidity >= 0.0f && (sensor_data->humidity < 0.0f || sensor_data->humidity > 100.0f)) {
        ESP_LOGW(TAG, "⚠️ Humedad fuera de rango: %.2f", sensor_data->humidity);
        if (sensor_data->humidity > 100.0f) sensor_data->humidity = 100.0f;
        if (sensor_data->humidity < 0.0f) sensor_data->humidity = 0.0f;
    }
    
    if (sensor_data->pressure >= 0.0f && (sensor_data->pressure < 300.0f || sensor_data->pressure > 1100.0f)) {
        ESP_LOGW(TAG, "⚠️ Presión fuera de rango: %.2f", sensor_data->pressure);
    }
    
    // Guardar última lectura válida
    if (sensor_data->temperature > -100.0f) {
        last_sensor_data = *sensor_data;
    }
    
    // Si está en modo forzado, reiniciar para siguiente lectura
    if (bme680_dev.mode == BME680_FORCED_MODE) {
        esp_err_t write_ret = bme680_set_operation_mode(BME680_FORCED_MODE);
        if (write_ret != ESP_OK) {
            ESP_LOGW(TAG, "⚠️ Error reiniciando medición: %s", esp_err_to_name(write_ret));
        }
    }
    
    ESP_LOGD(TAG, "📊 Datos BME680 procesados - Temp: %.2f°C, Hum: %.2f%%, Pres: %.2fhPa", 
             sensor_data->temperature, sensor_data->humidity, sensor_data->pressure);
    
    return ESP_OK;
}

// ==================== VERIFICACIÓN DE CONEXIÓN (TU CÓDIGO) ====================
bool bme680_is_connected(void) {
    uint8_t chip_id;
    esp_err_t ret = bme680_read_bytes(0xD0, &chip_id, 1);
    
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "BME680 no responde en 0x%02X: %s", 
                 bme680_current_addr, esp_err_to_name(ret));
        return false;
    }
    
    if (chip_id != 0x61) {
        ESP_LOGD(TAG, "Chip ID incorrecto en 0x%02X: 0x%02X (esperado: 0x61)", 
                 bme680_current_addr, chip_id);
        return false;
    }
    
    ESP_LOGI(TAG, "BME680 detectado en: 0x%02X, Chip ID: 0x%02X", 
             bme680_current_addr, chip_id);
    return true;
}
