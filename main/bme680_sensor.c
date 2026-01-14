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
    
    // Cálculo de resistencia del heater (algoritmo de gschorcht)
    int32_t var1, var2, var3, var4, var5, heatr_res;
    
    var1 = (((int32_t)bme680_dev.ambient_temp * bme680_dev.calib.par_gh3) / 1000) * 256;
    var2 = (bme680_dev.calib.par_gh1 + 784) * (((((bme680_dev.calib.par_gh2 + 154009) * temperature * 5) / 100) + 3276800) / 10);
    var3 = var1 + (var2 / 2);
    var4 = (var3 / (bme680_dev.calib.par_gh4 + 4));
    var5 = (131 * bme680_dev.calib.par_gh1) + 65536;
    heatr_res = (int32_t)((var4 / var5) - 250);
    
    if (heatr_res < 0) heatr_res = 0;
    if (heatr_res > 255) heatr_res = 255;
    
    // Configurar resistencia y duración
    ESP_ERROR_CHECK(bme680_write_byte(0x5A, (uint8_t)heatr_res));
    ESP_ERROR_CHECK(bme680_write_byte(0x64, (uint8_t)duration));
    
    // Habilitar gas sensor
    ESP_ERROR_CHECK(bme680_write_byte(0x71, 0x10 | 0)); // enable gas, profile 0
    
    ESP_LOGI(TAG, "🔥 Heater configurado: %d°C por %dms (Resistencia: %"PRId32")", 
            temperature, duration, heatr_res);
    
    return ESP_OK;
}

esp_err_t bme680_set_operation_mode(bme680_mode_t mode) {
    if (!bme680_dev.initialized) {
        ESP_LOGE(TAG, "BME680 no inicializado");
        return ESP_FAIL;
    }
    
    bme680_dev.mode = mode;
    
    uint8_t ctrl_meas;
    ESP_ERROR_CHECK(bme680_read_bytes(0x74, &ctrl_meas, 1));
    
    ctrl_meas &= ~0x03;
    ctrl_meas |= (mode & 0x03);
    
    ESP_ERROR_CHECK(bme680_write_byte(0x74, ctrl_meas));
    
    const char* mode_str[] = {"SLEEP", "FORCED", "PARALLEL", "NORMAL"};
    ESP_LOGI(TAG, "🔄 Modo de operación: %s", mode_str[mode]);
    
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
    if (!bme680_dev.initialized) {
        return BME680_SLEEP_MODE;
    }
    
    uint8_t ctrl_meas;
    if (bme680_read_bytes(0x74, &ctrl_meas, 1) != ESP_OK) {
        return BME680_SLEEP_MODE;
    }
    
    return (bme680_mode_t)(ctrl_meas & 0x03);
}

esp_err_t bme680_apply_config(bme680_config_t *config) {
    if (!config) {
        ESP_LOGE(TAG, "Configuración inválida");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!bme680_dev.initialized) {
        ESP_LOGE(TAG, "❌ BME680 no inicializado - Llama a bme680_configure_sensor() primero");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "🎛️ Aplicando configuración BME680...");
    
    esp_err_t ret;
    
    // Aplicar oversampling
    ret = bme680_set_oversampling_rates(config->osr_temperature, 
                                       config->osr_pressure, 
                                       config->osr_humidity);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Error configurando oversampling: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Aplicar heater
    ret = bme680_set_heater_profile(config->heater_temperature, 
                                   config->heater_duration);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Error configurando heater: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Aplicar temperatura ambiente
    ret = bme680_set_ambient_temperature(config->ambient_temperature);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Error configurando temp ambiente: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Aplicar modo de operación
    ret = bme680_set_operation_mode(config->operation_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Error configurando modo: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "✅ Configuración aplicada correctamente");
    return ESP_OK;
}

// ==================== LECTURA DE CALIBRACIÓN (TU CÓDIGO) ====================

static esp_err_t bme680_read_calibration_data(void) {
	ESP_LOGI(TAG, "📖 Iniciando lectura de calibración...");
    uint8_t calib_data_raw[41];
    
    
    
    // Leer bloques de calibración
    ESP_ERROR_CHECK(bme680_read_bytes(0x89, calib_data_raw, 25));
    ESP_ERROR_CHECK(bme680_read_bytes(0xE1, calib_data_raw + 25, 16));
    
    // Parsear calibración temperatura
    bme680_dev.calib.par_t1 = (calib_data_raw[33] << 8) | calib_data_raw[32];
    bme680_dev.calib.par_t2 = (calib_data_raw[1] << 8) | calib_data_raw[0];
    bme680_dev.calib.par_t3 = (int8_t)calib_data_raw[2];
    
    // Parsear calibración presión
    bme680_dev.calib.par_p1 = (calib_data_raw[6] << 8) | calib_data_raw[5];
    bme680_dev.calib.par_p2 = (calib_data_raw[8] << 8) | calib_data_raw[7];
    bme680_dev.calib.par_p3 = (int8_t)calib_data_raw[9];
    bme680_dev.calib.par_p4 = (calib_data_raw[12] << 8) | calib_data_raw[11];
    bme680_dev.calib.par_p5 = (calib_data_raw[14] << 8) | calib_data_raw[13];
    bme680_dev.calib.par_p6 = (int8_t)calib_data_raw[16];
    bme680_dev.calib.par_p7 = (int8_t)calib_data_raw[15];
    bme680_dev.calib.par_p8 = (calib_data_raw[20] << 8) | calib_data_raw[19];
    bme680_dev.calib.par_p9 = (calib_data_raw[22] << 8) | calib_data_raw[21];
    bme680_dev.calib.par_p10 = calib_data_raw[23];
    
    // Parsear calibración humedad
    bme680_dev.calib.par_h1 = (calib_data_raw[27] << 4) | (calib_data_raw[26] & 0x0F);
    bme680_dev.calib.par_h2 = (calib_data_raw[25] << 4) | (calib_data_raw[26] >> 4);
    bme680_dev.calib.par_h3 = (int8_t)calib_data_raw[28];
    bme680_dev.calib.par_h4 = (int8_t)calib_data_raw[29];
    bme680_dev.calib.par_h5 = (int8_t)calib_data_raw[30];
    bme680_dev.calib.par_h6 = calib_data_raw[31];
    bme680_dev.calib.par_h7 = (int8_t)calib_data_raw[34];
    
    // Parsear calibración gas
    bme680_dev.calib.par_gh1 = calib_data_raw[35];
    bme680_dev.calib.par_gh2 = (calib_data_raw[37] << 8) | calib_data_raw[36];
    bme680_dev.calib.par_gh3 = (int8_t)calib_data_raw[38];
    bme680_dev.calib.par_gh4 = (int8_t)calib_data_raw[39];
    
    ESP_LOGI(TAG, "=== CALIBRACIÓN LEÍDA ===");
	ESP_LOGI(TAG, "Temp - T1: %u, T2: %d, T3: %d", 
         bme680_dev.calib.par_t1, 
         bme680_dev.calib.par_t2, 
         bme680_dev.calib.par_t3);
	ESP_LOGI(TAG, "Pres - P1: %u, P2: %d, P3: %d", 
         bme680_dev.calib.par_p1, 
         bme680_dev.calib.par_p2, 
         bme680_dev.calib.par_p3);
	ESP_LOGI(TAG, "Hum - H1: %u, H2: %u, H3: %d", 
         bme680_dev.calib.par_h1, 
         bme680_dev.calib.par_h2, 
         bme680_dev.calib.par_h3);
    
    return ESP_OK;
}

// ==================== COMPENSACIÓN (TUS FUNCIONES) ====================

static float bme680_compensate_temperature(uint32_t temp_adc) {
    float var1, var2, var3;
    float calc_temp;
    
    // Algoritmo DEL DATASHEET BME680 (revisión 1.0)
    var1 = ((float)temp_adc / 16384.0 - ((float)bme680_dev.calib.par_t1 / 1024.0));
    var2 = var1 * ((float)bme680_dev.calib.par_t2);
    var3 = (var1 * var1) * ((float)bme680_dev.calib.par_t3 * 16.0);
    
    bme680_dev.calib.t_fine = var2 + var3;
    calc_temp = bme680_dev.calib.t_fine / 5120.0;
    
    // DEBUG
    ESP_LOGI(TAG, "🔧 Compensación T - var1: %.6f, var2: %.6f, var3: %.6f, t_fine: %.6f",
             var1, var2, var3, bme680_dev.calib.t_fine);
    
    return calc_temp;
}

static float bme680_compensate_pressure(uint32_t press_adc) {
    float var1, var2, var3, pressure;
    
    var1 = ((float)bme680_dev.calib.t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((float)bme680_dev.calib.par_p6 / 131072.0);
    var2 = var2 + (var1 * (float)bme680_dev.calib.par_p5 * 2.0);
    var2 = (var2 / 4.0) + ((float)bme680_dev.calib.par_p4 * 65536.0);
    var1 = (((float)bme680_dev.calib.par_p3 * var1 * var1) / 16384.0 + ((float)bme680_dev.calib.par_p2 * var1)) / 524288.0;
    var1 = (1.0 + (var1 / 32768.0)) * (float)bme680_dev.calib.par_p1;
    
    if (var1 == 0.0) {
        return 0.0;
    }
    
    pressure = 1048576.0 - (float)press_adc;
    pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((float)bme680_dev.calib.par_p9 * pressure * pressure) / 2147483648.0;
    var2 = pressure * ((float)bme680_dev.calib.par_p8 / 32768.0);
    var3 = (pressure / 256.0) * (pressure / 256.0) * (pressure / 256.0) * (bme680_dev.calib.par_p10 / 131072.0);
    
    pressure = pressure + (var1 + var2 + var3 + ((float)bme680_dev.calib.par_p7 * 128.0)) / 16.0;
    
    return pressure / 100.0;
}

static float bme680_compensate_humidity(uint16_t hum_adc) {
    float var1, var2, var3, var4, var5, temp_comp;
    
    temp_comp = ((float)bme680_dev.calib.t_fine) / 5120.0;
    
    var1 = (float)hum_adc - ((float)bme680_dev.calib.par_h1 * 16.0) + (((float)bme680_dev.calib.par_h3 / 2.0) * temp_comp);
    var2 = var1 * ((float)bme680_dev.calib.par_h2 / 262144.0 * (1.0 + ((float)bme680_dev.calib.par_h4 / 16384.0 * temp_comp) + 
            ((float)bme680_dev.calib.par_h5 / 1048576.0 * temp_comp * temp_comp)));
    var3 = (float)bme680_dev.calib.par_h6 / 16384.0;
    var4 = (float)bme680_dev.calib.par_h7 / 2097152.0;
    var5 = var2 + (var3 + var4 * temp_comp) * var2 * var2;
    
    if (var5 > 100.0) var5 = 100.0;
    if (var5 < 0.0) var5 = 0.0;
    
    return var5;
}

static uint32_t bme680_compensate_gas(uint16_t gas_adc, uint8_t gas_range) {
    float var1, var2, var3, gas_res;
    const float lookup_k1_range[16] = {0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, -0.8, 0.0, 0.0, -0.2, -0.5, 0.0, -1.0, 0.0, 0.0};
    const float lookup_k2_range[16] = {0.0, 0.0, 0.0, 0.0, 0.1, 0.7, 0.0, -0.8, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    var1 = (1340.0 + 5.0 * (float)bme680_dev.calib.par_gh3) * (1.0 + (float)bme680_dev.calib.par_gh1 / 100.0);
    var2 = var1 * (1.0 + ((float)gas_adc / 100.0));
    var3 = 1.0 + ((float)bme680_dev.calib.par_gh2 / 100.0);
    
    gas_res = var2 * var3;
    
    // Aplicar compensación de rango
    gas_res += lookup_k1_range[gas_range] * (1.0 - gas_res / 100.0);
    gas_res += lookup_k2_range[gas_range] * (1.0 - gas_res / 100.0);
    
    return (uint32_t)gas_res;
}

// ==================== CALCULAR CALIDAD DEL AIRE (TU FUNCIÓN) ====================

static float bme680_calculate_air_quality(uint32_t gas_resistance, float humidity) {
    float quality_score = 0.0;
    
    if (gas_resistance > 0) {
        quality_score = (gas_resistance / 50000.0) * 100.0;
        
        if (humidity < 30.0 || humidity > 70.0) {
            quality_score *= 0.8;
        }
        
        if (quality_score > 100.0) quality_score = 100.0;
        if (quality_score < 0.0) quality_score = 0.0;
    }
    
    return quality_score;
}

// ==================== CONFIGURACIÓN MEJORADA ====================

esp_err_t bme680_configure_sensor(void) {
    // Verificar chip ID
    uint8_t chip_id;
    esp_err_t ret = bme680_read_bytes(0xD0, &chip_id, 1);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Error comunicando con BME680");
        return ESP_FAIL;
    }
    
    if (chip_id != 0x61) {
        ESP_LOGE(TAG, "❌ Chip ID incorrecto: 0x%02X. Esperado: 0x61", chip_id);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "✅ Chip ID correcto: 0x%02X", chip_id);
    
    // Leer datos de calibración
    ESP_LOGI(TAG, "📖 Leyendo datos de calibración...");
    ret = bme680_read_calibration_data();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Error leyendo calibración: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // ⚠️ CORRECCIÓN: Marcar como inicializado ANTES de aplicar configuración
    bme680_dev.initialized = true;
    
    // Configuración por defecto mejorada
    bme680_config_t default_config = {
        .osr_temperature = BME680_OSR_2X,
        .osr_pressure = BME680_OSR_4X,
        .osr_humidity = BME680_OSR_2X,
        .operation_mode = BME680_FORCED_MODE,
        .heater_temperature = 100,
        .heater_duration = 25,
        .ambient_temperature = 25
    };
    
    // Aplicar configuración
    ESP_LOGI(TAG, "⚙️ Aplicando configuración por defecto...");
    ESP_ERROR_CHECK(bme680_apply_config(&default_config));
    
    ESP_LOGI(TAG, "⚙️ Sensor BME680 configurado completamente");
    return ESP_OK;
}

// ==================== LECTURA ROBUSTA DE TODOS LOS DATOS (TU CÓDIGO MEJORADO) ====================

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
        sensor_data->humidity = bme680_compensate_humidity(hum_adc);
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

// ==================== INICIALIZACIÓN I2C (TU CÓDIGO) ====================

void bme680_init(void) {
    ESP_LOGI(TAG, "Inicializando BME680...");
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
    
    ESP_LOGI(TAG, "I2C inicializado correctamente");
    
    // Pequeña pausa para estabilización
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // Ejecutar diagnóstico completo (opcional)
    i2c_diagnostic();
    
    // Intentar configurar sensor
    if (bme680_is_connected()) {
        if (bme680_configure_sensor() == ESP_OK) {
            ESP_LOGI(TAG, "BME680 inicializado y configurado en 0x%02X", bme680_current_addr);
        } else {
            ESP_LOGE(TAG, "Error configurando BME680");
        }
    } else {
        ESP_LOGE(TAG, "BME680 no detectado después del diagnostico");
    }
}

// ==================== TAREA DE LECTURA CONTINUA (TU CÓDIGO) ====================

static void bme680_reading_task(void *pvParameters) {
    bme680_data_t sensor_data;
    
    ESP_LOGI(TAG, "📈 Iniciando lecturas continuas BME680...");
    
    while(1) {
        esp_err_t ret = bme680_read_all_data(&sensor_data);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "🌡️ BME680 - Temp: %.2f°C, Hum: %.1f%%, Pres: %.2fhPa, Gas: %"PRIu32"Ω, Calidad: %.1f/100", 
                     sensor_data.temperature, sensor_data.humidity, sensor_data.pressure, 
                     sensor_data.gas_resistance, sensor_data.air_quality);
        } else {
            ESP_LOGE(TAG, "❌ Error leyendo BME680: %s", esp_err_to_name(ret));
        }
        
        // Esperar 5 segundos entre lecturas
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

// ==================== INICIAR TAREA DE LECTURA (TU CÓDIGO) ====================

void bme680_start_reading_task(void) {
    xTaskCreate(bme680_reading_task, "bme680_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "🔄 Tarea de lectura BME680 iniciada");
}

// ==================== OBTENER ÚLTIMOS DATOS (TU CÓDIGO) ====================

bme680_data_t* bme680_get_last_data(void) {
    return &last_sensor_data;
}

// ==================== FUNCIÓN DE REINICIO SUAVE (NUEVA - de gschorcht) ====================

esp_err_t bme680_soft_reset(void) {
    ESP_LOGI(TAG, "🔄 Realizando reinicio suave del BME680...");
    
    // Enviar comando de reset
    esp_err_t ret = bme680_write_byte(0xE0, 0xB6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Error en reinicio suave");
        return ret;
    }
    
    // Esperar a que el reinicio se complete
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // Verificar que el sensor responde
    if (!bme680_is_connected()) {
        ESP_LOGE(TAG, "❌ BME680 no responde después del reinicio");
        return ESP_FAIL;
    }
    
    // Reconfigurar el sensor
    ret = bme680_configure_sensor();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Error reconfigurando después del reinicio");
        return ret;
    }
    
    ESP_LOGI(TAG, "✅ Reinicio suave completado correctamente");
    return ESP_OK;
}

// ==================== FUNCIÓN PARA CAMBIAR MODO DINÁMICAMENTE (NUEVA) ====================

esp_err_t bme680_switch_to_normal_mode(void) {
    ESP_LOGI(TAG, "🔄 Cambiando a modo NORMAL para lecturas continuas...");
    
    bme680_config_t normal_config = {
        .osr_temperature = bme680_dev.osr_t,
        .osr_pressure = bme680_dev.osr_p,
        .osr_humidity = bme680_dev.osr_h,
        .operation_mode = BME680_NORMAL_MODE,
        .heater_temperature = bme680_dev.heater_temp,
        .heater_duration = bme680_dev.heater_duration,
        .ambient_temperature = bme680_dev.ambient_temp
    };
    
    return bme680_apply_config(&normal_config);
}

esp_err_t bme680_switch_to_forced_mode(void) {
    ESP_LOGI(TAG, "🔄 Cambiando a modo FORCED para lecturas bajo demanda...");
    
    bme680_config_t forced_config = {
        .osr_temperature = bme680_dev.osr_t,
        .osr_pressure = bme680_dev.osr_p,
        .osr_humidity = bme680_dev.osr_h,
        .operation_mode = BME680_FORCED_MODE,
        .heater_temperature = bme680_dev.heater_temp,
        .heater_duration = bme680_dev.heater_duration,
        .ambient_temperature = bme680_dev.ambient_temp
    };
    
    return bme680_apply_config(&forced_config);
}

// ==================== FUNCIÓN DE ESTADO DEL SENSOR (NUEVA) ====================

void bme680_print_status(void) {
    if (!bme680_dev.initialized) {
        ESP_LOGI(TAG, "📊 Estado BME680: NO INICIALIZADO");
        return;
    }
    
    const char* mode_str[] = {"SLEEP", "FORCED", "PARALLEL", "NORMAL"};
    const char* osr_str[] = {"NONE", "1X", "2X", "4X", "8X", "16X"};
    
    ESP_LOGI(TAG, "📊 Estado BME680:");
    ESP_LOGI(TAG, "  🔌 Dirección I2C: 0x%02X", bme680_current_addr);
    ESP_LOGI(TAG, "  🎛️  Modo: %s", mode_str[bme680_dev.mode]);
    ESP_LOGI(TAG, "  📈 Oversampling - T:%s, P:%s, H:%s", 
             osr_str[bme680_dev.osr_t], osr_str[bme680_dev.osr_p], osr_str[bme680_dev.osr_h]);
    ESP_LOGI(TAG, "  🔥 Heater: %d°C por %dms", bme680_dev.heater_temp, bme680_dev.heater_duration);
    ESP_LOGI(TAG, "  🌡️  Temp ambiente: %d°C", bme680_dev.ambient_temp);
    ESP_LOGI(TAG, "  ✅ Inicializado: %s", bme680_dev.initialized ? "SÍ" : "NO");
    
    // Mostrar última lectura
    if (last_sensor_data.temperature > -100.0f) {
        ESP_LOGI(TAG, "  📊 Última lectura - T:%.2f°C, H:%.1f%%, P:%.2fhPa", 
                 last_sensor_data.temperature, last_sensor_data.humidity, last_sensor_data.pressure);
    }
}

// ==================== FUNCIÓN DE OPTIMIZACIÓN PARA BAJO CONSUMO (NUEVA) ====================

esp_err_t bme680_set_low_power_mode(void) {
    ESP_LOGI(TAG, "🔋 Configurando modo bajo consumo...");
    
    bme680_config_t low_power_config = {
        .osr_temperature = BME680_OSR_1X,    // Menos oversampling = menos consumo
        .osr_pressure = BME680_OSR_1X,
        .osr_humidity = BME680_OSR_1X,
        .operation_mode = BME680_FORCED_MODE, // Solo leer cuando se necesita
        .heater_temperature = 0,              // Apagar heater
        .heater_duration = 0,
        .ambient_temperature = 25
    };
    
    esp_err_t ret = bme680_apply_config(&low_power_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ Modo bajo consumo configurado");
    }
    return ret;
}

// ==================== FUNCIÓN DE OPTIMIZACIÓN PARA ALTA PRECISIÓN (NUEVA) ====================

esp_err_t bme680_set_high_accuracy_mode(void) {
    ESP_LOGI(TAG, "🎯 Configurando modo alta precisión...");
    
    bme680_config_t high_acc_config = {
        .osr_temperature = BME680_OSR_16X,   // Máximo oversampling
        .osr_pressure = BME680_OSR_16X,
        .osr_humidity = BME680_OSR_16X,
        .operation_mode = BME680_NORMAL_MODE, // Lecturas continuas
        .heater_temperature = 300,            // Heater más caliente
        .heater_duration = 150,               // Más tiempo
        .ambient_temperature = 25
    };
    
    esp_err_t ret = bme680_apply_config(&high_acc_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ Modo alta precisión configurado");
    }
    return ret;
}

// ==================== FUNCIÓN PARA VERIFICAR HEALTH DEL SENSOR (NUEVA) ====================

bool bme680_health_check(void) {
    if (!bme680_is_connected()) {
        ESP_LOGW(TAG, "❌ Health Check: Sensor no conectado");
        return false;
    }
    
    if (!bme680_dev.initialized) {
        ESP_LOGW(TAG, "❌ Health Check: Sensor no inicializado");
        return false;
    }
    
    // Leer datos para verificar que funciona
    bme680_data_t test_data;
    esp_err_t ret = bme680_read_all_data(&test_data);
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "❌ Health Check: Error leyendo datos");
        return false;
    }
    
    // Verificar que los valores están en rangos razonables
    bool temp_ok = (test_data.temperature >= -40.0f && test_data.temperature <= 85.0f);
    bool hum_ok = (test_data.humidity >= 0.0f && test_data.humidity <= 100.0f);
    bool press_ok = (test_data.pressure >= 300.0f && test_data.pressure <= 1100.0f);
    
    if (!temp_ok || !hum_ok || !press_ok) {
        ESP_LOGW(TAG, "❌ Health Check: Valores fuera de rango - T:%.2f H:%.1f P:%.2f", 
                 test_data.temperature, test_data.humidity, test_data.pressure);
        return false;
    }
    
    ESP_LOGI(TAG, "✅ Health Check: Sensor funcionando correctamente");
    return true;
}
