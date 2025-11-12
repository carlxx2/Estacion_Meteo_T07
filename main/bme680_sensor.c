#include "system_config.h"

static const char *TAG = "BME680_SENSOR";

static uint8_t bme680_current_addr = BME680_ADDR;

// Estructura de calibración completa
typedef struct {
    // Calibración temperatura
    uint16_t par_t1;
    int16_t par_t2;
    int8_t par_t3;
    
    // Calibración presión
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
    
    // Calibración humedad
    uint16_t par_h1;
    uint16_t par_h2;
    int8_t par_h3;
    int8_t par_h4;
    int8_t par_h5;
    uint8_t par_h6;
    int8_t par_h7;
    
    // Calibración gas
    uint8_t par_gh1;
    int16_t par_gh2;
    int8_t par_gh3;
    
    // Variables de compensación
    float t_fine;
    
} bme680_calib_data_t;

static bme680_calib_data_t calib_data;
static bme680_data_t last_sensor_data = {0};

// ==================== DIAGNÓSTICO COMPLETO I2C ====================
static void i2c_diagnostic(void) {
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
            
            // Función temporal para leer con dirección específica
            i2c_cmd_handle_t cmd_read = i2c_cmd_link_create();
            i2c_master_start(cmd_read);
            i2c_master_write_byte(cmd_read, (test_addresses[i] << 1) | I2C_MASTER_WRITE, true);
            i2c_master_write_byte(cmd_read, 0xD0, true); // Registro Chip ID
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


// ==================== FUNCIONES I2C BÁSICAS ====================
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

// ==================== LECTURA DE CALIBRACIÓN COMPLETA ====================
static esp_err_t bme680_read_calibration_data(void) {
    uint8_t calib_data_raw[41];
    
    // Leer bloques de calibración
    ESP_ERROR_CHECK(bme680_read_bytes(0x89, calib_data_raw, 25));  // 0x89-0xA1
    ESP_ERROR_CHECK(bme680_read_bytes(0xE1, calib_data_raw + 25, 16)); // 0xE1-0xF0
    
    // Parsear calibración temperatura
    calib_data.par_t1 = (calib_data_raw[33] << 8) | calib_data_raw[32];
    calib_data.par_t2 = (calib_data_raw[1] << 8) | calib_data_raw[0];
    calib_data.par_t3 = (int8_t)calib_data_raw[2];
    
    // Parsear calibración presión
    calib_data.par_p1 = (calib_data_raw[6] << 8) | calib_data_raw[5];
    calib_data.par_p2 = (calib_data_raw[8] << 8) | calib_data_raw[7];
    calib_data.par_p3 = (int8_t)calib_data_raw[9];
    calib_data.par_p4 = (calib_data_raw[12] << 8) | calib_data_raw[11];
    calib_data.par_p5 = (calib_data_raw[14] << 8) | calib_data_raw[13];
    calib_data.par_p6 = (int8_t)calib_data_raw[16];
    calib_data.par_p7 = (int8_t)calib_data_raw[15];
    calib_data.par_p8 = (calib_data_raw[20] << 8) | calib_data_raw[19];
    calib_data.par_p9 = (calib_data_raw[22] << 8) | calib_data_raw[21];
    calib_data.par_p10 = calib_data_raw[23];
    
    // Parsear calibración humedad
    calib_data.par_h1 = (calib_data_raw[27] << 4) | (calib_data_raw[26] & 0x0F);
    calib_data.par_h2 = (calib_data_raw[25] << 4) | (calib_data_raw[26] >> 4);
    calib_data.par_h3 = (int8_t)calib_data_raw[28];
    calib_data.par_h4 = (int8_t)calib_data_raw[29];
    calib_data.par_h5 = (int8_t)calib_data_raw[30];
    calib_data.par_h6 = calib_data_raw[31];
    calib_data.par_h7 = (int8_t)calib_data_raw[34];
    
    // Parsear calibración gas
    calib_data.par_gh1 = calib_data_raw[35];
    calib_data.par_gh2 = (calib_data_raw[37] << 8) | calib_data_raw[36];
    calib_data.par_gh3 = (int8_t)calib_data_raw[38];
    
    ESP_LOGI(TAG, "📊 Calibración BME680 leída correctamente");
    ESP_LOGI(TAG, "  Temp: T1=%u, T2=%d, T3=%d", calib_data.par_t1, calib_data.par_t2, calib_data.par_t3);
    ESP_LOGI(TAG, "  Hum: H1=%u, H2=%u, H3=%d", calib_data.par_h1, calib_data.par_h2, calib_data.par_h3);
    
    return ESP_OK;
}

// ==================== COMPENSACIÓN DE TEMPERATURA ====================
static float bme680_compensate_temperature(uint32_t temp_adc) {
    float var1, var2;
    
    var1 = ((float)temp_adc / 16384.0 - (float)calib_data.par_t1 / 1024.0) * (float)calib_data.par_t2;
    var2 = (((float)temp_adc / 131072.0 - (float)calib_data.par_t1 / 8192.0) * 
           ((float)temp_adc / 131072.0 - (float)calib_data.par_t1 / 8192.0)) * ((float)calib_data.par_t3 * 16.0);
    
    calib_data.t_fine = var1 + var2;
    
    return ((var1 + var2) / 5120.0);
}

// ==================== COMPENSACIÓN DE PRESIÓN ====================
static float bme680_compensate_pressure(uint32_t press_adc) {
    float var1, var2, var3, pressure;
    
    var1 = ((float)calib_data.t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((float)calib_data.par_p6 / 131072.0);
    var2 = var2 + (var1 * (float)calib_data.par_p5 * 2.0);
    var2 = (var2 / 4.0) + ((float)calib_data.par_p4 * 65536.0);
    var1 = (((float)calib_data.par_p3 * var1 * var1) / 16384.0 + ((float)calib_data.par_p2 * var1)) / 524288.0;
    var1 = (1.0 + (var1 / 32768.0)) * (float)calib_data.par_p1;
    
    if (var1 == 0.0) {
        return 0.0; // Evitar división por cero
    }
    
    pressure = 1048576.0 - (float)press_adc;
    pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((float)calib_data.par_p9 * pressure * pressure) / 2147483648.0;
    var2 = pressure * ((float)calib_data.par_p8 / 32768.0);
    var3 = (pressure / 256.0) * (pressure / 256.0) * (pressure / 256.0) * (calib_data.par_p10 / 131072.0);
    
    pressure = pressure + (var1 + var2 + var3 + ((float)calib_data.par_p7 * 128.0)) / 16.0;
    
    return pressure / 100.0; // Convertir a hPa
}

// ==================== COMPENSACIÓN DE HUMEDAD ====================
static float bme680_compensate_humidity(uint16_t hum_adc) {
    float var1, var2, var3, var4, var5, temp_comp;
    
    temp_comp = ((float)calib_data.t_fine) / 5120.0;
    
    var1 = (float)hum_adc - ((float)calib_data.par_h1 * 16.0) + (((float)calib_data.par_h3 / 2.0) * temp_comp);
    var2 = var1 * ((float)calib_data.par_h2 / 262144.0 * (1.0 + ((float)calib_data.par_h4 / 16384.0 * temp_comp) + 
            ((float)calib_data.par_h5 / 1048576.0 * temp_comp * temp_comp)));
    var3 = (float)calib_data.par_h6 / 16384.0;
    var4 = (float)calib_data.par_h7 / 2097152.0;
    var5 = var2 + (var3 + var4 * temp_comp) * var2 * var2;
    
    if (var5 > 100.0) var5 = 100.0;
    if (var5 < 0.0) var5 = 0.0;
    
    return var5;
}

// ==================== COMPENSACIÓN DE GAS ====================
static uint32_t bme680_compensate_gas(uint16_t gas_adc, uint8_t gas_range) {
    float var1, var2, var3, gas_res;
    const float lookup_k1_range[16] = {0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, -0.8, 0.0, 0.0, -0.2, -0.5, 0.0, -1.0, 0.0, 0.0};
    const float lookup_k2_range[16] = {0.0, 0.0, 0.0, 0.0, 0.1, 0.7, 0.0, -0.8, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    var1 = (1340.0 + 5.0 * (float)calib_data.par_gh3) * (1.0 + (float)calib_data.par_gh1 / 100.0);
    var2 = var1 * (1.0 + ((float)gas_adc / 100.0));
    var3 = 1.0 + ((float)calib_data.par_gh2 / 100.0);
    
    gas_res = var2 * var3;
    
    // Aplicar compensación de rango
    gas_res += lookup_k1_range[gas_range] * (1.0 - gas_res / 100.0);
    gas_res += lookup_k2_range[gas_range] * (1.0 - gas_res / 100.0);
    
    return (uint32_t)gas_res;
}

// ==================== CALCULAR CALIDAD DEL AIRE ====================
static float bme680_calculate_air_quality(uint32_t gas_resistance, float humidity) {
    // Algoritmo simple para calidad del aire basado en resistencia de gas y humedad
    float quality_score = 0.0;
    
    if (gas_resistance > 0) {
        // Valores más altos de gas_resistance indican mejor calidad del aire
        quality_score = (gas_resistance / 50000.0) * 100.0;
        
        // Ajustar por humedad (humedad ideal ~40-60%)
        if (humidity < 30.0 || humidity > 70.0) {
            quality_score *= 0.8; // Penalizar humedad extremas
        }
        
        // Limitar entre 0-100
        if (quality_score > 100.0) quality_score = 100.0;
        if (quality_score < 0.0) quality_score = 0.0;
    }
    
    return quality_score;
}

// ==================== INICIALIZACIÓN I2C ====================
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
    
    // Ejecutar diagnóstico completo
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

// ==================== CONFIGURACIÓN DEL SENSOR ====================
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
    ESP_ERROR_CHECK(bme680_read_calibration_data());
    
    // Configurar oversampling: Temp x2, Pres x4, Hum x2
    ESP_ERROR_CHECK(bme680_write_byte(0x72, 0x02)); // hum_osr x2
    ESP_ERROR_CHECK(bme680_write_byte(0x74, 0x93)); // temp_osr x2, pres_osr x4, modo normal
    
    // Configurar sensor de gas
    ESP_ERROR_CHECK(bme680_write_byte(0x71, 0x10)); // enable gas, heater off
    ESP_ERROR_CHECK(bme680_write_byte(0x64, 0x59)); // heater temp 200°C
    ESP_ERROR_CHECK(bme680_write_byte(0x71, 0x20)); // run gas
    
    ESP_LOGI(TAG, "⚙️ Sensor BME680 configurado completamente");
    return ESP_OK;
}

// ==================== LECTURA ROBUSTA DE TODOS LOS DATOS ====================
esp_err_t bme680_read_all_data(bme680_data_t *sensor_data) {
    // Verificar si el sensor está conectado
    if (!bme680_is_connected()) {
        ESP_LOGW(TAG, "⚠️ BME680 no detectado. Retornando valores por defecto.");
        
        // Inicializar estructura con valores por defecto/error
        sensor_data->temperature = -999.0f;     // Valor de error reconocible
        sensor_data->humidity = -1.0f;          // Valor inválido
        sensor_data->pressure = -1.0f;          // Valor inválido  
        sensor_data->gas_resistance = 0;
        sensor_data->air_quality = 0.0f;
        sensor_data->raw_gas = 0;
        
        return ESP_ERR_NOT_FOUND;
    }
    
    uint8_t sensor_data_raw[15];
    uint8_t gas_range;
    
    // Leer todos los datos del sensor (registros 0x1D a 0x2B)
    esp_err_t ret = bme680_read_bytes(0x1D, sensor_data_raw, 15);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Error leyendo datos del sensor: %s", esp_err_to_name(ret));
        
        // Retornar valores de error pero permitir el envío
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
    
    // Verificar si los valores ADC son razonables (evitar valores extremos por hardware defectuoso)
    if (temp_adc == 0 || temp_adc > 0xFFFFF) { // 20-bit max value
        ESP_LOGW(TAG, "⚠️ Valor ADC de temperatura fuera de rango: %lu", (unsigned long)temp_adc);
        sensor_data->temperature = -999.0f;
    } else {
        // Compensar temperatura
        sensor_data->temperature = bme680_compensate_temperature(temp_adc);
    }
    
    if (press_adc == 0 || press_adc > 0xFFFFF) {
        ESP_LOGW(TAG, "⚠️ Valor ADC de presión fuera de rango: %lu", (unsigned long)press_adc);
        sensor_data->pressure = -1.0f;
    } else {
        // Compensar presión
        sensor_data->pressure = bme680_compensate_pressure(press_adc);
    }
    
    if (hum_adc == 0 || hum_adc > 0xFFFF) {
        ESP_LOGW(TAG, "⚠️ Valor ADC de humedad fuera de rango: %u", hum_adc);
        sensor_data->humidity = -1.0f;
    } else {
        // Compensar humedad
        sensor_data->humidity = bme680_compensate_humidity(hum_adc);
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
    if (sensor_data->temperature > -100.0f) { // Si no es valor de error
        last_sensor_data = *sensor_data;
    }
    
    // Reiniciar medición para siguiente lectura
    esp_err_t write_ret = bme680_write_byte(0x74, 0x93); // Temp x2, Pres x4, Modo Normal
    if (write_ret != ESP_OK) {
        ESP_LOGW(TAG, "⚠️ Error reiniciando medición: %s", esp_err_to_name(write_ret));
    }
    
    ESP_LOGD(TAG, "📊 Datos BME680 procesados - Temp: %.2f°C, Hum: %.2f%%, Pres: %.2fhPa", 
             sensor_data->temperature, sensor_data->humidity, sensor_data->pressure);
    
    return ESP_OK;
}

// ==================== VERIFICACIÓN DE CONEXIÓN ====================
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

// ==================== TAREA DE LECTURA CONTINUA ====================
static void bme680_reading_task(void *pvParameters) {
    bme680_data_t sensor_data;
    
    ESP_LOGI(TAG, "📈 Iniciando lecturas continuas BME680...");
    
    while(1) {
        esp_err_t ret = bme680_read_all_data(&sensor_data);
        
        if (ret == ESP_OK) {
            // ✅ CORREGIDO: Usar PRIu32 para uint32_t
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

// ==================== INICIAR TAREA DE LECTURA ====================
void bme680_start_reading_task(void) {
    xTaskCreate(bme680_reading_task, "bme680_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "🔄 Tarea de lectura BME680 iniciada");
}

// ==================== OBTENER ÚLTIMOS DATOS ====================
bme680_data_t* bme680_get_last_data(void) {
    return &last_sensor_data;
}