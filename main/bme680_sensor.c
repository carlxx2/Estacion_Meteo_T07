#include "system_config.h"

static const char *TAG = "BME680_SENSOR";

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

// ==================== FUNCIONES I2C BÁSICAS ====================
static esp_err_t bme680_write_byte(uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME680_ADDR << 1) | I2C_MASTER_WRITE, true);
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
    i2c_master_write_byte(cmd, (BME680_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME680_ADDR << 1) | I2C_MASTER_READ, true);
    
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
    float var1, var2, var3, var4, var5, var6, temp_comp;
    
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
    ESP_LOGI(TAG, "🔧 Inicializando BME680...");
    
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
    
    ESP_LOGI(TAG, "✅ I2C inicializado correctamente");
    
    // Pequeña pausa para estabilización
    vTaskDelay(1000 / portTICK_PERIOD_MS);
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

// ==================== LECTURA DE TODOS LOS DATOS ====================
esp_err_t bme680_read_all_data(bme680_data_t *sensor_data) {
    uint8_t sensor_data_raw[15];
    uint8_t gas_range;
    
    // Leer todos los datos del sensor
    esp_err_t ret = bme680_read_bytes(0x1D, sensor_data_raw, 15);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Error leyendo datos del sensor");
        return ret;
    }
    
    // Extraer valores ADC
    uint32_t temp_adc = (uint32_t)((sensor_data_raw[5] << 12) | (sensor_data_raw[6] << 4) | (sensor_data_raw[7] >> 4));
    uint32_t press_adc = (uint32_t)((sensor_data_raw[2] << 12) | (sensor_data_raw[3] << 4) | (sensor_data_raw[4] >> 4));
    uint16_t hum_adc = (uint16_t)((sensor_data_raw[8] << 8) | sensor_data_raw[9]);
    uint16_t gas_adc = (uint16_t)((sensor_data_raw[13] << 2) | (sensor_data_raw[14] >> 6));
    gas_range = sensor_data_raw[14] & 0x0F;
    
    // Compensar valores
    sensor_data->temperature = bme680_compensate_temperature(temp_adc);
    sensor_data->pressure = bme680_compensate_pressure(press_adc);
    sensor_data->humidity = bme680_compensate_humidity(hum_adc);
    sensor_data->gas_resistance = bme680_compensate_gas(gas_adc, gas_range);
    sensor_data->raw_gas = gas_adc;
    sensor_data->air_quality = bme680_calculate_air_quality(sensor_data->gas_resistance, sensor_data->humidity);
    
    // Guardar última lectura
    last_sensor_data = *sensor_data;
    
    // Reiniciar medición
    ESP_ERROR_CHECK(bme680_write_byte(0x74, 0x93));
    
    return ESP_OK;
}

// ==================== VERIFICACIÓN DE CONEXIÓN ====================
bool bme680_is_connected(void) {
    uint8_t chip_id;
    esp_err_t ret = bme680_read_bytes(0xD0, &chip_id, 1);
    return (ret == ESP_OK && chip_id == 0x61);
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