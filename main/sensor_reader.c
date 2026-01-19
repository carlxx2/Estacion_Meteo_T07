#include "system_config.h"
#include "driver/adc.h"

static const char *TAG = "SENSOR_READER";



// Rangos de calibración (AJUSTA SEGÚN TU SENSOR)
#define PLUVIOMETRO_MIN_VOLTAGE    0.0f     // 0V = 0mm (seco)
#define PLUVIOMETRO_MAX_VOLTAGE    0.3f     // 3.3V = lluvia máxima
#define PLUVIOMETRO_MAX_MM         100.0f   // Máxima lluvia medible en mm

#define ANEMOMETRO_MIN_VOLTAGE     0.0f     // 0.0V = 0 m/s (calma)
#define ANEMOMETRO_MAX_VOLTAGE     0.3f     // 3.3V = viento máximo
#define ANEMOMETRO_MAX_MS          30.0f    // Máxima velocidad en m/s

// ==================== FUNCIÓN COMÚN ADC ====================

static float read_adc_value(adc1_channel_t channel, const char* sensor_name) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel, ADC_ATTEN_DB_12);
    
    // Promediar lecturas para reducir ruido
    int total = 0;
    for (int i = 0; i < 10; i++) {
        total += adc1_get_raw(channel);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    int raw_value = total / 10;
    
    // Convertir a voltaje (12 bits, 3.3V referencia)
    float voltage = (raw_value / 4095.0) * 3.3;
    
    ESP_LOGD(TAG, "%s - Raw: %d, Voltaje: %.3fV", sensor_name, raw_value, voltage);
    
    return voltage;
}

// ==================== LECTURA PLUVIÓMETRO ====================

float read_pluviometro_value(void) {
    float voltage = read_adc_value(PLUVIOMETRO_ADC_CHANNEL, "Pluviometro");
    
    // Convertir voltaje a mm de lluvia (lineal)
    float rainfall_mm = 0.0;
    
    if (voltage <= PLUVIOMETRO_MIN_VOLTAGE) {
        rainfall_mm = 0.0;
    } else if (voltage >= PLUVIOMETRO_MAX_VOLTAGE) {
        rainfall_mm = PLUVIOMETRO_MAX_MM;
    } else {
        // Interpolación lineal
        rainfall_mm = ((voltage - PLUVIOMETRO_MIN_VOLTAGE) / 
                      (PLUVIOMETRO_MAX_VOLTAGE - PLUVIOMETRO_MIN_VOLTAGE)) * PLUVIOMETRO_MAX_MM;
    }
    
    ESP_LOGI(TAG, "Pluviometro - Voltaje: %.3fV, Lluvia: %.2f mm", voltage, rainfall_mm);
    return rainfall_mm;
}

// ==================== LECTURA ANEMÓMETRO ====================

float read_anemometro_value(void) {
    float voltage = read_adc_value(ANEMOMETRO_ADC_CHANNEL, "Anemometro");
    
    // Convertir voltaje a velocidad del viento (lineal)
    float wind_speed_ms = 0.0;
    
    if (voltage <= ANEMOMETRO_MIN_VOLTAGE) {
        wind_speed_ms = 0.0;
    } else if (voltage >= ANEMOMETRO_MAX_VOLTAGE) {
        wind_speed_ms = ANEMOMETRO_MAX_MS;
    } else {
        // Interpolación lineal
        wind_speed_ms = ((voltage - ANEMOMETRO_MIN_VOLTAGE) / 
                        (ANEMOMETRO_MAX_VOLTAGE - ANEMOMETRO_MIN_VOLTAGE)) * ANEMOMETRO_MAX_MS;
    }
    
    float wind_speed_kmh = wind_speed_ms * 3.6;
    
    ESP_LOGI(TAG, "Anemometro - Voltaje: %.3fV, Velocidad: %.1f m/s (%.1f km/h)", 
             voltage, wind_speed_ms, wind_speed_kmh);
    
    return wind_speed_ms;  // Retorna en m/s
}

// ==================== INICIALIZACIÓN ====================

void init_sensors(void) {
    ESP_LOGI(TAG, "Inicializando sensores meteorológicos ADC...");
    
    // Configurar ADC para ambos sensores
    adc1_config_width(ADC_WIDTH_BIT_12);
    
    // Pluviómetro
    adc1_config_channel_atten(PLUVIOMETRO_ADC_CHANNEL, ADC_ATTEN_DB_11);
    
    // Anemómetro  
    adc1_config_channel_atten(ANEMOMETRO_ADC_CHANNEL, ADC_ATTEN_DB_11);
    
    ESP_LOGI(TAG, "Sensores inicializados:");
    ESP_LOGI(TAG, "Pluviómetro en ADC1_CHANNEL_%d (GPIO32)", PLUVIOMETRO_ADC_CHANNEL);
    ESP_LOGI(TAG, "Anemómetro en ADC1_CHANNEL_%d (GPIO33)", ANEMOMETRO_ADC_CHANNEL);
    
    // Pequeña pausa para estabilización
    vTaskDelay(100 / portTICK_PERIOD_MS);
}