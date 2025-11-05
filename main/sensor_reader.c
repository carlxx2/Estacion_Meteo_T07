#include "system_config.h"

static const char *TAG = "SENSOR_READER";

float read_ldr_value(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LDR_ADC_CHANNEL, ADC_ATTEN_DB_11);
    
    int total = 0;
    for (int i = 0; i < 10; i++) {
        total += adc1_get_raw(LDR_ADC_CHANNEL);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    int raw_value = total / 10;
    float percentage = (raw_value / 4095.0) * 100.0;
    
    ESP_LOGI(TAG, "💡 LDR - Raw: %d, Porcentaje: %.2f%%", raw_value, percentage);
    return percentage;
}

void init_sensors(void) {
    ESP_LOGI(TAG, "🔧 Inicializando sensores...");
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LDR_ADC_CHANNEL, ADC_ATTEN_DB_11);
}