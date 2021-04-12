#include "adc.hpp"

myADC::myADC() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_2_5);
}

float myADC::getVoltage() {
    uint32_t adc_reading = 0;
    
    for (int i = 0; i < 1000; i++) adc_reading += adc1_get_raw(ADC1_CHANNEL_0);
    return ((float)adc_reading / 1000) * this->conversion_factor;
}

int myADC::getPercentage() {
    float voltage = this->getVoltage();
    int percentage = (voltage - 11.1)*66;
    if(percentage < 0) return 0;
    else if(percentage > 100) return 100;
    return percentage;
}