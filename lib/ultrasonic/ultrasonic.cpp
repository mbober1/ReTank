#include "ultrasonic.hpp"


Ultrasonic::Ultrasonic(gpio_num_t triggerPin, gpio_num_t echoPin) : triggerPin(triggerPin), echoPin(echoPin)
{
    int err = 0;

    err += gpio_set_direction(this->triggerPin, GPIO_MODE_OUTPUT);
    err += gpio_set_direction(this->echoPin, GPIO_MODE_INPUT);
    err += gpio_set_level(this->triggerPin, 0);

    if(err) ESP_LOGE("Ultrasonic", "sensor failed");
    else ESP_LOGE("Ultrasonic", "sensor initialized");
}

Ultrasonic::~Ultrasonic() {}


uint Ultrasonic::measure() {

    gpio_set_level(this->triggerPin, 0);
    ets_delay_us(4);
    gpio_set_level(this->triggerPin, 1);
    ets_delay_us(10);
    gpio_set_level(this->triggerPin, 0);


    int32_t start = esp_timer_get_time(); // wait for echo

    while (!gpio_get_level(this->echoPin))
    {
        if (esp_timer_get_time() - (start) > pingTIMEOUT) return 0;
    }

    int32_t stop = esp_timer_get_time();

    while (gpio_get_level(this->echoPin))
    {
        if (esp_timer_get_time() - (start) > echoTIMEOUT) return 0;
    }

    uint distance = (stop - start) / timeToCM;
    return distance;
}