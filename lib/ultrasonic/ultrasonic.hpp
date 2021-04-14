#pragma once
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/queue.h"

class Ultrasonic {
    gpio_num_t triggerPin, echoPin;
    ledc_channel_config_t ledc_channel = {};
    ledc_timer_config_t ledc_timer = {};

public:
    Ultrasonic(gpio_num_t triggerPin, gpio_num_t echoPin, ledc_channel_t pwmChannel);
    ~Ultrasonic();
};
