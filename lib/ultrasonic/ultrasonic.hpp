#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include "esp_log.h"

const uint32_t pingTIMEOUT = 20000;
const uint32_t echoTIMEOUT = 70000;
const uint8_t timeToCM = 58;

class Ultrasonic {

    gpio_num_t triggerPin, echoPin;

public:
    Ultrasonic(gpio_num_t triggerPin, gpio_num_t echoPin);
    ~Ultrasonic();
    uint measure();
};


