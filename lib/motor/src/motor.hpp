#pragma once
#include <stdint.h>
#include "soc/ledc_struct.h"
#include "esp32-hal-gpio.h"


enum class Direction {
    FORWARD,
    BACKWARD
};


class motor
{
private:
    uint8_t in1, in2, pwmPin, pwmChannel;
public:
    motor(uint8_t in1, uint8_t in2, uint8_t pwmPin, uint8_t pwmChannel);
    void direction(Direction dir);
    void power(uint8_t pow);
    void fastStop();
    void softStop();
};


