#pragma once
#include <stdint.h>
#include "soc/ledc_struct.h"
#include "esp32-hal-gpio.h"


enum class Direction {
    FORWARD,
    BACKWARD
};

#define MAX (2<<16)
#define MIN -(2<<16)

class motor
{
private:
    uint8_t in1, in2, pwmPin, pwmChannel;
    uint16_t setpoint;
    double kp, ki, kd;
    int64_t previousTime, lastError, integralError, derivativeError;

public:
    motor(uint8_t in1, uint8_t in2, uint8_t pwmPin, uint8_t pwmChannel);
    void direction(Direction dir);
    void power(uint16_t pow);
    void fastStop();
    void softStop();
    double compute(uint16_t input);
};

