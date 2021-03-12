#pragma once
#include <stdint.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"


enum class Direction {
    FORWARD,
    BACKWARD
};

#define MAX (2<<16)
#define MIN -(2<<16)

class motor
{
private:
    gpio_num_t in1, in2;
    uint8_t pwmPin, pwmChannel, encoderA, encoderB;
    // double kp, ki, kd;
    int64_t previousTime, lastError, integralError, derivativeError;
    ledc_channel_config_t ledc_channel;
    ledc_timer_config_t ledc_timer;
    
public:
    uint16_t setpoint;
    pcnt_unit_t encoder;
    motor(gpio_num_t in1, gpio_num_t in2, uint8_t pwmPin, uint8_t encoderA, uint8_t encoderB, uint8_t pwmChannel, pcnt_unit_t pcntUnit);
    void direction(Direction dir);
    void power(uint16_t pow);
    void fastStop();
    void softStop();
    int compute();
};

