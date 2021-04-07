#pragma once
#include <stdint.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "math.h"

extern QueueHandle_t speedQueue;

enum class Direction {
    FORWARD,
    BACKWARD
};


#define MAX_POWER (1<<16)
#define MAX_INTEGRAL 100

class motor
{
private:
    gpio_num_t in1, in2;
    uint8_t pwmPin, pwmChannel, encoderA, encoderB;
    int kp, ki, kd;
    int64_t previousTime, lastError, integralError, derivativeError;
    ledc_channel_config_t ledc_channel = {};
    ledc_timer_config_t ledc_timer = {};
    
public:
    int setpoint;
    pcnt_unit_t encoder;
    motor(gpio_num_t in1, gpio_num_t in2, uint8_t pwmPin, uint8_t encoderA, uint8_t encoderB, uint8_t pwmChannel, pcnt_unit_t pcntUnit);
    void direction(const Direction &dir);
    void power(const uint32_t &pow);
    void compute(const int16_t &input, uint32_t &pow, int8_t &direction);
    void fastStop();
    void softStop();
    void drive(const int16_t &input);
};

