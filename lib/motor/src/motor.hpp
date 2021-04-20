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
    gpio_num_t in1, in2, pwmPin;
    int kp, ki, kd;
    int64_t previousTime;
    int32_t epsilonOld, epsilonSuma, integralError, derivativeError;
    ledc_channel_config_t ledc_channel = {};
    ledc_timer_config_t ledc_timer = {};
    
public:
    pcnt_unit_t encoder;
    motor(gpio_num_t in1, gpio_num_t in2, gpio_num_t encoderA, gpio_num_t encoderB, gpio_num_t pwmPin, ledc_channel_t pwmChannel, pcnt_unit_t pcntUnit);
    void direction(const Direction &dir);
    void power(const uint32_t &pow);
    void compute(uint32_t &pow, int8_t &direction, const int &setpoint);
    void fastStop();
    void softStop();
    void drive(const int &setpoint);
};

