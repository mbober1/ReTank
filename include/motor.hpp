#pragma once
#include <stdint.h>
#include "soc/ledc_struct.h"


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


motor::motor(uint8_t in1, uint8_t in2, uint8_t pwmPin, uint8_t pwmChannel) : in1(in1), in2(in2), pwmPin(pwmPin), pwmChannel(pwmChannel) {
    pinMode(this->in1, OUTPUT);
    pinMode(this->in2, OUTPUT);
    pinMode(this->pwmPin, OUTPUT);
    ledcSetup(this->pwmChannel, 2000, 8);
    gpio_matrix_out(this->pwmPin, LEDC_HS_SIG_OUT0_IDX + this->pwmChannel, false, false); //attach pin
    softStop();
}


void motor::direction(Direction dir) {
    if(static_cast<uint8_t>(dir)) {
        GPIO.out_w1ts = ((uint32_t)1 << in1);
        GPIO.out_w1tc = ((uint32_t)1 << in2);
    } else {
        GPIO.out_w1tc = ((uint32_t)1 << in1);
        GPIO.out_w1ts = ((uint32_t)1 << in2);
    }
}

void motor::fastStop() {
    GPIO.out_w1ts = ((uint32_t)1 << in1);
    GPIO.out_w1ts = ((uint32_t)1 << in2);
}

void motor::softStop() {
    GPIO.out_w1tc = ((uint32_t)1 << in1);
    GPIO.out_w1tc = ((uint32_t)1 << in2);
}



void motor::power(uint8_t pow) {
    LEDC.channel_group[0].channel[this->pwmChannel].duty.duty = pow << 4;//25 bit (21.4)
    if(pow) {
        LEDC.channel_group[0].channel[this->pwmChannel].conf0.sig_out_en = 1;//This is the output enable control bit for channel
        LEDC.channel_group[0].channel[this->pwmChannel].conf1.duty_start = 1;//When duty_num duty_cycle and duty_scale has been configured. these register won't take effect until set duty_start. this bit is automatically cleared by hardware.
        LEDC.channel_group[0].channel[this->pwmChannel].conf0.clk_en = 1;
    } else {
        LEDC.channel_group[0].channel[this->pwmChannel].conf0.sig_out_en = 0;//This is the output enable control bit for channel
        LEDC.channel_group[0].channel[this->pwmChannel].conf1.duty_start = 0;//When duty_num duty_cycle and duty_scale has been configured. these register won't take effect until set duty_start. this bit is automatically cleared by hardware.
        LEDC.channel_group[0].channel[this->pwmChannel].conf0.clk_en = 0;
    }
}

