#include "motor.hpp"


motor::motor(uint8_t in1, uint8_t in2, uint8_t pwmPin, uint8_t pwmChannel) : in1(in1), in2(in2), pwmPin(pwmPin), pwmChannel(pwmChannel) {
    pinMode(this->in1, OUTPUT);
    pinMode(this->in2, OUTPUT);
    pinMode(this->pwmPin, OUTPUT);
    ledcSetup(this->pwmChannel, 2000, 16); //16bit 2kHz
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



void motor::power(uint16_t pow) {
    LEDC.channel_group[0].channel[this->pwmChannel].duty.duty = pow << 4;
    if(pow) {
        LEDC.channel_group[0].channel[this->pwmChannel].conf0.sig_out_en = 1;
        LEDC.channel_group[0].channel[this->pwmChannel].conf1.duty_start = 1;
        LEDC.channel_group[0].channel[this->pwmChannel].conf0.clk_en = 1;
    } else {
        LEDC.channel_group[0].channel[this->pwmChannel].conf0.sig_out_en = 0;
        LEDC.channel_group[0].channel[this->pwmChannel].conf1.duty_start = 0;
        LEDC.channel_group[0].channel[this->pwmChannel].conf0.clk_en = 0;
    }
}



double motor::compute(uint16_t input) {
    int64_t currentTime = esp_timer_get_time();
    int64_t elapsedTime = currentTime - previousTime;

    int error = setpoint - input;
    integralError += error * elapsedTime; //calka
    derivativeError = (error - lastError)/elapsedTime; //pochodna
    //dodać ograniczenie na całkę
    //dodać ograniczenie na wyjście
    lastError = error;
    previousTime = currentTime;
    return kp*error + ki*integralError + kd*derivativeError;    
}
