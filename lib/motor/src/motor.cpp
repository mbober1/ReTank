#include "motor.hpp"


motor::motor(gpio_num_t in1, gpio_num_t in2, uint8_t pwmPin, uint8_t encoderA, uint8_t encoderB, uint8_t pwmChannel, pcnt_unit_t pcntUnit) : in1(in1), in2(in2), pwmPin(pwmPin), pwmChannel(pwmChannel), encoderA(encoderA), encoderB(encoderB) {
    gpio_pad_select_gpio(this->in1);
    gpio_pad_select_gpio(this->in2);
    gpio_set_direction(this->in1, GPIO_MODE_OUTPUT);
    gpio_set_direction(this->in2, GPIO_MODE_OUTPUT);

    ledc_timer.duty_resolution = LEDC_TIMER_16_BIT;
    ledc_timer.freq_hz = 1220;
    ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_timer.timer_num = LEDC_TIMER_0;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    ledc_channel.channel = LEDC_CHANNEL_0;
    ledc_channel.duty = 0;
    ledc_channel.gpio_num = this->pwmPin;
    ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_channel.hpoint     = 0;
    ledc_channel.timer_sel  = LEDC_TIMER_0;
    
    ledc_channel_config(&ledc_channel);

    this->encoder = pcntUnit;
    pcnt_config_t pcnt_config;
    pcnt_config.pulse_gpio_num = this->encoderA;
    pcnt_config.ctrl_gpio_num = this->encoderB;
    pcnt_config.channel = PCNT_CHANNEL_0;
    pcnt_config.unit = this->encoder;
    pcnt_config.pos_mode = PCNT_COUNT_INC;
    pcnt_config.neg_mode = PCNT_COUNT_DIS;
    pcnt_config.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_unit_config(&pcnt_config);
    // pcnt_set_filter_value(this->encoder, 100);
    pcnt_filter_enable(this->encoder);
    pcnt_counter_pause(this->encoder);
    pcnt_counter_clear(this->encoder);
    pcnt_counter_resume(this->encoder);

    softStop();
}


void motor::direction(Direction dir) {
    if(static_cast<uint8_t>(dir)) {
        gpio_set_level(this->in1, 1);
        gpio_set_level(this->in2, 0);
    } else {
        gpio_set_level(this->in1, 0);
        gpio_set_level(this->in2, 1);
    }
}

void motor::fastStop() {
    gpio_set_level(this->in1, 1);
    gpio_set_level(this->in2, 1);
}

void motor::softStop() {
    gpio_set_level(this->in1, 0);
    gpio_set_level(this->in2, 0);
}



void motor::power(uint16_t pow) {
    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, pow);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
}



int motor::compute() {
    int64_t currentTime = esp_timer_get_time();
    int64_t elapsedTime = currentTime - this->previousTime;
    int16_t input;
    pcnt_get_counter_value(encoder, &input);
    double kp = 1;
    double ki = 0;
    double kd = 0;

    int error = this->setpoint - input;
    this->integralError += error * elapsedTime; //calka
    this->derivativeError = (error - this->lastError)/elapsedTime; //pochodna
    //dodać ograniczenie na całkę
    //dodać ograniczenie na wyjście
    this->lastError = error;
    this->previousTime = currentTime;
    pcnt_counter_clear(encoder);
    return kp*error + ki*this->integralError + kd*this->derivativeError;    
}
