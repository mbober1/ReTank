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
    pcnt_config_t pcnt_config = {};
    pcnt_config.pulse_gpio_num = this->encoderA;
    pcnt_config.ctrl_gpio_num = this->encoderB;
    pcnt_config.channel = PCNT_CHANNEL_0;
    pcnt_config.unit = this->encoder;
    pcnt_config.pos_mode = PCNT_COUNT_INC;
    pcnt_config.neg_mode = PCNT_COUNT_INC;
    pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config.counter_h_lim = (1000);
    pcnt_config.counter_l_lim = 0;
    pcnt_unit_config(&pcnt_config);
    pcnt_counter_pause(this->encoder);
    pcnt_counter_clear(this->encoder);
    pcnt_filter_disable(this->encoder);
    pcnt_intr_disable(this->encoder);
    pcnt_counter_resume(this->encoder);
    // pcnt_set_mode(this->encoder, PCNT_CHANNEL_0, PCNT_COUNT_INC, PCNT_COUNT_INC, PCNT_MODE_KEEP, PCNT_MODE_KEEP);

    this->encoder2 = pcntUnit;
//␛[0;31mE (314) ledc: ledc_channel_config(380): gpio_num argument is invalid␛[0m
// ␛[0;31mE (324) pcnt: _pcnt_set_event_value(197): PCNT limit value error␛[0m
    pcnt_config_t pcnt_config2 = {};
    pcnt_config2.pulse_gpio_num = this->encoderB;
    pcnt_config2.ctrl_gpio_num = this->encoderA;
    pcnt_config2.channel = PCNT_CHANNEL_1;
    pcnt_config2.unit = this->encoder2;
    pcnt_config2.pos_mode = PCNT_COUNT_INC;
    pcnt_config2.neg_mode = PCNT_COUNT_INC;
    pcnt_config2.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config2.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config.counter_h_lim = (1000);
    pcnt_config.counter_l_lim = 0;
    pcnt_unit_config(&pcnt_config2);
    pcnt_counter_pause(this->encoder2);
    pcnt_counter_clear(this->encoder2);
    pcnt_filter_disable(this->encoder2);
    pcnt_intr_disable(this->encoder2);
    pcnt_counter_resume(this->encoder2);

    // pcnt_set_mode(this->encoder2, PCNT_CHANNEL_0, PCNT_COUNT_INC, PCNT_COUNT_INC, PCNT_MODE_KEEP, PCNT_MODE_KEEP);


    softStop();

    this->integralError = 0;
    this->lastError = 0;

}


inline void motor::direction(const Direction &dir) {
    if(static_cast<uint8_t>(dir)) {
        gpio_set_level(this->in1, 1);
        gpio_set_level(this->in2, 0);
    } else {
        gpio_set_level(this->in1, 0);
        gpio_set_level(this->in2, 1);
    }
}

inline void motor::fastStop() {
    gpio_set_level(this->in1, 1);
    gpio_set_level(this->in2, 1);
}

inline void motor::softStop() {
    gpio_set_level(this->in1, 0);
    gpio_set_level(this->in2, 0);
}



inline void motor::power(const uint16_t &pow) {
    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, pow);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
}



inline void motor::compute(uint16_t &pow) {
    int64_t currentTime = esp_timer_get_time();
    int64_t elapsedTime = currentTime - this->previousTime;
    int16_t input;
    pcnt_get_counter_value(encoder, &input);
    xQueueSendToBack(speedQueue, &input, 0);

    this->kp = 1;
    this->ki = 0.01;
    this->kd = 0.1;

    int error = this->setpoint - input;
    this->integralError += error * elapsedTime; //calka
    if(this->integralError > 200) this->integralError = 400;
    if(this->integralError < -200) this->integralError = -400;
    this->derivativeError = (error - this->lastError)/elapsedTime; //pochodna
    this->lastError = error;
    this->previousTime = currentTime;
    pcnt_counter_clear(encoder);
    pow = kp*error + ki*this->integralError + kd*this->derivativeError;    
}

void motor::drive() {
    uint16_t val;
    this->compute(val);
    this->power(val);
    if(this->setpoint > 0) this->direction(Direction::FORWARD);
    else if(this->setpoint < 0) this->direction(Direction::BACKWARD);
}

