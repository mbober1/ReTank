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
    pcnt_config.pos_mode = PCNT_COUNT_DEC;
    pcnt_config.neg_mode = PCNT_COUNT_INC;
    pcnt_config.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config.counter_h_lim = 1000;
    pcnt_config.counter_l_lim = -1000;
    esp_err_t err = pcnt_unit_config(&pcnt_config);

    pcnt_config_t pcnt_config2 = {};
    pcnt_config2.pulse_gpio_num = this->encoderB;
    pcnt_config2.ctrl_gpio_num = this->encoderA;
    pcnt_config2.channel = PCNT_CHANNEL_1;
    pcnt_config2.unit = this->encoder;
    pcnt_config2.pos_mode = PCNT_COUNT_INC;
    pcnt_config2.neg_mode = PCNT_COUNT_DEC;
    pcnt_config2.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config2.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config2.counter_h_lim = 1000;
    pcnt_config2.counter_l_lim = -1000;
    err += pcnt_unit_config(&pcnt_config2);

    err += pcnt_counter_pause(PCNT_UNIT_0);
    err += pcnt_counter_clear(PCNT_UNIT_0);
    err += pcnt_filter_disable(PCNT_UNIT_0);
    err += pcnt_intr_disable(PCNT_UNIT_0);
    err +=  pcnt_counter_resume(PCNT_UNIT_0);

    printf("Encoder initialized %d", err);

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



inline void motor::power(const uint32_t &pow) {
    if(pow > MAX_POWER) ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, MAX_POWER);
    else ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, pow);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
}


void motor::compute(uint32_t &pow, int8_t &direction, const int &setpoint) {
    int64_t currentTime = esp_timer_get_time();
    int64_t elapsedTime = currentTime - this->previousTime;
    int16_t input;
    pcnt_get_counter_value(this->encoder, &input);
    pcnt_counter_clear(this->encoder);
    // xQueueSendToBack(speedQueue, &input[2], 0);

    int error = setpoint/2 - input;
   

    this->integralError += error * elapsedTime; //calka
    if(this->integralError > MAX_INTEGRAL) this->integralError = MAX_INTEGRAL;
    if(this->integralError < -MAX_INTEGRAL) this->integralError = -MAX_INTEGRAL;
    this->derivativeError = (error - this->lastError)/elapsedTime; //pochodna
    this->lastError = error;
    this->previousTime = currentTime;


    this->kp = 4000;
    this->ki = 250;
    this->kd = 0;

    int p = kp*error;
    int i = ki*this->integralError;
    int d = kd*this->derivativeError;


    int32_t pid = p + i + d;
    

    pow = abs(pid); 
    if(error > 0) direction = 1;
    else if(error < 0) direction = -1;
    else direction = 0;

    // printf("Error: %+4d, Input1: %+3d, P: %7d + I: %7d + D: %7d = PID: %7d --> ", error, input, p, i, d, pid);
}

void motor::drive(const int &setpoint) {
    int8_t dir;
    uint32_t pow;
    this->compute(pow, dir, setpoint);
    this->power(pow);

    if(dir > 0) {
        dir = 1; 
        this->direction(Direction::FORWARD);
    }
    else if(dir < 0) {
        dir = -1;
        this->direction(Direction::BACKWARD);
    }

    if(setpoint == 0) {
        this->softStop();
        dir = 0;
    }

    // printf("Setpoint: %+4d, Power: %d%%, Direction: %d\n", this->setpoint, (pow*100)/MAX_POWER, dir);
}

