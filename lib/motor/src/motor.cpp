#include "motor.hpp"


/**
 * Constructor.
 * @param in1 input1 GPIO pin.
 * @param in2 input2 GPIO pin.
 * @param encoderA encoder first GPIO pin.
 * @param encoderB encoder second GPIO pin.
 * @param pwmPin enable GPIO pin.
 * @param pwmChannel PWM channel.
 * @param pcntUnit PCNT unit.
 */
motor::motor(gpio_num_t in1, gpio_num_t in2, gpio_num_t encoderA, gpio_num_t encoderB, gpio_num_t pwmPin, ledc_channel_t pwmChannel, pcnt_unit_t pcntUnit) : in1(in1), in2(in2), pwmPin(pwmPin) {
    esp_err_t err;

    gpio_config_t in1_conf = {};
    in1_conf.mode = GPIO_MODE_OUTPUT;
    in1_conf.pin_bit_mask = (1ULL<<in1);
    err = gpio_config(&in1_conf);

    gpio_config_t in2_conf = {};
    in2_conf.mode = GPIO_MODE_OUTPUT;
    in2_conf.pin_bit_mask = (1ULL<<in2);
    err += gpio_config(&in2_conf);

    if(!err) printf("Motor %d GPIO initialized\n", pcntUnit);
    else printf("Motor %d GPIO failed with error: %d\n", pcntUnit, err);

    ledc_timer.duty_resolution = LEDC_TIMER_10_BIT;
    ledc_timer.freq_hz = 25000;
    ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_timer.timer_num = LEDC_TIMER_0;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    err = ledc_timer_config(&ledc_timer);

    ledc_channel.channel = pwmChannel;
    ledc_channel.duty = 0;
    ledc_channel.gpio_num = pwmPin;
    ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_channel.hpoint     = 0;
    ledc_channel.timer_sel  = LEDC_TIMER_0;
    err += ledc_channel_config(&ledc_channel);


    if(!err) printf("Motor %d PWM initialized, freq %d\n", pcntUnit, ledc_get_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0));
    else printf("Encoder %d PWM failed with error: %d\n", pcntUnit, err);

    printf("Encoder %d Config: \n", pcntUnit);
    printf("- ENCA = %d\n", encoderA);
    printf("- ENCB = %d\n", encoderB);

    pcnt_config_t pcnt_config = {};
    pcnt_config.pulse_gpio_num = encoderA;
    pcnt_config.ctrl_gpio_num = encoderB;
    pcnt_config.channel = PCNT_CHANNEL_0;
    pcnt_config.unit = pcntUnit;
    pcnt_config.pos_mode = PCNT_COUNT_DEC;
    pcnt_config.neg_mode = PCNT_COUNT_INC;
    pcnt_config.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config.counter_h_lim = 1000;
    pcnt_config.counter_l_lim = -1000;
    err = pcnt_unit_config(&pcnt_config);

    pcnt_config_t pcnt_config2 = {};
    pcnt_config2.pulse_gpio_num = encoderB;
    pcnt_config2.ctrl_gpio_num = encoderA;
    pcnt_config2.channel = PCNT_CHANNEL_1;
    pcnt_config2.unit = pcntUnit;
    pcnt_config2.pos_mode = PCNT_COUNT_INC;
    pcnt_config2.neg_mode = PCNT_COUNT_DEC;
    pcnt_config2.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config2.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config2.counter_h_lim = 1000;
    pcnt_config2.counter_l_lim = -1000;
    err += pcnt_unit_config(&pcnt_config2);

    err += pcnt_counter_pause(pcntUnit);
    err += pcnt_counter_clear(pcntUnit);
    err += pcnt_filter_disable(pcntUnit);
    err += pcnt_intr_disable(pcntUnit);
    err +=  pcnt_counter_resume(pcntUnit);

    if(!err) printf("Encoder %d initialized\n", pcntUnit);
    else printf("Encoder %d failed with error: %d\n", pcntUnit, err);

    softStop();

    this->encoder = pcntUnit;
    this->integralError = 0;
    this->epsilonOld = 0;
    this->epsilonSuma = 0;
    this->derivativeError = 0;
}

/**
 * Set motor direction.
 * @param dir direction
 */
inline void motor::direction(const Direction &dir) {
    if(static_cast<uint8_t>(dir)) {
        gpio_set_level(this->in1, 1);
        gpio_set_level(this->in2, 0);
    } else {
        gpio_set_level(this->in1, 0);
        gpio_set_level(this->in2, 1);
    }
}

/**
 * Set fast stop mode.
 */
inline void motor::fastStop() {
    gpio_set_level(this->in1, 1);
    gpio_set_level(this->in2, 1);
}

/**
 * Set soft stop mode.
 */
inline void motor::softStop() {
    gpio_set_level(this->in1, 0);
    gpio_set_level(this->in2, 0);
}

/**
 * Set motor power.
 * @param power Motor power [0, (2**duty_resolution)]
 */
inline void motor::power(const uint16_t &pow) {
    ledc_set_duty(this->ledc_channel.speed_mode, this->ledc_channel.channel, pow);
    ledc_update_duty(this->ledc_channel.speed_mode, this->ledc_channel.channel);
}

/**
 * Compute PID.
 * @param setpoint Setpoint
 */
void motor::compute(const int &setpoint) {
    static uint16_t count;
    count++;
    int16_t input;
    pcnt_get_counter_value(this->encoder, &input);
    pcnt_counter_clear(this->encoder);

    int epsilon = setpoint/2 - input;
   
    this->integralError+= epsilon;
    if(this->integralError > MAX_INTEGRAL) this->integralError = MAX_INTEGRAL;
    else if(this->integralError < -MAX_INTEGRAL) this->integralError = -MAX_INTEGRAL;
    this->derivativeError = epsilon - epsilonOld;

    this->kp = 40;
    this->ki = 6;
    this->kd = 2;

    int p = kp*epsilon;
    int i = ki*this->integralError;
    int d = kd*this->derivativeError;


    int32_t pid = p + i + d;
    

    uint16_t pow = abs(pid); 
    if(pow > MAX_POWER) pow = MAX_POWER;
    if(pid > 0) this->direction(Direction::FORWARD);
    else if(pid < 0) this->direction(Direction::BACKWARD);
    else this->softStop();

    this->power(pow);

    epsilonOld = epsilon;

    // printf("Motor %d -> Error: %+4d, Input1: %+3d, P: %7d + I: %7d + D: %7d = PID: %7d power: %d\n", this->encoder, epsilon, input, p, i, d, pid, pow);

    if(count >= 1000) {
        count = 0;

        input = abs(input) * speedVar;
        xQueueSendToBack(speedQueue, &input, 0);
    }
}


