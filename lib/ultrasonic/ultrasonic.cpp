#include "ultrasonic.hpp"

static int distance;
static unsigned long sensorTime;

static void IRAM_ATTR triggerInterrupt(void* arg)
{
    if(!gpio_get_level(GPIO_NUM_2)) {
        distance = (esp_timer_get_time() - sensorTime)/58;
        xQueueSendToBack(distanceQueue, &distance, 0);
    }
    else sensorTime = esp_timer_get_time();
}


Ultrasonic::Ultrasonic(gpio_num_t triggerPin, gpio_num_t echoPin, ledc_channel_t pwmChannel) : triggerPin(triggerPin), echoPin(echoPin)
{
    int err = 0;
    int echo_num = echoPin;
    err += gpio_set_direction(this->triggerPin, GPIO_MODE_OUTPUT);
    err += gpio_set_level(this->triggerPin, 0);

    err += gpio_install_isr_service(0);
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pin_bit_mask = (1ULL<<echo_num);
    err += gpio_config(&io_conf);

    err += gpio_isr_handler_add(this->echoPin, triggerInterrupt, (void*) &echo_num);

    ledc_timer.duty_resolution = LEDC_TIMER_15_BIT;
    ledc_timer.freq_hz = 5;
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer.timer_num = LEDC_TIMER_3;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    err += ledc_timer_config(&ledc_timer);

    ledc_channel.channel = pwmChannel;
    ledc_channel.duty = 2;
    ledc_channel.gpio_num = this->triggerPin;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.hpoint     = 0;
    ledc_channel.timer_sel  = LEDC_TIMER_3;
    
    err += ledc_channel_config(&ledc_channel);

    if(err) ESP_LOGE("Ultrasonic", "sensor failed");
    else ESP_LOGE("Ultrasonic", "sensor initialized");
}

Ultrasonic::~Ultrasonic() {}
