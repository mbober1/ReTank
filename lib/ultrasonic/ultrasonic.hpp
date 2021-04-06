#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/timer.h"
#include "freertos/queue.h"
#include "esp_err.h"


#define ROUNDTRIP_M 58
#define TRIGGER_LOW_DELAY 4
#define TRIGGER_HIGH_DELAY 10
#define PING_TIMEOUT 60000
#define max_time_us 60000

const uint32_t TIMEOUT = pdMS_TO_TICKS(100);

typedef struct {
    int timer_group;
    int timer_idx;
    int alarm_interval;
    bool auto_reload;
} timer_info_t;


typedef struct {
    timer_info_t info;
    uint64_t timer_counter_value;
} timer_event_t;


class Ultrasonic
{
private:
    timer_info_t timer_info;
    gpio_num_t triggerPin, echoPin;
    timer_group_t group_num;
    timer_idx_t timer_num;

public:
    Ultrasonic(gpio_num_t triggerPin, gpio_num_t echoPin);
    ~Ultrasonic();
    float measure();
    uint32_t measureTime();
};


