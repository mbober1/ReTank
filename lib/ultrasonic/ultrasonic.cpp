#include "ultrasonic.hpp"

// static xQueueHandle distanceTimerQueue;
static xQueueHandle gpio_evt_queue = NULL;

// static bool IRAM_ATTR timer_group_isr_callback(void *args)
// {
// //     printf("TIMER INT\n");
//     BaseType_t high_task_awoken = pdFALSE;
//     timer_info_t *info = (timer_info_t *) args;

//     uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, TIMER_0);

// //     // /* Prepare basic event data that will be then sent back to task */
// //     // timer_event_t evt = {};

// //     // evt.info.timer_group = info->timer_group;
// //     // evt.info.timer_idx = info->timer_idx;
// //     // evt.timer_counter_value = timer_counter_value;
// //     // timer_pause(info->timer_group, info->timer_idx);

// //     /* Now just send the event data back to the main program task */
// //     xQueueSendFromISR(distanceTimerQueue, &timer_counter_value, &high_task_awoken);

//     return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
// }

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}


Ultrasonic::Ultrasonic(gpio_num_t triggerPin, gpio_num_t echoPin) : triggerPin(triggerPin), echoPin(echoPin)
{
    printf("zaczynam inicializacje\n");

    int err = 0;
    // distanceTimerQueue = xQueueCreate(10, sizeof(uint32_t));
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    printf("1\n");

    err += gpio_set_direction(this->triggerPin, GPIO_MODE_OUTPUT);
    err += gpio_set_direction(this->echoPin, GPIO_MODE_INPUT);
    err += gpio_set_level(this->triggerPin, 0);
    err += gpio_install_isr_service(0);

    gpio_set_intr_type(this->echoPin, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(this->echoPin, gpio_isr_handler, (void*) this->echoPin);
    gpio_intr_enable(this->echoPin);

    printf("2\n");

    /* Select and initialize basic parameters of the timer */
    // timer_config_t config = {};
    // config.divider = 800;
    // config.counter_dir = TIMER_COUNT_UP;
    // config.counter_en = TIMER_PAUSE;
    // config.alarm_en = TIMER_ALARM_EN;
    // config.auto_reload = TIMER_AUTORELOAD_DIS;
    
    // // // default clock source is APB
    // err += timer_init(TIMER_GROUP_0, TIMER_0, &config);

    // printf("3\n");

    // // /* Timer's counter will initially start from value below.
    // //    Also, if auto_reload is set, this value will be automatically reload on alarm */
    // err += timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);

    // printf("4\n");

    // // /* Configure the alarm value and the interrupt on alarm. */
    // err += timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 10);
    // err += timer_enable_intr(TIMER_GROUP_0, TIMER_0);

    // printf("5\n");
    // timer_info_t *timer_info = new timer_info_t;
    // // this->timer_info.timer_group = TIMER_GROUP_0;
    // // this->timer_info.timer_idx = TIMER_0;
    // printf("6\n");

    // timer_info->timer_group = TIMER_GROUP_0;
    // timer_info->timer_idx = TIMER_0;
    // timer_info->auto_reload = TIMER_AUTORELOAD_DIS;
    // timer_info->alarm_interval = 10;
    // ESP_ERROR_CHECK(timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_group_isr_callback, timer_info, 0));

    if(err) printf("Ultrasonic sensor failed\n");
    else printf("Ultrasonic sensor initialized\n");
}

Ultrasonic::~Ultrasonic() {}

// float Ultrasonic::measure() {
//     uint32_t time = this->measureTime();
//     float distance = (float)time / ROUNDTRIP_M;
//     printf("Time: %d, Distance: %f\n", time, distance);
//     return distance;
// }

uint32_t Ultrasonic::measureTime() {
    uint32_t timerValue = 0;

    // printf("zaraz ide czekac\n");
    gpio_set_level(this->triggerPin, 1);

    
    // // timer_start(TIMER_GROUP_0, TIMER_0); //wait 100us
    // // if(xQueueReceive(distanceTimerQueue, &timerValue, TIMEOUT) != pdTRUE) {
    // //     printf("TIMEOUT\n");
    // //     return 0;
    // // } else printf("gites %lld\n", timerValue);
    vTaskDelay(10 / configTICK_RATE_HZ);

    gpio_set_level(this->triggerPin, 0);

    xQueueReset(gpio_evt_queue);
    // // timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    // // timer_start(TIMER_GROUP_0, TIMER_0);
    int32_t start = esp_timer_get_time(); // wait for echo

    if(xQueueReceive(gpio_evt_queue, &timerValue, TIMEOUT) != pdTRUE) {
        printf("TIMEOUT\n");
        return 0;
    }

    // timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &timerValue);
    // while (gpio_get_level(this->echoPin))
    // {
        // if((esp_timer_get_time() - (start)) >= (PING_TIMEOUT)) {
        //     printf("Device is not responding\n");
        //     return 0;
        // }

    // }

    // // got echo, measuring
    int32_t kurwa = esp_timer_get_time();

    
    printf("ESP: %d   Distance: %f\n", (kurwa - start), ((float)(kurwa - start))/58);

    // timer_pause(TIMER_GROUP_0, TIMER_0);
    // timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    // timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, TIMER_0, 10);

    // return start - echo_start;
    return 0;
}

