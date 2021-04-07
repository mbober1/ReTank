#include "ultrasonic.hpp"

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

Ultrasonic::Ultrasonic(gpio_num_t triggerPin, gpio_num_t echoPin) : triggerPin(triggerPin), echoPin(echoPin)
{
    int err = 0;
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    err += gpio_set_direction(this->triggerPin, GPIO_MODE_OUTPUT);
    err += gpio_set_direction(this->echoPin, GPIO_MODE_INPUT);
    err += gpio_set_level(this->triggerPin, 0);
    // err += gpio_install_isr_service(0);

    err += gpio_set_intr_type(this->echoPin, GPIO_INTR_POSEDGE);
    err += gpio_isr_handler_add(this->echoPin, gpio_isr_handler, (void*) this->echoPin);
    err += gpio_intr_enable(this->echoPin);


    if(err) printf("Ultrasonic sensor failed\n");
    else printf("Ultrasonic sensor initialized\n");
}

Ultrasonic::~Ultrasonic() {}

uint Ultrasonic::measure() {
    uint32_t time = this->measureTime();
    uint distance = time / ROUNDTRIP_M;
    return distance;
}

uint32_t Ultrasonic::measureTime() {
    timer_event_t evt;

    gpio_set_level(this->triggerPin, 1);

    vTaskDelay(10 / configTICK_RATE_HZ);

    gpio_set_level(this->triggerPin, 0);

    xQueueReset(gpio_evt_queue);

    int32_t start = esp_timer_get_time(); // wait for echo

    if(xQueueReceive(gpio_evt_queue, &evt, TIMEOUT) != pdTRUE) {
        printf("TIMEOUT\n");
        return 0;
    }
    
    int32_t stop = esp_timer_get_time();

    return (stop - start);
}

