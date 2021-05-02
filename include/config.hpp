#pragma once
#define INCLUDE_vTaskSuspend 1

#include <string>
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_pm.h"
#include "driver/pcnt.h"

// module includes
#include <tcp.hpp>
#include <wifi.hpp>
#include <ultrasonic.hpp>
#include <packet.hpp>

// watchdog includes
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

// UDP/TCP config
static int UDP_PORT = 8090;
static int TCP_PORT = 8091;

// motors config
const gpio_num_t ENC1A = GPIO_NUM_32;
const gpio_num_t ENC1B = GPIO_NUM_33;
const gpio_num_t ENC2A = GPIO_NUM_16;
const gpio_num_t ENC2B = GPIO_NUM_34;

const gpio_num_t PWM1 = GPIO_NUM_12;
const gpio_num_t PWM2 = GPIO_NUM_26;
const gpio_num_t IN1 = GPIO_NUM_13;
const gpio_num_t IN2 = GPIO_NUM_14;
const gpio_num_t IN3 = GPIO_NUM_27;
const gpio_num_t IN4 = GPIO_NUM_25;
const pcnt_unit_t PCNT1 = PCNT_UNIT_0;
const pcnt_unit_t PCNT2 = PCNT_UNIT_1;
const ledc_channel_t MOTOR_PWM = LEDC_CHANNEL_0;

// ultrasonic sensor config
const gpio_num_t TRIG = GPIO_NUM_4;
const gpio_num_t ECHO = GPIO_NUM_2;
const ledc_channel_t SENSOR_PWM = LEDC_CHANNEL_6;

// i2c config
const gpio_num_t SDA = GPIO_NUM_21;
const gpio_num_t SCL = GPIO_NUM_22;
const uint32_t CLOCK = 100000U;


// queues init
QueueHandle_t engineQueue, batteryQueue, distanceQueue;
QueueHandle_t accelQueue, gyroQueue, speedQueue;