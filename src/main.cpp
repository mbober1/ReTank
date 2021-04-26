#define INCLUDE_vTaskSuspend 1

#include <string>
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_pm.h"

#include <udp.hpp>
#include <tcp.hpp>
#include <wifi.hpp>
#include <robot.hpp>
#include <I2Cbus.hpp>
#include <MPU.hpp>
#include "mpu/math.hpp"
#include "mpu/types.hpp"
#include <adc.hpp>
#include <ultrasonic.hpp>

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

//UDP/TCP config
static int UDP_PORT = 8090;
static int TCP_PORT = 8091;

//motors config
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

//ultrasonic sensor config
const gpio_num_t TRIG = GPIO_NUM_4;
const gpio_num_t ECHO = GPIO_NUM_2;
const ledc_channel_t SENSOR_PWM = LEDC_CHANNEL_6;

//i2c config
const gpio_num_t SDA = GPIO_NUM_21;
const gpio_num_t SCL = GPIO_NUM_22;
const uint32_t CLOCK = 100000U;
QueueHandle_t engineQueue, batteryQueue, distanceQueue;
QueueHandle_t accelQueue, gyroQueue, speedQueue;


static void batteryTask(void*) {
    myADC battery;

    while (1)
    {
        int percentage = 100;
        // int percentage = battery.getVoltage() * 25;
        // printf("Voltage: %.2fV | Percentage %3.0d\n", battery.getVoltage(), battery.getPercentage());
        xQueueSendToBack(batteryQueue, &percentage, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}



static void robotDriver(void*) {
    RobotConfig config = {};
    config.in1 = IN1;
    config.in2 = IN2;
    config.in3 = IN3;
    config.in4 = IN4;
    config.pwm1 = PWM1;
    config.pwm2 = PWM2;
    config.enc1a = ENC1A;
    config.enc1b = ENC1B;
    config.enc2a = ENC2A;
    config.enc2b = ENC2B;
    config.pcntUnit1 = PCNT1;
    config.pcntUnit2 = PCNT2;
    config.pwmChannel = MOTOR_PWM;

    Robot robot(config);
    EnginePacket packet;
    // int64_t currentTime = 0;
    // int16_t input[2];
    
    while (1) {
        xQueueReceive(engineQueue, &packet, 0);
        robot.drive(packet);

        TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
        TIMERG0.wdt_feed=1;
        TIMERG0.wdt_wprotect=0;
        // pcnt_get_counter_value(PCNT1, input);
        // pcnt_get_counter_value(PCNT2, input+1);
        // printf("PCNT1: %d, PCNT2: %d\n", input[0], input[1]);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        // currentTime = esp_timer_get_time();
    }
    vTaskDelete(NULL);
}

static void mpuTask(void*) {
    i2c0.begin(SDA, SCL, CLOCK);
    MPU_t MPU;
    MPU.setBus(i2c0);
    MPU.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);

    ESP_ERROR_CHECK(MPU.testConnection());
    ESP_ERROR_CHECK(MPU.initialize());

    // MPU.setSampleRate(250);  // in (Hz)
    MPU.setAccelFullScale(mpud::ACCEL_FS_4G);
    MPU.setGyroFullScale(mpud::GYRO_FS_500DPS);
    MPU.setDigitalLowPassFilter(mpud::DLPF_42HZ);  // smoother data
    MPU.setInterruptEnabled(mpud::INT_EN_RAWDATA_READY);  // enable INT pin


    while (1) {
        mpud::raw_axes_t accelRaw;     // holds x, y, z axes as int16
        mpud::raw_axes_t gyroRaw;      // holds x, y, z axes as int16
        MPU.acceleration(&accelRaw);  // fetch raw data from the registers
        MPU.rotation(&gyroRaw);       // fetch raw data from the registers
        // printf("accel: %+d %+d %+d\n", accelRaw.x, accelRaw.y, accelRaw.z);
        // printf("gyro: %+d %+d %+d\n", gyroRaw[0], gyroRaw[1], gyroRaw[2]);


        mpud::float_axes_t accelG = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_4G);  // raw data to gravity
        mpud::float_axes_t gyroDPS = mpud::gyroDegPerSec(gyroRaw, mpud::GYRO_FS_500DPS);  // raw data to º/s
        // printf("accel: %+.2f %+.2f %+.2f\n", accelG[0], accelG[1], accelG[2]);
        // printf("gyro: %+.2f %+.2f %+.2f\n", gyroDPS.x, gyroDPS.y, gyroDPS.z);


        AcceloPacket packetA(accelG[0], accelG[1], accelG[2]);
        xQueueSendToBack(accelQueue, &packetA, 0);

        GyroPacket packetG(gyroDPS.x, gyroDPS.y, gyroDPS.z);
        xQueueSendToBack(gyroQueue, &packetG, 0);


        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL);
}

extern "C" void app_main()
{
    esp_pm_config_esp32_t power = {};
    power.min_freq_mhz = 240;
    power.max_freq_mhz = 240;
    power.light_sleep_enable = false;
    esp_pm_configure(&power);

    initialise_wifi();

    engineQueue = xQueueCreate(5, sizeof(EnginePacket));
    accelQueue = xQueueCreate(5, sizeof(AcceloPacket));
    gyroQueue = xQueueCreate(5, sizeof(GyroPacket));
    batteryQueue = xQueueCreate(5, sizeof(int));
    distanceQueue = xQueueCreate(5, sizeof(int));
    speedQueue = xQueueCreate(5, sizeof(int16_t));


    xTaskCreate(udpServerTask, "udp_server", 4096, (void*)UDP_PORT, 10, NULL);
    xTaskCreate(tcpServerTask, "tcp_server", 14096, (void*)TCP_PORT, 8, NULL);
    xTaskCreate(robotDriver, "driver", 4096, nullptr, 20, NULL);
    xTaskCreate(batteryTask, "batteryTask", configMINIMAL_STACK_SIZE * 3, NULL, 3, NULL);
    xTaskCreate(mpuTask, "mpuTask", 4096, NULL, 5, NULL);
    Ultrasonic sensor(TRIG, ECHO, SENSOR_PWM);

    vTaskSuspend(NULL);
}