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
// #include <MPU.hpp>
// #include "esp_log.h"
// #include "esp_err.h"
// #include "mpu/math.hpp"
// #include "mpu/types.hpp"
// #include <adc.hpp>
#include <ultrasonic.hpp>

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

static int udpPort = 8090;
static int tcpPort = 8091;
const uint8_t ENC1A = 35;
const uint8_t ENC1B = 34;
const uint8_t ENC2A = 33;
const uint8_t ENC2B = 33;
const uint8_t PWM1 = 25;
const gpio_num_t IN1 = GPIO_NUM_26;
const gpio_num_t IN2 = GPIO_NUM_27;
const gpio_num_t IN3 = GPIO_NUM_14;
const gpio_num_t IN4 = GPIO_NUM_12;
const uint8_t PWM2 = 13;
const uint8_t PWMCHANNEL = 0;

QueueHandle_t engineQueue, batteryQueue, distanceQueue;
QueueHandle_t accelQueue, gyroQueue, speedQueue;

// static void batteryTask(void*) {
//     myADC battery;

//     while (1)
//     {
//         int percentage = battery.getVoltage() * 25;
//         // printf("Voltage: %.2fV | Percentage %3.0d\n", voltage, percentage);
//         xQueueSendToBack(batteryQueue, &percentage, 0);
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
//     vTaskDelete(NULL);
// }

unsigned long sensor1Time;
float distance1_cm;

static void distanceTask(void*) {
    Ultrasonic sensor(GPIO_NUM_4, GPIO_NUM_2);

    while (1)
    {  
        printf("Distance: %f, Sensor time: %ld, \n", distance1_cm, sensor1Time);
        // if(distance > 5) xQueueSendToBack(distanceQueue, &distance, 0);
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    vTaskDelete(NULL);
}

// static void robotDriver(void*) {
//     robot Robot(IN1, IN2, PWM1, PWMCHANNEL, IN3, IN4, PWM2, ENC2A, ENC2B, ENC1A, ENC1B, PCNT_UNIT_0, PCNT_UNIT_1);
//     EnginePacket packet;
//     // int64_t currentTime = 0;
    
//     while (1) {
//         xQueueReceive(engineQueue, &packet, 0);
//         Robot.drive(packet);

//         TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
//         TIMERG0.wdt_feed=1;
//         TIMERG0.wdt_wprotect=0;

//         // printf("System lag: %lld\n", esp_timer_get_time() - currentTime);
//         // currentTime = esp_timer_get_time();
//         // vTaskDelay(50 / portTICK_PERIOD_MS);
//     }
//     vTaskDelete(NULL);
// }

// static void mpuTask(void*) {
//     MPU_t MPU;
//     int err = 1;
//     MPU.setBus(i2c0);
//     MPU.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);

//     ESP_ERROR_CHECK(MPU.testConnection());
//     ESP_ERROR_CHECK(MPU.initialize());

//     MPU.setSampleRate(250);  // in (Hz)
//     MPU.setAccelFullScale(mpud::ACCEL_FS_4G);
//     MPU.setGyroFullScale(mpud::GYRO_FS_500DPS);
//     MPU.setDigitalLowPassFilter(mpud::DLPF_42HZ);  // smoother data
//     MPU.setInterruptEnabled(mpud::INT_EN_RAWDATA_READY);  // enable INT pin


//     while (!err) {
//         mpud::raw_axes_t accelRaw;     // holds x, y, z axes as int16
//         mpud::raw_axes_t gyroRaw;      // holds x, y, z axes as int16
//         MPU.acceleration(&accelRaw);  // fetch raw data from the registers
//         MPU.rotation(&gyroRaw);       // fetch raw data from the registers
//         printf("accel: %+d %+d %+d\n", accelRaw.x, accelRaw.y, accelRaw.z);
//         printf("gyro: %+d %+d %+d\n", gyroRaw[0], gyroRaw[1], gyroRaw[2]);


//         mpud::float_axes_t accelG = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_4G);  // raw data to gravity
//         mpud::float_axes_t gyroDPS = mpud::gyroDegPerSec(gyroRaw, mpud::GYRO_FS_500DPS);  // raw data to º/s
//         printf("accel: %+.2f %+.2f %+.2f\n", accelG[0], accelG[1], accelG[2]);
//         printf("gyro: %+.2f %+.2f %+.2f\n", gyroDPS.x, gyroDPS.y, gyroDPS.z);


//         AcceloPacket packetA(accelG[0], accelG[1], accelG[2]);
//         xQueueSendToBack(accelQueue, &packetA, 0);

//         GyroPacket packetG(gyroDPS.x, gyroDPS.y, gyroDPS.z);
//         xQueueSendToBack(gyroQueue, &packetG, 0);


//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
//     vTaskDelete(NULL);
// }

extern "C" void app_main()
{
    esp_pm_config_esp32_t power = {};
    power.min_freq_mhz = 240;
    power.max_freq_mhz = 240;
    power.light_sleep_enable = false;

    esp_pm_configure(&power);

    // gpio_install_isr_service(0);
    initialise_wifi();

    // I2C_t myI2C(I2C_NUM_0);
    // myI2C.begin(GPIO_NUM_21, GPIO_NUM_22, 400000);
    // myI2C.scanner();

    engineQueue = xQueueCreate(5, sizeof(EnginePacket));
    accelQueue = xQueueCreate(5, sizeof(AcceloPacket));
    gyroQueue = xQueueCreate(5, sizeof(GyroPacket));
    batteryQueue = xQueueCreate(5, sizeof(int));
    distanceQueue = xQueueCreate(5, sizeof(int));
    speedQueue = xQueueCreate(5, sizeof(int));


    xTaskCreate(udpServerTask, "udp_server", 4096, (void*)udpPort, 5, NULL);
    xTaskCreate(tcpServerTask, "tcp_server", 4096, (void*)tcpPort, 4, NULL);
    // xTaskCreate(robotDriver, "driver", 14096, nullptr, 20, NULL);
    // xTaskCreate(batteryTask, "batteryTask", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(distanceTask, "distanceTask", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    // xTaskCreate(mpuTask, "mpuTask", 4096, NULL, 5, NULL);

    vTaskSuspend(NULL);
}