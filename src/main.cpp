#define INCLUDE_vTaskSuspend 1

#include <string>
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <udp.hpp>
#include <tcp.hpp>
#include <wifi.hpp>
// #include <robot.hpp>
#include <I2Cbus.hpp>
// #include <MPU.hpp>
#include "esp_log.h"
#include "esp_err.h"
// #include "mpu/math.hpp"
// #include "mpu/types.hpp"
#include <adc.hpp>
#include <ultrasonic.hpp>

static int udpPort = 8090;
static int tcpPort = 8091;
QueueHandle_t engineQueue, batteryQueue, distanceQueue;
QueueHandle_t accelQueue, gyroQueue;


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


static void distanceTask(void*) {
    Ultrasonic distance(GPIO_NUM_26, GPIO_NUM_17);

    while (1)
    {  
        printf("test\n");
        distance.measureTime();
        // xQueueSendToBack(distanceQueue, &distance, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}


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
    initialise_wifi();
    // robot Robot(IN1, IN2, PWM1, PWMCHANNEL, IN3, IN4, PWM2, ENC1A, ENC1B, ENC2A, ENC2B, PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2, PCNT_UNIT_3);

    // I2C_t myI2C(I2C_NUM_0);
    // myI2C.begin(GPIO_NUM_21, GPIO_NUM_22, 400000);
    // myI2C.scanner();

    engineQueue = xQueueCreate(5, sizeof(EnginePacket));
    accelQueue = xQueueCreate(5, sizeof(AcceloPacket));
    gyroQueue = xQueueCreate(5, sizeof(GyroPacket));
    batteryQueue = xQueueCreate(5, sizeof(int));
    distanceQueue = xQueueCreate(5, sizeof(int));


    xTaskCreate(udpServerTask, "udp_server", 4096, (void*)udpPort, 5, NULL);
    xTaskCreate(tcpServerTask, "tcp_server", 4096, (void*)tcpPort, 5, NULL);
    // xTaskCreate(batteryTask, "batteryTask", 4096, NULL, 5, NULL);
    xTaskCreate(distanceTask, "distanceTask", 4096, NULL, 5, NULL);
    // xTaskCreate(mpuTask, "mpuTask", 4096, NULL, 5, NULL);


    

    while (1) {
        // EnginePacket dupa(0,0);
        // xQueueReceive(engineQueue, &dupa, portMAX_DELAY);
        // printf("L: %d, R: %d\n", dupa.left, dupa.right);
        // distance.measure();
        // printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(1000));
        // Robot.setPoint(left/7, right/7);
        // Robot.autos();
    }



    // printf("Reading sensor data:\n");
    // mpud::raw_axes_t accelRaw;   // x, y, z axes as int16
    // mpud::raw_axes_t gyroRaw;    // x, y, z axes as int16
    // mpud::float_axes_t accelG;   // accel axes in (g) gravity format
    // mpud::float_axes_t gyroDPS;  // gyro axes in (DPS) º/s format
    // while (true) {
    //     // Read
    //     MPU.acceleration(&accelRaw);  // fetch raw data from the registers
    //     MPU.rotation(&gyroRaw);       // fetch raw data from the registers
    //     // MPU.motion(&accelRaw, &gyroRaw);  // read both in one shot
    //     // Convert
    //     accelG = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_4G);
    //     gyroDPS = mpud::gyroDegPerSec(gyroRaw, mpud::GYRO_FS_500DPS);
    //     // Debug
    //     printf("accel: [%+6.2f %+6.2f %+6.2f ] (G) \t", accelG.x, accelG.y, accelG.z);
    //     printf("gyro: [%+7.2f %+7.2f %+7.2f ] (º/s)\n", gyroDPS[0], gyroDPS[1], gyroDPS[2]);
    //     vTaskDelay(100 / portTICK_PERIOD_MS);
    // }

    // while (1) {
        
    //     mpud::raw_axes_t accelRaw;     // holds x, y, z axes as int16
    //     mpud::raw_axes_t gyroRaw;      // holds x, y, z axes as int16
    //     MPU.acceleration(&accelRaw);  // fetch raw data from the registers
    //     MPU.rotation(&gyroRaw);       // fetch raw data from the registers
    //     printf("accel: %+d %+d %+d\n", accelRaw.x, accelRaw.y, accelRaw.z);
    //     printf("gyro: %+d %+d %+d\n", gyroRaw[0], gyroRaw[1], gyroRaw[2]);

    //     mpud::float_axes_t accelG = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_4G);  // raw data to gravity
    //     mpud::float_axes_t gyroDPS = mpud::gyroDecPerSec(gyroRaw, mpud::GYRO_FS_500DPS);  // raw data to º/s
    //     printf("accel: %+.2f %+.2f %+.2f\n", accelG[0], accelG[1], accelG[2]);
    //     printf("gyro: %+.2f %+.2f %+.2f\n", gyroDPS.x, gyroDPS.y, gyroDPS.z);
    //     // Robot.setPoint(left/7, right/7);
    //     // Robot.autos();        

    //     vTaskDelay(pdMS_TO_TICKS(10));
    //     // int64_t currentTime = esp_timer_get_time();
    //     // if(currentTime - previousTime > 100000) {
    //     //     previousTime = currentTime;

    //     //     pcnt_get_counter_value(Robot.engine[0].encoder, &input[0]);
    //     //     pcnt_get_counter_value(Robot.engine[0].encoder2, &input[1]);
    //     //     pcnt_get_counter_value(Robot.engine[1].encoder, &input[2]);
    //     //     pcnt_get_counter_value(Robot.engine[1].encoder2, &input[3]);
    //     //     // printf("Setpoint: %d, ENC: %d, Error: %f\n", left, input1 + input2, e1);
    //     //     // printf("L1: %d, L2: %d, R1: %d, R2: %d\n", input[0], input[1], input[2], input[3]);
    //     // }

    // }
}