#pragma once

#include "robot.hpp"
#include <I2Cbus.hpp>
#include <MPU.hpp>
#include "mpu/math.hpp"
#include <adc.hpp>

#include "config.hpp"

void batteryTask(void*) {
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



void robotDriver(void*) {
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
    
    while (1) {
        xQueueReceive(engineQueue, &packet, 0);
        robot.drive(packet);

        TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
        TIMERG0.wdt_feed=1;
        TIMERG0.wdt_wprotect=0;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}



void mpuTask(void*) {
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

        AcceloPacket packetA(accelG[0] * 100, accelG[1] * 100, accelG[2] * 100);
        xQueueSendToBack(accelQueue, &packetA, 0);

        // GyroPacket packetG(gyroDPS.x, gyroDPS.y, gyroDPS.z);
        // xQueueSendToBack(gyroQueue, &packetG, 0);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL);
}