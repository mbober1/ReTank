#include <string>
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <udp.hpp>
#include <wifi.hpp>
#include <robot.hpp>
#include <I2Cbus.hpp>

// int left;
// int right;
// int previousTime;
// int previousTime2;
// int16_t input[4];


#include "esp_log.h"
#include "driver/i2c.h"

// #define I2C_MASTER_SCL_IO 22               /*!< gpio number for I2C master clock */
// #define I2C_MASTER_SDA_IO 21               /*!< gpio number for I2C master data  */
// #define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
// #define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
// #define I2C_MASTER_RX_BUF_DISABLE 0  

#define I2C_ADDR 0x60



// static esp_err_t i2c_master_init(void)
// {
//     int i2c_master_port = I2C_NUM_0;
//     i2c_config_t conf;
//     conf.mode = I2C_MODE_MASTER,
//     conf.sda_io_num = I2C_MASTER_SDA_IO,
//     conf.sda_pullup_en = GPIO_PULLUP_ENABLE,
//     conf.scl_io_num = I2C_MASTER_SCL_IO,
//     conf.scl_pullup_en = GPIO_PULLUP_ENABLE,
//     conf.master.clk_speed = 400000,
//         // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
//     esp_err_t err = i2c_param_config(i2c_master_port, &conf);
//     if (err != ESP_OK) {
//         return err;
//     }
//     return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
// }


// // /**
// //  * @brief test code to read esp-i2c-slave
// //  *        We need to fill the buffer of esp slave device, then master can read them out.
// //  *
// //  * _______________________________________________________________________________________
// //  * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
// //  * --------|--------------------------|----------------------|--------------------|------|
// //  *
// //  */
// // static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
// // {
// //     if (size == 0) {
// //         return ESP_OK;
// //     }
// //     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
// //     i2c_master_start(cmd);
// //     i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
// //     if (size > 1) {
// //         i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
// //     }
// //     i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
// //     i2c_master_stop(cmd);
// //     esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
// //     i2c_cmd_link_delete(cmd);
// //     return ret;
// // }



// /**
//  * @brief test code to operate on BH1750 sensor
//  *
//  * 1. set operation mode(e.g One time L-resolution mode)
//  * _________________________________________________________________
//  * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | stop |
//  * --------|---------------------------|---------------------|------|
//  * 2. wait more than 24 ms
//  * 3. read data
//  * ______________________________________________________________________________________
//  * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
//  * --------|---------------------------|--------------------|--------------------|------|
//  */
// static esp_err_t i2c_master_sensor_test(i2c_port_t i2c_num, uint8_t *data_h, uint8_t *data_l)
// {
//     int ret;
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, I2C_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, BH1750_CMD_START, ACK_CHECK_EN);
//     i2c_master_stop(cmd);
//     ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//     i2c_cmd_link_delete(cmd);
//     if (ret != ESP_OK) {
//         return ret;
//     }
//     vTaskDelay(30 / portTICK_RATE_MS);
//     cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
//     i2c_master_read_byte(cmd, data_h, ACK_VAL);
//     i2c_master_read_byte(cmd, data_l, NACK_VAL);
//     i2c_master_stop(cmd);
//     ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }







extern "C" void app_main()
{
    // robot Robot(IN1, IN2, PWM1, PWMCHANNEL, IN3, IN4, PWM2, ENC1A, ENC1B, ENC2A, ENC2B, PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2, PCNT_UNIT_3);
    // initialise_wifi();

    // xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);

    I2C_t myI2C(I2C_NUM_0);

    myI2C.begin(GPIO_NUM_21, GPIO_NUM_22);
    myI2C.scanner();


    while (1) {
        // Robot.setPoint(left/7, right/7);
        // Robot.autos();        

        vTaskDelay(pdMS_TO_TICKS(10));
        // int64_t currentTime = esp_timer_get_time();
        // if(currentTime - previousTime > 100000) {
        //     previousTime = currentTime;

        //     pcnt_get_counter_value(Robot.engine[0].encoder, &input[0]);
        //     pcnt_get_counter_value(Robot.engine[0].encoder2, &input[1]);
        //     pcnt_get_counter_value(Robot.engine[1].encoder, &input[2]);
        //     pcnt_get_counter_value(Robot.engine[1].encoder2, &input[3]);
        //     // printf("Setpoint: %d, ENC: %d, Error: %f\n", left, input1 + input2, e1);
        //     // printf("L1: %d, L2: %d, R1: %d, R2: %d\n", input[0], input[1], input[2], input[3]);
        // }

        // if(currentTime - previousTime2 > 1000000) {
        //     previousTime2 = currentTime;
        //     left ++;
        //     right ++;
        //     if(left > 25) left = 0;
        //     if(right > 25) right = 0;
        //     Robot.setPoint(left, right);
        // }
    }
}