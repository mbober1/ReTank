#include <string>
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// #include <udp.hpp>
// #include <wifi.hpp>
// #include <robot.hpp>
// #include <I2Cbus.hpp>
// #include <MPU.hpp>
#include "esp_log.h"
#include "esp_err.h"

// #include "mpu/math.hpp"
// #include "mpu/types.hpp"


#include <adc.hpp>
#include <camera.hpp>

// int left;
// int right;
// int previousTime;
// int previousTime2;
// int16_t input[4];

// void upload_image(uint8_t *buffer, uint32_t length){
// 	esp_err_t err;
// 	uint32_t lengthWithMac = length+6;
// 	uint8_t *macAndCheese = malloc(lengthWithMac);
// 	memset((void*)macAndCheese, 0, lengthWithMac);
// 	err = esp_efuse_mac_get_default(macAndCheese);
// 	memcpy((void*)macAndCheese+6, (void*)buffer, length);
// 	esp_http_client_config_t config = {
// 		.url = "https://robotany.queueunderflow.com/api/dataUpload/v1/uploadImage",
// 		.event_handler = _http_event_handler,
// 		.method = HTTP_METHOD_POST,
//     };
//     esp_http_client_handle_t client = esp_http_client_init(&config);
// 	esp_http_client_set_post_field(client, (const char*)macAndCheese, lengthWithMac);
// 	esp_http_client_set_header(client, "Content-Type", "application/octet-stream");
// 	err = esp_http_client_perform(client);
// 	if (err == ESP_OK) {
// 		ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d",
// 				esp_http_client_get_status_code(client),
// 				esp_http_client_get_content_length(client));
// 	} else {
// 		ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
// 	}
// 	esp_http_client_cleanup(client);
// 	free(macAndCheese);
// }


void capture_image(uint8_t **buffer, camera Cam){
	Cam.start_capture();
	// TODO: Read the done flag.
	while(!Cam.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
	uint32_t fifo_length = Cam.read_fifo_length();
	printf("FIFO Length: %d\n", fifo_length);
	if(fifo_length == 0){
		printf("FIFO length not set.\n");
	}
	
	uint32_t bufferLength = 0;
	Cam.image_read(fifo_length, buffer, &bufferLength);
    printf("FIFO len %d, buffer len %d\n", fifo_length, bufferLength);
	
	// upload_image(*buffer, bufferLength);

	return;
}

spi_device_handle_t spiiii;


extern "C" void app_main()
{
    // myADC battery;

    // while (1) {
    //     printf("Voltage: %.2fV\n", battery.getVoltage());
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }




    // robot Robot(IN1, IN2, PWM1, PWMCHANNEL, IN3, IN4, PWM2, ENC1A, ENC1B, ENC2A, ENC2B, PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2, PCNT_UNIT_3);
    // initialise_wifi();

    // xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);

    camera Cam;
    // vTaskDelay(2000 / portTICK_PERIOD_MS);
	int i = 0;

    while (1) {
		printf("[%d] Captured!\n", i);
		i++;
		uint8_t *rxBuf;
		capture_image(&rxBuf, Cam);
		free(rxBuf);
        printf("Sleeping...\n");
		vTaskDelay((10*1000) / portTICK_PERIOD_MS);
	}
    // I2C_t myI2C(I2C_NUM_0);
    // myI2C.begin(GPIO_NUM_21, GPIO_NUM_22, 400000);
    // myI2C.scanner();


    // MPU_t MPU;
    // MPU.setBus(i2c0);
    // MPU.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);

    // ESP_ERROR_CHECK(MPU.testConnection());
    // ESP_ERROR_CHECK(MPU.initialize());


    // MPU.setSampleRate(250);  // in (Hz)
    // MPU.setAccelFullScale(mpud::ACCEL_FS_4G);
    // MPU.setGyroFullScale(mpud::GYRO_FS_500DPS);
    // MPU.setDigitalLowPassFilter(mpud::DLPF_42HZ);  // smoother data
    // MPU.setInterruptEnabled(mpud::INT_EN_RAWDATA_READY);  // enable INT pin

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

    //     // if(currentTime - previousTime2 > 1000000) {
    //     //     previousTime2 = currentTime;
    //     //     left ++;
    //     //     right ++;
    //     //     if(left > 25) left = 0;
    //     //     if(right > 25) right = 0;
    //     //     Robot.setPoint(left, right);
    //     // }
    // }
}