#pragma once

#include <stdio.h>
#include <string.h>
#include "CameraBytes.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

 #include <stdio.h>
 #include <stdint.h>
 #include "driver/spi_common.h"
 #include "driver/spi_master.h"
 #include "esp_log.h"
 #include "esp_err.h"
 #include "sdkconfig.h"

#include <SPIbus.hpp>

// SPI MISO = GPIO 19
#define CAM_MISO_PIN GPIO_NUM_19
// SPI MOSI = GPIO 23
#define CAM_MOSI_PIN GPIO_NUM_23
// SPI clock = GPIO 18
#define CAM_SCK_PIN GPIO_NUM_18

// I2C data = GPIO 21
#define CAM_SDA_PIN GPIO_NUM_21
// I2C clock = GPIO 22
#define CAM_SCL_PIN GPIO_NUM_22
// chip select = GPIO 17
#define CAM_CS_PIN GPIO_NUM_5

// camera power on 
// #define CAM_POWER_ON D10
// I2C Frequency
#define I2C_MASTER_FREQ 100000
// #define I2C_MASTER_ACK 0
// #define I2C_MASTER_NACK 1

// OV2640-specific macros (may want to export to regs.h)
#define OV2640_CHIPID_HIGH  0x0A
#define OV2640_CHIPID_LOW   0x0B
#define ARDUCHIP_TEST1      0x00

#define FIFO_SIZE1				0x42  //Camera write FIFO size[7:0] for burst to read
#define FIFO_SIZE2				0x43  //Camera write FIFO size[15:8]
#define FIFO_SIZE3				0x44  //Camera write FIFO size[18:16]

#define ARDUCHIP_TEST1       	0x00  //TEST register
#define ARDUCHIP_TEST2      	0x01  //TEST register

#define ARDUCHIP_TRIG      		0x41  //Trigger source
#define VSYNC_MASK         		0x01
#define SHUTTER_MASK       		0x02
#define CAP_DONE_MASK      		0x08

#define BMP 	0
#define JPEG	1

#define OV2640_160x120 		0	//160x120
#define OV2640_176x144 		1	//176x144
#define OV2640_320x240 		2	//320x240
#define OV2640_352x288 		3	//352x288
#define OV2640_640x480		4	//640x480
#define OV2640_800x600 		5	//800x600
#define OV2640_1024x768		6	//1024x768
#define OV2640_1280x1024	7	//1280x1024
#define OV2640_1600x1200	8	//1600x1200

#define MAX_FIFO_SIZE		0x5FFFF	//384KByte
#define ARDUCHIP_FIFO      		0x04  //FIFO and I2C control
#define FIFO_CLEAR_MASK    		0x01
#define FIFO_START_MASK    		0x02
#define FIFO_RDPTR_RST_MASK     0x10
#define FIFO_WRPTR_RST_MASK     0x20

#define I2C_ADDRESS             0x60

class camera
{
private:
    spi_device_handle_t spiiii;

    // SPI Base operations
    esp_err_t bus_write(uint8_t addr, uint8_t data);
    uint8_t bus_read(uint8_t addr);

    // I2C Base Operations
    void write_sensor_reg(uint8_t addr, uint8_t data);
    void read_sensor_reg(uint8_t addr, uint8_t *buffer);

public:
    camera();
    // ~camera();

    // SPI Extended
    void clear_fifo_flag();
    void start_capture();
    int read_fifo_length();
    void set_fifo_burst();
    uint8_t get_bit(uint8_t addr, uint8_t bit);
    void image_read(uint32_t fifoLength, uint8_t** rxBuf, uint32_t* rxLen);


    // I2C Extended
    void write_many_sensor_regs(const struct sensor_reg s[]);
    void OV2640_set_JPEG_size();
    void init_cam_regs(int format);

};