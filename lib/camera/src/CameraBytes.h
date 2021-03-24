#ifndef CAMERABYTES_H_
#define CAMERABYTES_H_

#include <stdio.h>

struct sensor_reg {
	uint16_t reg;
	uint16_t val;
};

extern const struct sensor_reg OV2640_QVGA[];

extern const struct sensor_reg OV2640_JPEG_INIT[];

extern const struct sensor_reg OV2640_YUV422[];

extern const struct sensor_reg OV2640_JPEG[];

extern const struct sensor_reg OV2640_320x240_JPEG[];

extern const struct sensor_reg OV2640_640x480_JPEG[];  

extern const struct sensor_reg OV2640_1280x1024_JPEG[];

#endif