
#include "ArduCAM.h"

ArduCAM::ArduCAM(int mosi_io_num, int miso_io_num, int sclk_io_num, int max_transfer_sz)
{
	spi_device_handle_t device;
	ESP_ERROR_CHECK( this->mySPI.begin(MOSI_PIN, MISO_PIN, SCLK_PIN));
    ESP_ERROR_CHECK( this->mySPI.addDevice(0, 8000000, CS_PIN, &device));
	this->sensor_addr = 0x30;
}

void ArduCAM::InitCAM()
{
	wrSensorReg8_8(0xff, 0x01);
	wrSensorReg8_8(0x12, 0x80);
	vTaskDelay(pdMS_TO_TICKS(100));
	
	if (m_fmt == JPEG)
	{
		wrSensorRegs8_8(OV2640_JPEG_INIT);
		wrSensorRegs8_8(OV2640_YUV422);
		wrSensorRegs8_8(OV2640_JPEG);
		wrSensorReg8_8(0xff, 0x01);
		wrSensorReg8_8(0x15, 0x00);
		wrSensorRegs8_8(OV2640_320x240_JPEG);
		//wrSensorReg8_8(0xff, 0x00);
		//wrSensorReg8_8(0x44, 0x32);
	}
	else
	{
		wrSensorRegs8_8(OV2640_QVGA);
	}
}

void ArduCAM::flush_fifo(void)
{
	write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

void ArduCAM::start_capture(void)
{
	write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
}

void ArduCAM::clear_fifo_flag(void )
{
	write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

uint32_t ArduCAM::read_fifo_length(void)
{
	uint32_t len1,len2,len3,length=0;
	len1 = read_reg(FIFO_SIZE1);
  len2 = read_reg(FIFO_SIZE2);
  len3 = read_reg(FIFO_SIZE3) & 0x7f;
  length = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
	return length;	
}

void ArduCAM::set_fifo_burst()
{
	#if defined (RASPBERRY_PI)
	transfer(BURST_FIFO_READ);
	#else
    SPI.transfer(BURST_FIFO_READ);
   #endif
		
}

void ArduCAM::CS_HIGH(void)
{
	 sbi(P_CS, B_CS);	
}
void ArduCAM::CS_LOW(void)
{
	 cbi(P_CS, B_CS);	
}

uint8_t ArduCAM::read_fifo(void)
{
	uint8_t data;
	data = bus_read(SINGLE_FIFO_READ);
	return data;
}

uint8_t ArduCAM::read_reg(uint8_t addr)
{
	uint8_t data;
	data = bus_read(addr & 0x7F);
	return data;
}

void ArduCAM::write_reg(uint8_t addr, uint8_t data)
{
	bus_write(addr | 0x80, data);
}

//Set corresponding bit  
void ArduCAM::set_bit(uint8_t addr, uint8_t bit)
{
	uint8_t temp;
	temp = read_reg(addr);
	write_reg(addr, temp | bit);
}
//Clear corresponding bit 
void ArduCAM::clear_bit(uint8_t addr, uint8_t bit)
{
	uint8_t temp;
	temp = read_reg(addr);
	write_reg(addr, temp & (~bit));
}

//Get corresponding bit status
uint8_t ArduCAM::get_bit(uint8_t addr, uint8_t bit)
{
  uint8_t temp;
  temp = read_reg(addr);
  temp = temp & bit;
  return temp;
}

//Set ArduCAM working mode
//MCU2LCD_MODE: MCU writes the LCD screen GRAM
//CAM2LCD_MODE: Camera takes control of the LCD screen
//LCD2MCU_MODE: MCU read the LCD screen GRAM
void ArduCAM::set_mode(uint8_t mode)
{
  switch (mode)
  {
    case MCU2LCD_MODE:
      write_reg(ARDUCHIP_MODE, MCU2LCD_MODE);
      break;
    case CAM2LCD_MODE:
      write_reg(ARDUCHIP_MODE, CAM2LCD_MODE);
      break;
    case LCD2MCU_MODE:
      write_reg(ARDUCHIP_MODE, LCD2MCU_MODE);
      break;
    default:
      write_reg(ARDUCHIP_MODE, MCU2LCD_MODE);
      break;
  }
}

uint8_t ArduCAM::bus_write(int address,int value)
{	
	cbi(P_CS, B_CS);
	SPI.transfer(address);
	SPI.transfer(value);
	sbi(P_CS, B_CS);
	return 1;
}

uint8_t ArduCAM:: bus_read(int address)
{
	uint8_t value;
	cbi(P_CS, B_CS);
	SPI.transfer(address);
	value = SPI.transfer(0x00);
	// take the SS pin high to de-select the chip:
	sbi(P_CS, B_CS);
	return value;
}

// void ArduCAM::OV2640_set_JPEG_size(uint8_t size)
// {
// 	switch(size)
// 	{
// 		case OV2640_160x120:
// 			wrSensorRegs8_8(OV2640_160x120_JPEG);
// 			break;
// 		case OV2640_176x144:
// 			wrSensorRegs8_8(OV2640_176x144_JPEG);
// 			break;
// 		case OV2640_320x240:
// 			wrSensorRegs8_8(OV2640_320x240_JPEG);
// 			break;
// 		case OV2640_352x288:
// 	  	wrSensorRegs8_8(OV2640_352x288_JPEG);
// 			break;
// 		case OV2640_640x480:
// 			wrSensorRegs8_8(OV2640_640x480_JPEG);
// 			break;
// 		case OV2640_800x600:
// 			wrSensorRegs8_8(OV2640_800x600_JPEG);
// 			break;
// 		case OV2640_1024x768:
// 			wrSensorRegs8_8(OV2640_1024x768_JPEG);
// 			break;
// 		case OV2640_1280x1024:
// 			wrSensorRegs8_8(OV2640_1280x1024_JPEG);
// 			break;
// 		case OV2640_1600x1200:
// 			wrSensorRegs8_8(OV2640_1600x1200_JPEG);
// 			break;
// 		default:
// 			wrSensorRegs8_8(OV2640_320x240_JPEG);
// 			break;
// 	}
// }

// void ArduCAM::set_format(unsigned char fmt)
// {
//   if (fmt == BMP)
//     m_fmt = BMP;
//   else if(fmt == RAW)
//     m_fmt = RAW;
//   else
//     m_fmt = JPEG;
// }

// void ArduCAM::OV2640_set_Light_Mode(uint8_t Light_Mode)
// {
// 		switch(Light_Mode)
// 		{
		
// 			case Auto:
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0xc7, 0x00); //AWB on
// 			break;
// 			case Sunny:
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0xc7, 0x40); //AWB off
// 			wrSensorReg8_8(0xcc, 0x5e);
// 			wrSensorReg8_8(0xcd, 0x41);
// 			wrSensorReg8_8(0xce, 0x54);
// 			break;
// 			case Cloudy:
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0xc7, 0x40); //AWB off
// 			wrSensorReg8_8(0xcc, 0x65);
// 			wrSensorReg8_8(0xcd, 0x41);
// 			wrSensorReg8_8(0xce, 0x4f);  
// 			break;
// 			case Office:
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0xc7, 0x40); //AWB off
// 			wrSensorReg8_8(0xcc, 0x52);
// 			wrSensorReg8_8(0xcd, 0x41);
// 			wrSensorReg8_8(0xce, 0x66);
// 			break;
// 			case Home:
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0xc7, 0x40); //AWB off
// 			wrSensorReg8_8(0xcc, 0x42);
// 			wrSensorReg8_8(0xcd, 0x3f);
// 			wrSensorReg8_8(0xce, 0x71);
// 			break;
// 			default :
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0xc7, 0x00); //AWB on
// 			break; 
// 		}	

// }


// void ArduCAM::OV2640_set_Color_Saturation(uint8_t Color_Saturation)
// {
// 	switch(Color_Saturation)
// 	{
// 		case Saturation2:
		
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0x7c, 0x00);
// 			wrSensorReg8_8(0x7d, 0x02);
// 			wrSensorReg8_8(0x7c, 0x03);
// 			wrSensorReg8_8(0x7d, 0x68);
// 			wrSensorReg8_8(0x7d, 0x68);
// 		break;
// 		case Saturation1:
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0x7c, 0x00);
// 			wrSensorReg8_8(0x7d, 0x02);
// 			wrSensorReg8_8(0x7c, 0x03);
// 			wrSensorReg8_8(0x7d, 0x58);
// 			wrSensorReg8_8(0x7d, 0x58);
// 		break;
// 		case Saturation0:
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0x7c, 0x00);
// 			wrSensorReg8_8(0x7d, 0x02);
// 			wrSensorReg8_8(0x7c, 0x03);
// 			wrSensorReg8_8(0x7d, 0x48);
// 			wrSensorReg8_8(0x7d, 0x48);
// 		break;
// 		case Saturation_1:
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0x7c, 0x00);
// 			wrSensorReg8_8(0x7d, 0x02);
// 			wrSensorReg8_8(0x7c, 0x03);
// 			wrSensorReg8_8(0x7d, 0x38);
// 			wrSensorReg8_8(0x7d, 0x38);
// 		break;
// 		case Saturation_2:
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0x7c, 0x00);
// 			wrSensorReg8_8(0x7d, 0x02);
// 			wrSensorReg8_8(0x7c, 0x03);
// 			wrSensorReg8_8(0x7d, 0x28);
// 			wrSensorReg8_8(0x7d, 0x28);
// 		break;	
// 	}
// }


// void ArduCAM::OV2640_set_Brightness(uint8_t Brightness)
// {
// 	switch(Brightness)
// 	{
// 		case Brightness2:
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0x7c, 0x00);
// 			wrSensorReg8_8(0x7d, 0x04);
// 			wrSensorReg8_8(0x7c, 0x09);
// 			wrSensorReg8_8(0x7d, 0x40);
// 			wrSensorReg8_8(0x7d, 0x00);
// 		break;
// 		case Brightness1:
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0x7c, 0x00);
// 			wrSensorReg8_8(0x7d, 0x04);
// 			wrSensorReg8_8(0x7c, 0x09);
// 			wrSensorReg8_8(0x7d, 0x30);
// 			wrSensorReg8_8(0x7d, 0x00);
// 		break;	
// 		case Brightness0:
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0x7c, 0x00);
// 			wrSensorReg8_8(0x7d, 0x04);
// 			wrSensorReg8_8(0x7c, 0x09);
// 			wrSensorReg8_8(0x7d, 0x20);
// 			wrSensorReg8_8(0x7d, 0x00);
// 		break;
// 		case Brightness_1:
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0x7c, 0x00);
// 			wrSensorReg8_8(0x7d, 0x04);
// 			wrSensorReg8_8(0x7c, 0x09);
// 			wrSensorReg8_8(0x7d, 0x10);
// 			wrSensorReg8_8(0x7d, 0x00);
// 		break;
// 		case Brightness_2:
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0x7c, 0x00);
// 			wrSensorReg8_8(0x7d, 0x04);
// 			wrSensorReg8_8(0x7c, 0x09);
// 			wrSensorReg8_8(0x7d, 0x00);
// 			wrSensorReg8_8(0x7d, 0x00);
// 		break;	
// 	}
		
// }

// void ArduCAM::OV2640_set_Contrast(uint8_t Contrast)
// {
// 	switch(Contrast)
// 	{
// 		case Contrast2:
	
// 		wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0x7c, 0x00);
// 			wrSensorReg8_8(0x7d, 0x04);
// 			wrSensorReg8_8(0x7c, 0x07);
// 			wrSensorReg8_8(0x7d, 0x20);
// 			wrSensorReg8_8(0x7d, 0x28);
// 			wrSensorReg8_8(0x7d, 0x0c);
// 			wrSensorReg8_8(0x7d, 0x06);
// 		break;
// 		case Contrast1:
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0x7c, 0x00);
// 			wrSensorReg8_8(0x7d, 0x04);
// 			wrSensorReg8_8(0x7c, 0x07);
// 			wrSensorReg8_8(0x7d, 0x20);
// 			wrSensorReg8_8(0x7d, 0x24);
// 			wrSensorReg8_8(0x7d, 0x16);
// 			wrSensorReg8_8(0x7d, 0x06); 
// 		break;
// 		case Contrast0:
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0x7c, 0x00);
// 			wrSensorReg8_8(0x7d, 0x04);
// 			wrSensorReg8_8(0x7c, 0x07);
// 			wrSensorReg8_8(0x7d, 0x20);
// 			wrSensorReg8_8(0x7d, 0x20);
// 			wrSensorReg8_8(0x7d, 0x20);
// 			wrSensorReg8_8(0x7d, 0x06); 
// 		break;
// 		case Contrast_1:
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0x7c, 0x00);
// 			wrSensorReg8_8(0x7d, 0x04);
// 			wrSensorReg8_8(0x7c, 0x07);
// 			wrSensorReg8_8(0x7d, 0x20);
// 			wrSensorReg8_8(0x7d, 0x20);
// 			wrSensorReg8_8(0x7d, 0x2a);
// 		wrSensorReg8_8(0x7d, 0x06);	
// 		break;
// 		case Contrast_2:
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0x7c, 0x00);
// 			wrSensorReg8_8(0x7d, 0x04);
// 			wrSensorReg8_8(0x7c, 0x07);
// 			wrSensorReg8_8(0x7d, 0x20);
// 			wrSensorReg8_8(0x7d, 0x18);
// 			wrSensorReg8_8(0x7d, 0x34);
// 			wrSensorReg8_8(0x7d, 0x06);
// 		break;
// 	}
// }

// void ArduCAM::OV2640_set_Special_effects(uint8_t Special_effect)
// {
// 	switch(Special_effect)
// 	{
// 		case Antique:

// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0x7c, 0x00);
// 			wrSensorReg8_8(0x7d, 0x18);
// 			wrSensorReg8_8(0x7c, 0x05);
// 			wrSensorReg8_8(0x7d, 0x40);
// 			wrSensorReg8_8(0x7d, 0xa6);
// 		break;
// 		case Bluish:
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0x7c, 0x00);
// 			wrSensorReg8_8(0x7d, 0x18);
// 			wrSensorReg8_8(0x7c, 0x05);
// 			wrSensorReg8_8(0x7d, 0xa0);
// 			wrSensorReg8_8(0x7d, 0x40);
// 		break;
// 		case Greenish:
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0x7c, 0x00);
// 			wrSensorReg8_8(0x7d, 0x18);
// 			wrSensorReg8_8(0x7c, 0x05);
// 			wrSensorReg8_8(0x7d, 0x40);
// 			wrSensorReg8_8(0x7d, 0x40);
// 		break;
// 		case Reddish:
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0x7c, 0x00);
// 			wrSensorReg8_8(0x7d, 0x18);
// 			wrSensorReg8_8(0x7c, 0x05);
// 			wrSensorReg8_8(0x7d, 0x40);
// 			wrSensorReg8_8(0x7d, 0xc0);
// 		break;
// 		case BW:
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0x7c, 0x00);
// 			wrSensorReg8_8(0x7d, 0x18);
// 			wrSensorReg8_8(0x7c, 0x05);
// 			wrSensorReg8_8(0x7d, 0x80);
// 			wrSensorReg8_8(0x7d, 0x80);
// 		break;
// 		case Negative:
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0x7c, 0x00);
// 			wrSensorReg8_8(0x7d, 0x40);
// 			wrSensorReg8_8(0x7c, 0x05);
// 			wrSensorReg8_8(0x7d, 0x80);
// 			wrSensorReg8_8(0x7d, 0x80);
// 		break;
// 		case BWnegative:
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0x7c, 0x00);
// 			wrSensorReg8_8(0x7d, 0x58);
// 			wrSensorReg8_8(0x7c, 0x05);
// 			wrSensorReg8_8(0x7d, 0x80);
// 			wrSensorReg8_8(0x7d, 0x80);

// 		break;
// 		case Normal:
	
// 			wrSensorReg8_8(0xff, 0x00);
// 			wrSensorReg8_8(0x7c, 0x00);
// 			wrSensorReg8_8(0x7d, 0x00);
// 			wrSensorReg8_8(0x7c, 0x05);
// 			wrSensorReg8_8(0x7d, 0x80);
// 			wrSensorReg8_8(0x7d, 0x80);
		
// 		break;
				
// 	}
// }


	// Write 8 bit values to 8 bit register address
int ArduCAM::wrSensorRegs8_8(const struct sensor_reg reglist[])
{
		int err = 0;
	  uint16_t reg_addr = 0;
	  uint16_t reg_val = 0;
	  const struct sensor_reg *next = reglist;
	  while ((reg_addr != 0xff) | (reg_val != 0xff))
	  {
		// mySPI.readByte(this->device, &next->reg, )
	    reg_addr = pgm_read_word(&next->reg);
	    reg_val = pgm_read_word(&next->val);
	    err = wrSensorReg8_8(reg_addr, reg_val);
	    next++;
		yield();
	  }
	return 1;
}

	// Write 16 bit values to 8 bit register address
int ArduCAM::wrSensorRegs8_16(const struct sensor_reg reglist[])
{
		int err = 0;
	  unsigned int reg_addr, reg_val;
	  const struct sensor_reg *next = reglist;
	
	  while ((reg_addr != 0xff) | (reg_val != 0xffff))
	  {
	     reg_addr = pgm_read_word(&next->reg);
	     reg_val = pgm_read_word(&next->val);
	    err = wrSensorReg8_16(reg_addr, reg_val);
	    //  if (!err)
	    //return err;
	    next++;
			yield();
	  }
	return 1;
}

// Write 8 bit values to 16 bit register address
int ArduCAM::wrSensorRegs16_8(const struct sensor_reg reglist[])
{
		int err = 0;
	  unsigned int reg_addr;
	  unsigned char reg_val;
	  const struct sensor_reg *next = reglist;
	
	  while ((reg_addr != 0xffff) | (reg_val != 0xff))
	  {

	     reg_addr = pgm_read_word(&next->reg);
	     reg_val = pgm_read_word(&next->val);
	    err = wrSensorReg16_8(reg_addr, reg_val);
	    //if (!err)
	    //return err;
	    next++;
			yield();
	  }
	return 1;
}

//I2C Array Write 16bit address, 16bit data
int ArduCAM::wrSensorRegs16_16(const struct sensor_reg reglist[])
{
	  int err = 0;
	  unsigned int reg_addr, reg_val;
	  const struct sensor_reg *next = reglist;
	  reg_addr = pgm_read_word(&next->reg);
	  reg_val = pgm_read_word(&next->val);
	  while ((reg_addr != 0xffff) | (reg_val != 0xffff))
	  {
	    err = wrSensorReg16_16(reg_addr, reg_val);
	    //if (!err)
	    //   return err;
	    next++;
	    reg_addr = pgm_read_word(&next->reg);
	    reg_val = pgm_read_word(&next->val);
			    yield();
	  }
  return 1;
}



// Read/write 8 bit value to/from 8 bit register address	
unsigned char ArduCAM::wrSensorReg8_8(int regID, int regDat)
{
	this->myI2C.write(this->sensor_addr, regID, regDat);
	vTaskDelay(pdMS_TO_TICKS(1));
	return 1;
}

unsigned char ArduCAM::rdSensorReg8_8(uint8_t regID, uint8_t* regDat)
{	
		Wire.beginTransmission(sensor_addr >> 1);
	  Wire.write(regID & 0x00FF);
	  Wire.endTransmission();
	
	  Wire.requestFrom((sensor_addr >> 1), 1);
	  if (Wire.available())
	    *regDat = Wire.read();
	   vTaskDelay(pdMS_TO_TICKS(1));
	return 1;
	
}
// Read/write 16 bit value to/from 8 bit register address
unsigned char ArduCAM::wrSensorReg8_16(int regID, int regDat)
{
		Wire.beginTransmission(sensor_addr >> 1);
	  Wire.write(regID & 0x00FF);
	
	  Wire.write(regDat >> 8);            // sends data byte, MSB first
	  Wire.write(regDat & 0x00FF);
	  if (Wire.endTransmission())
	  {
	    return 0;
	  }	
	   vTaskDelay(pdMS_TO_TICKS(1));
	return 1;
}
unsigned char ArduCAM::rdSensorReg8_16(uint8_t regID, uint16_t* regDat)
{
  	uint8_t temp;
	  Wire.beginTransmission(sensor_addr >> 1);
	  Wire.write(regID);
	  Wire.endTransmission();
	
	  Wire.requestFrom((sensor_addr >> 1), 2);
	  if (Wire.available())
	  {
	    temp = Wire.read();
	    *regDat = (temp << 8) | Wire.read();
	  }
	   vTaskDelay(pdMS_TO_TICKS(1));
  	return 1;
}

// Read/write 8 bit value to/from 16 bit register address
unsigned char ArduCAM::wrSensorReg16_8(int regID, int regDat)
{
		Wire.beginTransmission(sensor_addr >> 1);
	  Wire.write(regID >> 8);            // sends instruction byte, MSB first
	  Wire.write(regID & 0x00FF);
	  Wire.write(regDat & 0x00FF);
	  if (Wire.endTransmission())
	  {
	    return 0;
	  }
	  vTaskDelay(pdMS_TO_TICKS(1));
	return 1;
}
unsigned char ArduCAM::rdSensorReg16_8(uint16_t regID, uint8_t* regDat)
{
		Wire.beginTransmission(sensor_addr >> 1);
	  Wire.write(regID >> 8);
	  Wire.write(regID & 0x00FF);
	  Wire.endTransmission();
	  Wire.requestFrom((sensor_addr >> 1), 1);
	  if (Wire.available())
	  {
	    *regDat = Wire.read();
	  }
	  vTaskDelay(pdMS_TO_TICKS(1));
	return 1;
}

//I2C Write 16bit address, 16bit data
unsigned char ArduCAM::wrSensorReg16_16(int regID, int regDat)
{
	  Wire.beginTransmission(sensor_addr >> 1);
	  Wire.write(regID >> 8);            // sends instruction byte, MSB first
	  Wire.write(regID & 0x00FF);
	  Wire.write(regDat >> 8);            // sends data byte, MSB first
	  Wire.write(regDat & 0x00FF);
	  if (Wire.endTransmission())
	  {
	    return 0;
	  }
	  vTaskDelay(pdMS_TO_TICKS(1));
  return (1);
}

//I2C Read 16bit address, 16bit data
unsigned char ArduCAM::rdSensorReg16_16(uint16_t regID, uint16_t* regDat)
{
	  uint16_t temp;
	  Wire.beginTransmission(sensor_addr >> 1);
	  Wire.write(regID >> 8);
	  Wire.write(regID & 0x00FF);
	  Wire.endTransmission();
	  Wire.requestFrom((sensor_addr >> 1), 2);
	  if (Wire.available())
	  {
	    temp = Wire.read();
	    *regDat = (temp << 8) | Wire.read();
	  }
	  vTaskDelay(pdMS_TO_TICKS(1));
  return (1);
}

inline void ArduCAM::setDataBits(uint16_t bits) {
  const uint32_t mask = ~((SPIMMOSI << SPILMOSI) | (SPIMMISO << SPILMISO));
  bits--;
  SPI1U1 = ((SPI1U1 & mask) | ((bits << SPILMOSI) | (bits << SPILMISO)));
}

void ArduCAM::transferBytes_(uint8_t * out, uint8_t * in, uint8_t size) {
  while (SPI1CMD & SPIBUSY) {}
  // Set in/out Bits to transfer

  setDataBits(size * 8);

  volatile uint32_t * fifoPtr = &SPI1W0;
  uint8_t dataSize = ((size + 3) / 4);

  if (out) {
    uint32_t * dataPtr = (uint32_t*) out;
    while (dataSize--) {
      *fifoPtr = *dataPtr;
      dataPtr++;
      fifoPtr++;
    }
  } else {
    // no out data only read fill with dummy data!
    while (dataSize--) {
      *fifoPtr = 0xFFFFFFFF;
      fifoPtr++;
    }
  }

  SPI1CMD |= SPIBUSY;
  while (SPI1CMD & SPIBUSY) {}

  if (in) {
    volatile uint8_t * fifoPtr8 = (volatile uint8_t *) &SPI1W0;
    dataSize = size;
    while (dataSize--) {
      *in = *fifoPtr8;
      in++;
      fifoPtr8++;
    }
  }
}

void ArduCAM::transferBytes(uint8_t * out, uint8_t * in, uint32_t size) {
  while (size) {
    if (size > 64) {
      transferBytes_(out, in, 64);
      size -= 64;
      if (out) out += 64;
      if (in) in += 64;
    } else {
      transferBytes_(out, in, size);
      size = 0;
    }
  }
}