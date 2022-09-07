#include "Wire.h"
#include "bmp3.h"
#include "vectormath.h"

struct bmp3_dev sensor;

#define I2C_MAX_BUFFER_SIZE 32


bool i2c_write(const uint8_t addr,const uint8_t reg_addr,const uint8_t *buffer,size_t len)
{
	if(len>I2C_MAX_BUFFER_SIZE)return false;
Wire1.beginTransmission(addr);

	if(Wire1.write(&reg_addr,1)!=1)return false;
	if(buffer!=NULL&&Wire1.write(buffer,len)!=len)return false;
int status=Wire1.endTransmission();
return status==0;
}



bool i2c_read(const uint8_t addr,uint8_t reg_addr,uint8_t *buffer,size_t len)
{
	if(len>I2C_MAX_BUFFER_SIZE)return false;
	if (!i2c_write(addr,reg_addr,NULL,0))return false;

size_t recv=Wire1.requestFrom(addr,(uint8_t)len,true);
	if(recv!=len)return false;

	for(uint16_t i=0;i<len;i++)buffer[i]=Wire1.read();

return true;
}


int8_t read_func(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	if(!i2c_read(0x77,reg_addr,reg_data,len))return 1;
return 0;
} 

int8_t write_func(uint8_t reg_addr,const uint8_t* reg_data,uint32_t len,void *intf_ptr)
{
	if(!i2c_write(0x77,reg_addr,reg_data,len))return 1;
return 0;
}


 
static void delay_usec(uint32_t us, void *intf_ptr)
{
delay(us);
}

/*
int8_t bmp3_get_regs2(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct bmp3_dev *dev)
{
dev->intf_rslt = dev->read(reg_addr,reg_data,len,dev->intf_ptr);

        if(dev->intf_rslt != BMP3_INTF_RET_SUCCESS)return BMP3_E_COMM_FAIL;
return BMP3_OK;
}

int8_t bmp3_set_regs2(uint8_t *reg_addr, const uint8_t *reg_data, uint32_t len, struct bmp3_dev *dev)
{

        if (len==0)return BMP3_E_INVALID_LEN;
dev->intf_rslt = dev->write(reg_addr,reg_data,len,dev->intf_ptr);

    if (dev->intf_rslt != BMP3_INTF_RET_SUCCESS)
    {
	Serial.println(dev->intf_rslt!=BMP3_INTF_RET_SUCCESS);
	return BMP3_E_COMM_FAIL;
    }
return BMP3_OK;
}
*/

static int8_t cal_crc(uint8_t seed, uint8_t data) {
  int8_t poly = 0x1D;
  int8_t var2;
  uint8_t i;

  for (i = 0; i < 8; i++) {
    if ((seed & 0x80) ^ (data & 0x80)) {
      var2 = 1;
    } else {
      var2 = 0;
    }

    seed = (seed & 0x7F) << 1;
    data = (data & 0x7F) << 1;
    seed = seed ^ (uint8_t)(poly * var2);
  }

  return (int8_t)seed;
}


static int8_t validate_trimming_param(struct bmp3_dev *dev) {
  int8_t rslt;
  uint8_t crc = 0xFF;
  uint8_t stored_crc;
  uint8_t trim_param[21];
  uint8_t i;

  rslt = bmp3_get_regs(BMP3_REG_CALIB_DATA, trim_param, 21, dev);
  if (rslt == BMP3_OK) {
    for (i = 0; i < 21; i++) {
      crc = (uint8_t)cal_crc(crc, trim_param[i]);
    }

    crc = (crc ^ 0xFF);
    rslt = bmp3_get_regs(0x30, &stored_crc, 1, dev);
    if (stored_crc != crc) {
      rslt = -1;
    }
  }

  return rslt;
}


bool pressure_sample(double* temp,double* press)
{
struct bmp3_data data;
int rslt = bmp3_get_sensor_data(BMP3_TEMP|BMP3_PRESS,&data,&sensor);
	if(rslt!=BMP3_OK)return false;

*temp=data.temperature;
*press=data.pressure;
return true;
}

bool pressure_init()
{
sensor.chip_id=0x77;
sensor.intf=BMP3_I2C_INTF;
sensor.read=&read_func;
sensor.write=&write_func;
sensor.intf_ptr=&Wire1;
sensor.dummy_byte=0;
sensor.delay_us = delay_usec;

int rslt = bmp3_init(&sensor);
	if (rslt!=BMP3_OK)return false;

rslt = validate_trimming_param(&sensor);

	if (rslt!=BMP3_OK)return false;

sensor.settings.temp_en=BMP3_ENABLE;
sensor.settings.press_en=BMP3_ENABLE;
sensor.settings.odr_filter.temp_os=BMP3_OVERSAMPLING_16X;
sensor.settings.odr_filter.press_os=BMP3_OVERSAMPLING_16X;
sensor.settings.odr_filter.iir_filter=BMP3_IIR_FILTER_COEFF_15;
sensor.settings.odr_filter.odr = BMP3_ODR_12_5_HZ;
sensor.settings.op_mode = BMP3_MODE_NORMAL;

rslt = bmp3_set_sensor_settings(BMP3_SEL_TEMP_EN|BMP3_SEL_PRESS_EN|BMP3_SEL_TEMP_OS|BMP3_SEL_PRESS_OS|BMP3_SEL_IIR_FILTER|BMP3_SEL_ODR,&sensor);

	if (rslt != BMP3_OK)return false;

rslt = bmp3_set_op_mode(&sensor);
	if (rslt != BMP3_OK)return false;

return true;
}

/*
	enum
	{
	QMC_MODE_STANDBY=0x00,
	QMC_MODE_CONTINUOUS=0x01
	};

	enum
	{
	QMC_ODR_10_HZ=0x00,
	QMC_ODR_50_HZ=0x04,
	QMC_ODR_100_HZ=0x08,
	QMC_ODR_200_HZ=0x0C
	};

	enum
	{
	QMC_SCALE_2G=0x00,
	QMC_SCALE_8G=0x10
	};

	enum
	{
	QMC_OVERSAMPLING_512X=0x00,
	QMC_OVERSAMPLING_256X=0x40,
	QMC_OVERSAMPLING_128X=0x80,
	QMC_OVERSAMPLING_64X=0xC0 
	};

bool magnetometer_init()
{
uint8_t value=0x01;
	if(!i2c_write(0xD,0x0B,&value,1))return false;
value=QMC_MODE_CONTINUOUS|QMC_ODR_10_HZ|QMC_SCALE_2G|QMC_OVERSAMPLING_64X;
	if(!i2c_write(0xD,0x09,&value,1))return false;
return true;
}

int magnetometer_sample(vector3_t* vec)
{
uint8_t buffer[6];
	if(!i2c_read(0xD,0x00,buffer,6))return 1;
	

float x=((int16_t)(buffer[0]|buffer[1]<<8))/16384.0;
float y=((int16_t)(buffer[2]|buffer[3]<<8))/16384.0;
float z=((int16_t)(buffer[4]|buffer[5]<<8))/16384.0;

//return vector3(x,y,z);

float A[3][3]={
	{ 2.79254877,-0.01239678,-0.00426029},
	{-0.01239678, 2.8476055 , 0.01950636},
	{-0.00426029, 0.01950636, 2.80041325}
	};


float b[3]={-0.10802357,-0.03297308,0.00781366};


x-=b[0];
y-=b[1];
z-=b[2];

vec->x=A[0][0]*x+A[0][1]*y+A[0][2]*z;
vec->y=A[1][0]*x+A[1][1]*y+A[1][2]*z;
vec->z=A[2][0]*x+A[2][1]*y+A[2][2]*z;

return 0;
}
*/
