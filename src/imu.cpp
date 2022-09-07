#include <Arduino.h>
#include<stdint.h>
#include "i2c.h"
#include "io.h"
#include "imu.h"
#include "timer.h"

ring_buffer<sample_t,256> sample_buffer;
vector3_t accel_bias={0,0,0};
vector3_t gyro_bias={0,0,0};
vector3_t mag_bias={0,0,0};
double accel_matrix[9]={1,0,0,0,1,0,0,0,1};
double gyro_matrix[9]={1,0,0,0,1,0,0,0,1};
double mag_matrix[9]={1,0,0,0,1,0,0,0,1};

vector3_t apply_calibration(vector3_t sample,vector3_t bias,double* matrix)
{
vector3_t vec=vector3_sub(sample,bias);
return vector3(matrix[0]*vec.x+matrix[1]*vec.y+matrix[2]*vec.z,matrix[3]*vec.x+matrix[4]*vec.y+matrix[5]*vec.z,matrix[6]*vec.x+matrix[7]*vec.y+matrix[8]*vec.z);
}

bool imu_read(uint64_t timestamp)
{
sample_t sample;
sample.timestamp=timestamp;

//Read data from IMU
uint8_t bytes[12];
	if(i2c_bus[0].read(IMU_ADDRESS,0x22,bytes,12))return false;

sample.accel.x=((int16_t)((bytes[7]<<8)|bytes[6]))*(2.0/32768.0)*GRAVITATIONAL_ACCELERATION;
sample.accel.y=((int16_t)((bytes[9]<<8)|bytes[8]))*(2.0/32768.0)*GRAVITATIONAL_ACCELERATION;
sample.accel.z=((int16_t)((bytes[11]<<8)|bytes[10]))*(2.0/32768.0)*GRAVITATIONAL_ACCELERATION;
sample.accel=apply_calibration(sample.accel,accel_bias,accel_matrix);

double scale=0.00875;
double degree_to_rad=M_PI/180;
sample.gyro.x=((int16_t)((bytes[1]<<8)|bytes[0]))*scale*degree_to_rad;
sample.gyro.y=((int16_t)((bytes[3]<<8)|bytes[2]))*scale*degree_to_rad;
sample.gyro.z=((int16_t)((bytes[5]<<8)|bytes[4]))*scale*degree_to_rad;
sample.gyro=apply_calibration(sample.gyro,gyro_bias,gyro_matrix);

	//Only sample magnetometer 10 times per second
	if(timestamp%100==0)
	{
		if(!i2c_bus[1].read(0xD,0x00,bytes,6))
		{
		sample.timestamp|=SAMPLE_HAS_MAG;
		sample.mag.x=((int16_t)(bytes[0]|bytes[1]<<8))/16384.0;
		sample.mag.y=((int16_t)(bytes[2]|bytes[3]<<8))/16384.0;
		sample.mag.z=((int16_t)(bytes[4]|bytes[5]<<8))/16384.0;
		sample.mag=apply_calibration(sample.mag,mag_bias,mag_matrix);
		}
	}
	
	if(sample_buffer.free()==0)sample_buffer.discard(1);
sample_buffer.push(&sample);
return true;
}
void imu_reset_calibration()
{
accel_bias=vector3(0,0,0);
gyro_bias=vector3(0,0,0);
mag_bias=vector3(0,0,0);
double identity[9]={1,0,0,0,1,0,0,0,1};
	for(int i=0;i<9;i++)
	{
	accel_matrix[i]=identity[i];
	gyro_matrix[i]=identity[i];
	mag_matrix[i]=identity[i];
	}
}

void imu_set_calibration(vector3_t accel_bias_new,double* accel_matrix_new,vector3_t gyro_bias_new,double* gyro_matrix_new,vector3_t mag_bias_new,double* mag_matrix_new)
{
accel_bias=accel_bias_new;
gyro_bias=gyro_bias_new;
mag_bias=mag_bias_new;
	for(int i=0;i<9;i++)
	{
	accel_matrix[i]=accel_matrix_new[i];
	gyro_matrix[i]=gyro_matrix_new[i];
	mag_matrix[i]=mag_matrix_new[i];
	}
}





bool is_whitespace(uint8_t c)
{
return c==' '||c=='\t';
}

void consume_whitespace(int file)
{
	while(available(file)>0)
	{
	uint8_t byte=peek_uint8(file);
		if(is_whitespace(byte))discard(file,1);
		else return;
	}
}



bool valid_name_char(uint8_t c)
{
return (c>='a'&&c<='z')||(c>='A'&&c<='Z')||c=='_';
}

size_t parse_name(int file,uint8_t* buf,size_t len)
{
consume_whitespace(file);
return read_while(file,valid_name_char,buf,len);
}


bool imu_load_calibration(const char* path)
{
log_message(LOG_IMU|LOG_DEBUG,"Loading calibration file");

io_file file;
	if(file.open(path,O_READ_ONLY));
	{
	return false;
	}

vector3_t accel_bias;
vector3_t gyro_bias;
vector3_t mag_bias;
double accel_matrix[9];
double gyro_matrix[9];
double mag_matrix[9];

char str[512];
str[0]=0;

int values_loaded=0;
	while(file.available(file))
	{
	//Parse var name
	uint8_t name[32];
	size_t name_len=parse_name(file,name,32);
		if(name_len==0)return false;//TODO useful error messages
	//Check for equal sign
	consume_whitespace(file);
		if(read_uint8(file)!='=')return false;
	name[name_len]=0;
	log_message(LOG_IMU,"%s\n",name);
	
	consume_whitespace(file);
		if(read_uint8(file)!='{')return false;
	}
	//
	//Parse values
	/*
		if(strcmp(name,"accel_bias")==0&&sscanf(str+i+1,"{%lf,%lf,%lf}",&(accel_bias.x),&(accel_bias.y),&(accel_bias.z))==3)values_loaded++;
		else if(strcmp(name,"gyro_bias")==0&&sscanf(str+i+1,"{%lf,%lf,%lf}",&(gyro_bias.x),&(gyro_bias.y),&(gyro_bias.z))==3)values_loaded++;
		else if(strcmp(name,"mag_bias")==0&&sscanf(str+i+1,"{%lf,%lf,%lf}",&(mag_bias.x),&(mag_bias.y),&(mag_bias.z))==3)values_loaded++;
		else if(strcmp(name,"accel_matrix")==0&&sscanf(str+i+1,"{%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf}",accel_matrix,accel_matrix+1,accel_matrix+2,accel_matrix+3,accel_matrix+4,accel_matrix+5,accel_matrix+6,accel_matrix+7,accel_matrix+8)==9)values_loaded++;
		else if(strcmp(name,"gyro_matrix")==0&&sscanf(str+i+1,"{%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf}",gyro_matrix,gyro_matrix+1,gyro_matrix+2,gyro_matrix+3,gyro_matrix+4,gyro_matrix+5,gyro_matrix+6,gyro_matrix+7,gyro_matrix+8)==9)values_loaded++;
		else if(strcmp(name,"mag_matrix")==0&&sscanf(str+i+1,"{%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf}",mag_matrix,mag_matrix+1,mag_matrix+2,mag_matrix+3,mag_matrix+4,mag_matrix+5,mag_matrix+6,mag_matrix+7,mag_matrix+8)==9)values_loaded++;
	*/
//		while(read_uint8()
/*	}
	if(values_loaded!=6)return 0;


imu_set_calibration(accel_bias,accel_matrix,gyro_bias,gyro_matrix,mag_bias,mag_matrix);
*/
file.close(file);
return true;
}



bool imu_init()
{

uint8_t id=0;
	if(i2c_bus[0].read(IMU_ADDRESS,IMU_REGISTER_WHOAMI,&id,1)||id!=IMU_ID)
	{
	Serial.println("init: IMU: ISM330DHCX not detected");
	return false;
	}
//Configure accelerometer
uint8_t data=(IMU_RATE_104_HZ<<4)|(ACCEL_RANGE_2_G<<2);
	if(i2c_bus[0].write(IMU_ADDRESS,IMU_REGISTER_ACCEL_CONFIG,&data,1))
	{
	Serial.println("init: IMU: Failed to write accelerometer configuration");
	return false;
	}
//Configure gyroscope
data=(IMU_RATE_104_HZ<<4)|(GYRO_RANGE_250_DPS<<2);
	if(i2c_bus[0].write(IMU_ADDRESS,IMU_REGISTER_GYRO_CONFIG,&data,1))
	{
	Serial.println("init: IMU: Failed to write gyroscope configuration");
	return false;
	}

//Configure magnetometer
//TODO check if magnetometer is present like we do with the other chip
uint8_t value=0x01;
	if(i2c_bus[1].write(0xD,0x0B,&value,1))
	{
	Serial.println("init: IMU: Failed to write magnetometer configuration");
	return false;
	}
value=QMC_MODE_CONTINUOUS|QMC_ODR_10_HZ|QMC_SCALE_2G|QMC_OVERSAMPLING_64X;
	if(i2c_bus[1].write(0xD,0x09,&value,1))
	{
	Serial.println("init: IMU: Failed to write magnetometer configuration");
	return false;
	}
return true;
}

int imu_available_samples()
{
timer_wait_for_lock();
size_t avail=sample_buffer.available();
timer_unlock();
return avail;
}

bool imu_get_next_sample(sample_t* sample)
{
timer_wait_for_lock();
bool success=sample_buffer.pop(sample);
timer_unlock();
return success;
}
