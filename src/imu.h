#ifndef IMU_INCLUDED
#define IMU_INCLUDED
#include<stdint.h>
#include "vectormath.h"


#define GRAVITATIONAL_ACCELERATION (9.80665F)

#define IMU_ADDRESS 0x6A
#define IMU_ID 0x6B
#define IMU_REGISTER_WHOAMI 0xF
#define IMU_REGISTER_FIFO_CONFIG 0xA
#define IMU_REGISTER_ACCEL_CONFIG 0x10
#define IMU_REGISTER_GYRO_CONFIG 0x11
#define IMU_REGISTER_INTERRUPT_CONFIG 0xD

typedef enum data_rate
	{
	IMU_RATE_SHUTDOWN,
	IMU_RATE_12_5_HZ,
	IMU_RATE_26_HZ,
	IMU_RATE_52_HZ,
	IMU_RATE_104_HZ,
	IMU_RATE_208_HZ,
	IMU_RATE_416_HZ,
	IMU_RATE_833_HZ,
	IMU_RATE_1_66K_HZ,
	IMU_RATE_3_33K_HZ,
	IMU_RATE_6_66K_HZ,
	}data_rate_t;

typedef enum accel_range
	{
	ACCEL_RANGE_2_G,
	ACCEL_RANGE_16_G,
	ACCEL_RANGE_4_G,
	ACCEL_RANGE_8_G
	}accel_range_t;

typedef enum gyro_range
	{
	GYRO_RANGE_125_DPS = 0b0010,
	GYRO_RANGE_250_DPS = 0b0000,
	GYRO_RANGE_500_DPS = 0b0100,
	GYRO_RANGE_1000_DPS = 0b1000,
	GYRO_RANGE_2000_DPS = 0b1100,
	GYRO_RANGE_4000_DPS = 0b0001
	}gyro_range_t;

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


const uint64_t SAMPLE_TIMESTAMP_MASK=0x3FFFFFFFFFFFFFFFULL;
const uint64_t SAMPLE_HAS_MAG=0x4000000000000000ULL;

typedef struct
{
uint64_t timestamp;
vector3_t accel;
vector3_t gyro;
vector3_t mag;
}imu_t;

void imu_reset_calibration();
void imu_set_calibration(vector3_t accel_bias_new,double* accel_matrix_new,vector3_t gyro_bias_new,double* gyro_matrix_new,vector3_t mag_bias_new,double* mag_matrix_new);
bool imu_load_calibration(const char* path);

bool imu_init();
int imu_available();
bool imu_read(uint64_t timestamp);
bool imu_get(imu_t* sample);
int imu_get(imu_t* samples,int num);

#endif
