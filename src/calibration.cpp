#include <Arduino.h>
#include<arm_math.h>
#include "vectormath.h"
#include "calibration.h"
#include "led.h"
#include "io.h"
#include "imu.h"

#define CALIBRATION_BUFFER_SIZE 128
#define MAX_MAG_SAMPLES 2048
#define MAX_GYRO_ROTATIONS 64

imu_t calibration_buffer[CALIBRATION_BUFFER_SIZE];
int calibration_state=0;
int accel_batches_processed=0;
int gyro_batches_processed=0;
int gyro_rotations_recorded=0;
int completed_axes=0;
int calibration_axis=0;
vector3_t accel_mean_total;
vector3_t accel_var_total;
vector3_t gyro_mean_total;
vector3_t mag_mean_total;
vector3_t mag_var_total;
vector3_t accel_values[6];
vector3_t gyro_values[6];
vector3_t mag_values[6];
vector3_t gyro_rotations[MAX_GYRO_ROTATIONS];
vector3_t gyro_rotation_mag_values[MAX_GYRO_ROTATIONS];
vector3_t gyro_integral_total;
int num_mag_samples=0;
vector3_t mag_samples[MAX_MAG_SAMPLES];
double mag_sample_rotations[MAX_MAG_SAMPLES];

//Final calibration result
vector3_t accel_bias_new;
double accel_matrix_new[9];
vector3_t gyro_bias_new;
double gyro_matrix_new[9];
vector3_t mag_bias_new;
double mag_matrix_new[9];



void calibration_accel_start_recording(int axis)
{
calibration_state=CALIBRATION_ACCEL_RECORDING;
calibration_axis=axis;
led_set(SOLID_YELLOW_BLINK_GREEN);
accel_batches_processed=0;
accel_mean_total=vector3(0,0,0);
accel_var_total=vector3(0,0,0);
mag_mean_total=vector3(0,0,0);
mag_var_total=vector3(0,0,0);
}

void calibration_accel_record_batch(vector3_t accel_mean,vector3_t gyro_mean,vector3_t accel_var)
{
//Add current batch to total
accel_mean_total=vector3_add(accel_mean_total,accel_mean);
accel_var_total=vector3_add(accel_var_total,accel_var);
gyro_mean_total=vector3_add(gyro_mean_total,gyro_mean);
accel_batches_processed+=1;	
gyro_batches_processed+=1;	
}

void calibration_accel_terminate_recording()
{
led_set(BLINK_YELLOW);
calibration_state=CALIBRATION_ACCEL_WAITING;
}

void calibration_accel_finish_recording()
{
vector3_t accel_mean=vector3_scale(accel_mean_total,1.0/accel_batches_processed);
accel_values[calibration_axis]=accel_mean;
completed_axes|=1<<calibration_axis;
led_set(SOLID_GREEN);
calibration_state=CALIBRATION_ACCEL_WAITING;
}

void calibration_complete_accel()
{
//Compute gyro bias
gyro_bias_new=vector3_scale(gyro_mean_total,1.0/gyro_batches_processed);

//Compute accel bias and scale factor matrix
accel_bias_new=vector3_scale(vector3_add(vector3_add(vector3_add(accel_values[0],accel_values[1]),vector3_add(accel_values[2],accel_values[3])),vector3_add(accel_values[4],accel_values[5])),1.0/6.0);

vector3_t x=vector3_scale(vector3_sub(accel_values[0],accel_values[1]),0.5/9.81);
vector3_t y=vector3_scale(vector3_sub(accel_values[2],accel_values[3]),0.5/9.81);
vector3_t z=vector3_scale(vector3_sub(accel_values[4],accel_values[5]),0.5/9.81);

double A_inv[9]={x.x,y.x,z.x,x.y,y.y,z.y,x.z,y.z,z.z};
arm_matrix_instance_f64 src={3,3,A_inv};
arm_matrix_instance_f64 dst={3,3,accel_matrix_new};
arm_mat_inverse_f64(&src,&dst);

//Start gyro calibration
led_set(BLINK_BOTH);
calibration_state=CALIBRATION_GYRO_READY|CALIBRATION_BLINK_DELAY_LONG;
completed_axes=0;
gyro_batches_processed=0;
}



void calibration_gyro_reset()
{
Serial.println("DEBUG: calibration_gyro_reset()");
gyro_integral_total=vector3(0,0,0);
gyro_batches_processed=0;
num_mag_samples=0;
}

void calibration_gyro_ready(int axis)
{
Serial.println("DEBUG: calibration_gyro_ready");
calibration_axis=axis;

	if(completed_axes&(1<<axis))led_set(SOLID_GREEN);
	else led_set(SOLID_BOTH);
calibration_gyro_reset();
}

void calibration_gyro_start_recording(int axis)
{
Serial.println("DEBUG: calibration_gyro_start_recording");
calibration_state=CALIBRATION_GYRO_RECORDING;
calibration_axis=axis;
gyro_rotations_recorded=0;
}

void calibration_gyro_record_value()
{
Serial.println("DEBUG: calibration_gyro_record_value");
	if(gyro_rotations_recorded==MAX_GYRO_ROTATIONS)return;
led_set(SOLID_YELLOW_BLINK_GREEN);
calibration_state|=CALIBRATION_BLINK_DELAY_SHORT;

double rotation_scale=1.0/fmax(gyro_integral_total.x,fmax(gyro_integral_total.y,gyro_integral_total.z));

double mag_integral_rot=0.0;
vector3_t mag_integral=vector3(0,0,0);
	for(int i=0;i<num_mag_samples-1;i++)
	{
	double rotation_delta=rotation_scale*(mag_sample_rotations[i+1]-mag_sample_rotations[i]);
	mag_integral=vector3_add(mag_integral,vector3_scale(vector3_add(mag_samples[i],mag_samples[i+1]),0.5*rotation_delta));
	mag_integral_rot+=rotation_delta;
	}

mag_integral=vector3_add(mag_integral,vector3_scale(mag_samples[0],rotation_scale*mag_sample_rotations[0]));
mag_integral=vector3_add(mag_integral,vector3_scale(mag_samples[num_mag_samples-1],1-rotation_scale*mag_sample_rotations[num_mag_samples-1]));
mag_integral_rot+=rotation_scale*mag_sample_rotations[0]+1-rotation_scale*mag_sample_rotations[num_mag_samples-1];

gyro_rotations[gyro_rotations_recorded]=gyro_integral_total;
gyro_rotation_mag_values[gyro_rotations_recorded]=mag_integral;
gyro_rotations_recorded++;

calibration_gyro_reset();
}

void calibration_gyro_reject()
{
Serial.println("DEBUG: calibration_gyro_reject");
led_set(SOLID_BOTH);
calibration_gyro_reset();
}

void calibration_gyro_terminate_recording()
{
Serial.println("DEBUG: calibration_gyro_terminate_recording");
led_set(BLINK_YELLOW);
calibration_state=CALIBRATION_GYRO_READY;
calibration_gyro_reset();
}

void calibration_gyro_finish_recording(vector3_t gyro_mean,vector3_t mag_mean)
{
Serial.println("DEBUG: calibration_gyro_finish_recording");
led_set(SOLID_GREEN);
Serial.println("Finished gyro recording");
gyro_values[calibration_axis]=gyro_mean;
mag_values[calibration_axis]=mag_mean;
calibration_state=CALIBRATION_GYRO_READY;
completed_axes|=1<<calibration_axis;
calibration_gyro_reset();
}

void calibration_complete_gyro()
{
Serial.println("DEBUG: calibration_complete_gyro");
//Compute gyro scale factor matrix
vector3_t x=vector3_scale(vector3_sub(gyro_values[0],gyro_values[1]),0.25/M_PI);
vector3_t y=vector3_scale(vector3_sub(gyro_values[2],gyro_values[3]),0.25/M_PI);
vector3_t z=vector3_scale(vector3_sub(gyro_values[4],gyro_values[5]),0.25/M_PI);

double A_inv[9]={x.x,y.x,z.x,x.y,y.y,z.y,x.z,y.z,z.z};
arm_matrix_instance_f64 src={3,3,A_inv};
arm_matrix_instance_f64 dst={3,3,gyro_matrix_new};
arm_mat_inverse_f64(&src,&dst);

//Compute mag bias and scale factor matrix
mag_bias_new=vector3_scale(vector3_add(vector3_add(vector3_add(mag_values[0],mag_values[1]),vector3_add(mag_values[2],mag_values[3])),vector3_add(mag_values[4],mag_values[5])),1.0/6.0);

x=vector3_scale(vector3_sub(mag_values[0],mag_values[1]),0.5);
y=vector3_scale(vector3_sub(mag_values[2],mag_values[3]),0.5);
z=vector3_scale(vector3_sub(mag_values[4],mag_values[5]),0.5);

double A_inv2[9]={x.x,y.x,z.x,x.y,y.y,z.y,x.z,y.z,z.z};
arm_matrix_instance_f64 src2={3,3,A_inv2};
arm_matrix_instance_f64 dst2={3,3,mag_matrix_new};
arm_mat_inverse_f64(&src2,&dst2);

//Calibration is complete
led_set(BLINK_GREEN);
calibration_state=CALIBRATION_COMPLETED;
}



int calibration_get_axis(vector3_t vector,double magnitude)
{
	if(vector.x>0.9*magnitude&&fabs(vector.y)<0.1*magnitude&&fabs(vector.z)<0.1*magnitude)return AXIS_X_PLUS;
	else if(vector.x<-0.9*magnitude&&fabs(vector.y)<0.1*magnitude&&fabs(vector.z)<0.1*magnitude)return AXIS_X_MINUS;
	else if(vector.y>0.9*magnitude&&fabs(vector.x)<0.1*magnitude&&fabs(vector.z)<0.1*magnitude)return AXIS_Y_PLUS;
	else if(vector.y<-0.9*magnitude&&fabs(vector.x)<0.1*magnitude&&fabs(vector.z)<0.1*magnitude)return AXIS_Y_MINUS;
	else if(vector.z>0.9*magnitude&&fabs(vector.y)<0.1*magnitude&&fabs(vector.x)<0.1*magnitude)return AXIS_Z_PLUS;
	else if(vector.z<-0.9*magnitude&&fabs(vector.y)<0.1*magnitude&&fabs(vector.x)<0.1*magnitude)return AXIS_Z_MINUS;
return AXIS_NONE;
}

int calibration_process_samples(vector3_t* batch_accel_mean,vector3_t* batch_gyro_mean,vector3_t* batch_accel_var)
{
	if(imu_available()>=CALIBRATION_BUFFER_SIZE)
	{
	//Get samples
		if(imu_get(calibration_buffer,CALIBRATION_BUFFER_SIZE)!=CALIBRATION_BUFFER_SIZE)
		{
		log_message(LOG_CALIBRATION|LOG_ERROR,"Did not recieve expected number of IMU samples");
		return 0;
		}
	
	//Compute sample means
	vector3_t accel_mean=vector3(0,0,0);
	vector3_t gyro_mean=vector3(0,0,0);
		for(int i=0;i<CALIBRATION_BUFFER_SIZE;i++)
		{
		accel_mean=vector3_add(accel_mean,calibration_buffer[i].accel);
		gyro_mean=vector3_add(gyro_mean,calibration_buffer[i].gyro);
		}
	accel_mean=vector3_scale(accel_mean,1.0/CALIBRATION_BUFFER_SIZE);
	gyro_mean=vector3_scale(gyro_mean,1.0/CALIBRATION_BUFFER_SIZE);

	//Compute sample variance
	vector3_t accel_var=vector3(0,0,0);
		for(int i=0;i<CALIBRATION_BUFFER_SIZE;i++)
		{
		vector3_t d=vector3_sub(calibration_buffer[i].accel,accel_mean);
		accel_var=vector3_add(accel_var,vector3(d.x*d.x,d.y*d.y,d.z*d.z));
		}
	accel_var=vector3_scale(accel_var,1.0/(CALIBRATION_BUFFER_SIZE-1));


	*batch_accel_mean=accel_mean;
	*batch_accel_var=accel_var;
	*batch_gyro_mean=gyro_mean;
	return 1;
	}
return 0;
}

void calibration_init()
{
calibration_axis=AXIS_NONE;
completed_axes=0;
gyro_mean_total=vector3(0,0,0);
gyro_batches_processed=0;
calibration_state=CALIBRATION_ACCEL_WAITING|CALIBRATION_BLINK_DELAY_LONG;
led_set(BLINK_BOTH);
}

void calibration_loop()
{
//Process samples
vector3_t accel_mean,gyro_mean,accel_var;
int batch_ready=calibration_process_samples(&accel_mean,&gyro_mean,&accel_var);
	
		if(batch_ready&&(calibration_state&CALIBRATION_BLINK_DELAY_MASK))calibration_state-=0x100;
		else if(batch_ready)
		{
		int is_static=accel_var.x<0.0002&&accel_var.y<0.0002&&accel_var.z<0.0002;

		if(calibration_state==CALIBRATION_ACCEL_WAITING&&is_static)
		{
		//Determine axis
		int axis=calibration_get_axis(accel_mean,9.81);
			if(completed_axes&(1<<axis))led_set(SOLID_GREEN);
			else if(axis!=AXIS_NONE)calibration_accel_start_recording(axis);
		}
		else if(calibration_state==CALIBRATION_ACCEL_WAITING)led_set(SOLID_YELLOW);
		else if(calibration_state==CALIBRATION_ACCEL_RECORDING&&!is_static)
		{
		calibration_accel_terminate_recording();
		}
		else if(calibration_state==CALIBRATION_ACCEL_RECORDING)
		{
		calibration_accel_record_batch(accel_mean,gyro_mean,accel_var);

		//Estimate variance of acceleration measurements as average of the sample variances computed for each batch
		vector3_t accel_var=vector3_scale(accel_var_total,1.0/accel_batches_processed);
		//Estimate variance of mean estimate as estimated measurement variance/number of samples
		accel_var=vector3_scale(accel_var,1.0/(accel_batches_processed*CALIBRATION_BUFFER_SIZE));
		accel_mean=vector3_scale(accel_mean_total,1.0/accel_batches_processed);
			//Terminate data gathering if standard deviation is estimated to be less than 0.0002 for all components
			if(sqrt(accel_var.x)<0.0002&&sqrt(accel_var.y)<0.0002&&sqrt(accel_var.z)<0.0002)
			{
			calibration_accel_finish_recording();
				if(completed_axes==0x3F)calibration_state=CALIBRATION_ACCEL_COMPLETED;
			}
		}
		else if(calibration_state==CALIBRATION_ACCEL_COMPLETED)calibration_complete_accel();
		else if((calibration_state==CALIBRATION_GYRO_READY||calibration_state==CALIBRATION_GYRO_RECORDING)&&!is_static)
		{
		Serial.println("DEBUG: Gyro unsteady");
		led_set(SOLID_YELLOW);
		//Compute gyro integral
			for(int i=0;i<CALIBRATION_BUFFER_SIZE;i++)
			{
			vector3_t gyro=vector3_sub(calibration_buffer[i].gyro,gyro_bias_new);
				if(calibration_buffer[i].timestamp&SAMPLE_HAS_MAG&&num_mag_samples<MAX_MAG_SAMPLES)//TODO fail if max mag samples is reached
				{
				mag_samples[num_mag_samples]=calibration_buffer[i].mag;
				mag_sample_rotations[num_mag_samples]=fmax(gyro_integral_total.x,fmax(gyro_integral_total.y,gyro_integral_total.z));
				num_mag_samples++;
				}
			gyro_integral_total=vector3_add(gyro_integral_total,vector3_scale(gyro,0.01));
			}
		gyro_batches_processed++;
		}
		else if(calibration_state==CALIBRATION_GYRO_READY)
		{
		Serial.println("DEBUG: Gyro ready");
		int axis=calibration_get_axis(accel_mean,9.81);
			if(axis!=AXIS_NONE&&axis==calibration_axis&&gyro_batches_processed>1)
			{
			//Check if rotation axis is consistent with acceleration axis
			int gyro_axis=calibration_get_axis(gyro_integral_total,2*M_PI);
				if(gyro_axis!=AXIS_NONE&&gyro_axis==axis&&!(completed_axes&(1<<gyro_axis)))
				{
				calibration_gyro_start_recording(gyro_axis);
				calibration_gyro_record_value();
				}else calibration_gyro_ready(axis);
			}else calibration_gyro_ready(axis);
		}
		else if(calibration_state==CALIBRATION_GYRO_RECORDING&&gyro_batches_processed>1)
		{
		Serial.println("DEBUG: Gyro recording");
		//Check acceleration axis has not changed
		int axis=calibration_get_axis(accel_mean,9.81);
			if(calibration_axis==axis)
			{
			int gyro_axis=calibration_get_axis(gyro_integral_total,2*M_PI);
				if(gyro_axis==calibration_axis)
				{
				calibration_gyro_record_value();

				vector3_t gyro_mean=vector3(0,0,0);
				vector3_t mag_mean=vector3(0,0,0);
					for(int i=0;i<gyro_rotations_recorded;i++)
					{
					gyro_mean=vector3_add(gyro_mean,gyro_rotations[i]);
					mag_mean=vector3_add(mag_mean,gyro_rotation_mag_values[i]);
					}
				gyro_mean=vector3_scale(gyro_mean,1.0/gyro_rotations_recorded);
				mag_mean=vector3_scale(mag_mean,1.0/gyro_rotations_recorded);

				vector3_t gyro_var=vector3(0,0,0);
				vector3_t mag_var=vector3(0,0,0);
					for(int i=0;i<gyro_rotations_recorded;i++)
					{
					vector3_t d=vector3_sub(gyro_rotations[i],gyro_mean);
					gyro_var=vector3_add(gyro_var,vector3(d.x*d.x,d.y*d.y,d.z*d.z));
					d=vector3_sub(gyro_rotation_mag_values[i],mag_mean);
					mag_var=vector3_add(mag_var,vector3(d.x*d.x,d.y*d.y,d.z*d.z));
					}
				gyro_var=vector3_scale(gyro_var,1.0/(gyro_rotations_recorded-1));
				gyro_var=vector3_scale(gyro_var,1.0/(gyro_rotations_recorded));
				mag_var=vector3_scale(mag_var,1.0/(gyro_rotations_recorded-1));
				mag_var=vector3_scale(mag_var,1.0/(gyro_rotations_recorded));

					if(gyro_rotations_recorded>4&&sqrt(gyro_var.x)<0.005&&sqrt(gyro_var.y)<0.005&&sqrt(gyro_var.z)<0.005&&sqrt(mag_var.x)<0.01&&sqrt(mag_var.y)<0.01&&sqrt(mag_var.z)<0.01)
					{
					calibration_gyro_finish_recording(gyro_mean,mag_mean);
						if(completed_axes==0x3F)calibration_complete_gyro();
					}
				}
				else calibration_gyro_reject();
			}
			else calibration_gyro_terminate_recording();
		}
		else if(calibration_state==CALIBRATION_GYRO_RECORDING)led_set(SOLID_BOTH);
		else if(calibration_state!=CALIBRATION_COMPLETED)Serial.println("DEBUG: Unknown calibration state");
	}
}

int calibrate()
{
//Reset IMU calibration (so we get raw measurements)
imu_reset_calibration();

//Initialize state
calibration_init();
//Run main loop
	while(calibration_state!=CALIBRATION_COMPLETED)calibration_loop();

//Let the LED blink for a few seconds so the user knows the calibration has finished
delay(3000);

//Write results
io_file file;
	if(file.open("calibration.txt", O_WRITE_ONLY | O_CREATE)!=0)
	{
	Serial.println("Failed to open calibration.txt");
	led_set(BLINK_YELLOW);
	delay(1000);
	return 1;
	}

file.writef("accel_bias=[%f,%f,%f]\n",accel_bias_new.x,accel_bias_new.y,accel_bias_new.z);
file.writef("accel_matrix=[%f,%f,%f,%f,%f,%f,%f,%f,%f]\n",accel_matrix_new[0],accel_matrix_new[1],accel_matrix_new[2],accel_matrix_new[3],accel_matrix_new[4],accel_matrix_new[5],accel_matrix_new[6],accel_matrix_new[7],accel_matrix_new[8]);
file.writef("gyro_bias=[%f,%f,%f]\n",gyro_bias_new.x,gyro_bias_new.y,gyro_bias_new.z);
file.writef("gyro_matrix=[%f,%f,%f,%f,%f,%f,%f,%f,%f]\n",gyro_matrix_new[0],gyro_matrix_new[1],gyro_matrix_new[2],gyro_matrix_new[3],gyro_matrix_new[4],gyro_matrix_new[5],gyro_matrix_new[6],gyro_matrix_new[7],gyro_matrix_new[8]);
file.writef("mag_bias=[%f,%f,%f]\n",mag_bias_new.x,mag_bias_new.y,mag_bias_new.z);
file.writef("mag_matrix=[%f,%f,%f,%f,%f,%f,%f,%f,%f]\n",mag_matrix_new[0],mag_matrix_new[1],mag_matrix_new[2],mag_matrix_new[3],mag_matrix_new[4],mag_matrix_new[5],mag_matrix_new[6],mag_matrix_new[7],mag_matrix_new[8]);

file.close();

//Update IMU calibration
imu_set_calibration(accel_bias_new,accel_matrix_new,gyro_bias_new,gyro_matrix_new,mag_bias_new,mag_matrix_new);

return 0;
}
