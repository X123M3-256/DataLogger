#include <Arduino.h>
#include "led.h"
#include "gps.h"
#include "log.h"
#include "io.h"

int log_state=NO_FIX;


bool log_start()
{

	if(log_state!=READY)
	{
	const char* reasons[]={"no GPS fix","","log is already waiting to start","log is already running"};
	log_message(LOG_ERROR|LOG_LOG,"Cannot start new log - %s",reasons[log_state]);
	return false;
	}
//int timestamp=3600000*ubx.nav_pvt.hour+60000*ubx.nav_pvt.min+1000*ubx.nav_pvt.sec+(int)round(ubx.nav_pvt.nanoseconds*1e-6);
/*
char date[256];
sprintf(date,"%04d-%02d-%02d",ubx.nav_pvt.year,ubx.nav_pvt.month,ubx.nav_pvt.day);
char time[256];
sprintf(time,"%d:%d:%d\n",ubx.nav_pvt.hour,ubx.nav_pvt.min,ubx.nav_pvt.sec);

Serial.println(directory);
Serial.println(file);
*/

log_state=INIT;
return 0;
}

void log_stop()
{
log_state=READY;
led_set(SOLID_GREEN);
}

bool log_is_running()
{
return !(log_state==NO_FIX||log_state==READY);
}


bool log_update()
{
bool data_processed=false;

//TODO don't expose raw UBX packets outside the GPS code
ubx_t ubx;
	while(ubx_receive(&ubx))
	{
	data_processed=true;
		//Ignore any packet that isn't UBX-NAV-PVT
		if(ubx.clss!=1&&ubx.id!=7)continue;

		if(ubx.nav_pvt.fix_type==0)
		{
		//TODO - terminating the log immediately upon losing the fix is not ideal. Consider pausing the log
		//until the fix is reacquired, and/or continuing using the IMU only for a few seconds
			if(log_state==ACTIVE)log_stop();
		log_state=NO_FIX;
		led_set(SOLID_YELLOW);
		}

		//Only process 3D fixes- 2D is pretty much useless here
		if(ubx.nav_pvt.fix_type!=3)continue;
	
		switch(log_state)
		{	
		case NO_FIX:
			log_state=READY;
			led_set(SOLID_GREEN);
		break;
		//TODO the init state might not actually be necessary - if the state is READY, then at least one 3D fix
		//has already been received. We could save the last GPS packet and start the log from there
		case INIT:
			log_state=ACTIVE;
			led_set(BLINK_GREEN);
		break;
		case ACTIVE:
		//process_gps_packet(timestamp,&ubx)
		break;
		}
	}
return data_processed;
}


/*
int initialized=0;
state_t state;
double covariance[15*15]={0};



typedef struct
{
double latitude;
double longitude;
double elevation;
double velocity_n;
double velocity_e;
double velocity_d;
}measurement_t;
void initialize_filter(int timestamp,ubx_t* ubx)
{
state.position.x=ubx->data.nav_pvt.latitude*1e-7;
state.position.y=ubx->data.nav_pvt.longitude*1e-7;
state.position.z=ubx->data.nav_pvt.height_msl*1e-3;
state.velocity.x=ubx->data.nav_pvt.velocity_n*1e-3;
state.velocity.y=ubx->data.nav_pvt.velocity_e*1e-3;
state.velocity.z=ubx->data.nav_pvt.velocity_d*1e-3;

covariance[0+9*0]=POSITION_NOISE;
covariance[1+9*1]=POSITION_NOISE;
covariance[2+9*2]=POSITION_NOISE;
covariance[3+9*3]=VELOCITY_NOISE;
covariance[4+9*4]=VELOCITY_NOISE;
covariance[5+9*5]=VELOCITY_NOISE;

//TODO initialize orientation
Serial.println(timestamp);
initialized=1;
}

void process_gps_packet(int timestamp,ubx_t* ubx)
{
	if(timestamp%10!=0)
	{
	Serial.println("Encountered misaligned GPS packet");
	}
//Get available IMU samples

//Get available magnetometer samples

int sample_timestamp=0;
	do 
	{
	//Get IMU sample to process etc etc
	//double input[6]={-accel.x,accel.y,accel.z,gyro.x,-gyro.y,gyro.z};
	//kalman_predict(&state,covariance,input);


	//char str[512];
	//sprintf(str,"Processed sample %d\n",current_timestamp-timestamp_at_start);
	//Serial.write(str);
	
	//current_timestamp++;
	}while(sample_timestamp<=timestamp);

//double measurement[9]={state.position.x,state.position.y,state.position.z,0,0,0,-mag_sample.y,mag_sample.x,mag_sample.z};
//kalman_update(&state,covariance,measurement);

//char buf[256];
//sprintf(buf,"%f %f %f\n",-mag_sample.y,mag_sample.x,mag_sample.z);
//Serial.write(buf);
//sprintf(buf,"Heading %.0f Inclination %.0f Magnitude %.2f\n",180*atan2(-mag_sample.y,mag_sample.x)/M_PI,180*atan2(mag_sample.z,sqrt(mag_sample.x*mag_sample.x+mag_sample.y*mag_sample.y))/M_PI,vector3_magnitude(mag_sample));
//Serial.write(buf);
//sprintf(buf,"%f %f %f %f %f %f %f %f\n",0.01*(timestamp-timestamp_at_start),state.position.x,state.position.y,state.position.z,state.orientation.w,state.orientation.i,state.orientation.j,state.orientation.k);
//Serial.write(buf);

return;
/*
//Record data
double latitude=ubx->data.nav_pvt.latitude*1e-7;
double longitude=ubx->data.nav_pvt.longitude*1e-7;
double elevation=ubx->data.nav_pvt.height_msl*1e-3;
double velocity_n=ubx->data.nav_pvt.velocity_n*1e-3;
double velocity_e=ubx->data.nav_pvt.velocity_e*1e-3;
double velocity_d=ubx->data.nav_pvt.velocity_d*1e-3;

char str[512];
sprintf(str,"Available samples %d\n",available_samples);
Serial.write(str);
sprintf(str,"Timestamp %d (since PPS %d)\n",timestamp-timestamp_at_start,centiseconds_since_pps);
Serial.write(str);
sprintf(str,"Lat %f Lon %f Altitude %f\n",latitude,longitude,elevation);
Serial.write(str);
sprintf(str,"Velocity N %f E %f D %f\n",velocity_n,velocity_e,velocity_d);
Serial.write(str);
*/	
//}
