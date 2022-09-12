#include <Arduino.h>
#include "led.h"
#include "timer.h"
#include "imu.h"
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
log_state=INIT;
return true;
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

uint64_t log_timestamp=0;

void log_init(gps_t* gps)
{
log_timestamp=gps->timestamp;
}


//TODO decide what to do about leap seconds
uint64_t timestamp_from_utc(uint16_t year,uint8_t month,uint8_t day,uint8_t hour,uint8_t min,uint8_t sec,int32_t nanoseconds)
{
//Calculate days since 1970
static const int cumdays[12]={0,31,59,90,120,151,181,212,243,273,304,334};
uint64_t result=(year-1970)*365+cumdays[month-1];
//Account for leap years
result+=(year-1968)/4;
result-=(year-1900)/100;
result+=(year-1600)/400;
	if((year%4)==0&&((year%100)!=0||(year%400)==0)&&(month)<2)result--;
result+=day-1;
//Calculate hours
result=result*24+hour;
//Calculate minutes
result=result*60+min;
//Calculate seconds
result=result*60+sec;
//Calculate nanoseconds
result=result*1000000000+nanoseconds;
//Round to nearest millisecond
result=(result+500000)/1000000;
return result;
}



void log_process_gps_packet(gps_t* gps)
{
usb_serial->writef("Log timestamp is %lld\r\n",log_timestamp);

//GPS timestamps should always be aligned to the tenth of a second
//If we encounter a packet that isn't, we discard it (TODO consider if we should round it instead)
	if(gps->timestamp%100!=0)
	{
	log_message(LOG_LOG|LOG_ERROR,"Encountered misaligned GPS packet (timestamp %d)",gps->timestamp);
	return;
	}

//Calculate the time since the last GPS packet. This should normally be 100ms, more indicates a 
//dropped GPS packet or a temporary loss of fix
uint64_t gps_delta_t=gps->timestamp-log_timestamp;

	//There should never be two GPS packets with the same timestamp, but if any are encountered
	//we discard the duplicate
	if(gps_delta_t==0)
	{
	log_message(LOG_LOG|LOG_ERROR,"Encountered duplicate GPS packet (timestamp %d)",gps->timestamp);
	return;
	}

usb_serial->writef("GPS delta T: %lldms\r\n",gps_delta_t);


//Kalman prediction step

	do
	{
	imu_t imu;
		if(!imu_get(&imu))
		{
		//TODO handle this case
		}

	uint64_t imu_timestamp=imu.timestamp&SAMPLE_TIMESTAMP_MASK;

	//IMU timestamp should always be divisible by 10, as we sample every 10ms.
		if(imu_timestamp%10!=0)
		{
		log_message(LOG_LOG|LOG_ERROR,"Encountered misaligned IMU packet (timestamp %lld)",imu_timestamp);
		continue;
		}

	//IMU timestamp should be greater than the current filter timestamp, as all data
	//up to the current timestamp should already have been processed. This check should
	//never fail, but if does, we discard the IMU packet
		if(imu_timestamp<=log_timestamp)
		{
		log_message(LOG_LOG|LOG_INFO,"Encountered outdated IMU reading (Log current timestamp %lld IMU timestamp %lld)",log_timestamp,imu_timestamp);
		continue;
		}

	//IMU timestamp should now be greater than the current filter timestamp and divisible
	//by 10ms. We now check if equal to 
	uint64_t delta_t=imu_timestamp-log_timestamp;

		//Check that delta T is not zero, which would indicate multiple IMU readings
		//in the same 10ms interval. This shouldn't occur, but if it does we discard
		//the duplicate packet
		if(delta_t==0)
		{
		log_message(LOG_LOG|LOG_INFO,"Encountered duplicate IMU packet");
		continue;
		}

		while(delta_t>10)
		{
		//Duplicate packets
		}

	usb_serial->writef("IMU delta T: %lldms\r\n",delta_t);
	//The IMU updates at 104Hz, so when we poll the sensor, the reading could be up to 9.6ms old.
	//Therefore, the filter timestamp after the correction step is performed is equal to the timestamp
	//of the IMU packet, and the actual IMU measurement may fall anywhere within the current time step

//	double input[6]={-imu.accel.x,imu.accel.y,imu.accel.z,imu.gyro.x,-imu.gyro.y,imu.gyro.z};
//	kalman_predict(&state,covariance,input);
	log_timestamp=imu_timestamp;		
	}while(log_timestamp<gps->timestamp);

usb_serial->writef("IMU packets left in buffer: %d\r\n",imu_available());

usb_serial->flush();



//double measurement[9]={state.position.x,state.position.y,state.position.z,0,0,0,-mag_sample.y,mag_sample.x,mag_sample.z};
//kalman_update(&state,covariance,measurement);

//char buf[256];
//sprintf(buf,"%f %f %f\n",-mag_sample.y,mag_sample.x,mag_sample.z);
//Serial.write(buf);
//sprintf(buf,"Heading %.0f Inclination %.0f Magnitude %.2f\n",180*atan2(-mag_sample.y,mag_sample.x)/M_PI,180*atan2(mag_sample.z,sqrt(mag_sample.x*mag_sample.x+mag_sample.y*mag_sample.y))/M_PI,vector3_magnitude(mag_sample));
//Serial.write(buf);
//sprintf(buf,"%f %f %f %f %f %f %f %f\n",0.01*(timestamp-timestamp_at_start),state.position.x,state.position.y,state.position.z,state.orientation.w,state.orientation.i,state.orientation.j,state.orientation.k);
//Serial.write(buf);



}

bool log_update()
{
bool data_processed=false;

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

		//Read packet contents
		gps_t gps;
		gps.timestamp=timestamp_from_utc(ubx.nav_pvt.year,ubx.nav_pvt.month,ubx.nav_pvt.day,ubx.nav_pvt.hour,ubx.nav_pvt.min,ubx.nav_pvt.sec,ubx.nav_pvt.nanoseconds);
		timer_set_gps_time(gps.timestamp);
		gps.lon=ubx.nav_pvt.longitude*1e-7;
		gps.lat=ubx.nav_pvt.latitude*1e-7;
		gps.alt=ubx.nav_pvt.height_msl*1e-3;
		gps.vel_n=ubx.nav_pvt.velocity_n*1e-3;
		gps.vel_e=ubx.nav_pvt.velocity_e*1e-3;
		gps.vel_d=ubx.nav_pvt.velocity_d*1e-3;
		gps.horz_acc=ubx.nav_pvt.horizontal_acc*1e-3;
		gps.vert_acc=ubx.nav_pvt.vertical_acc*1e-3;
		gps.vel_acc=ubx.nav_pvt.speed_acc*1e-3;
		gps.dop=ubx.nav_pvt.position_dop*1e-2;
		gps.num_sv=ubx.nav_pvt.num_sv;
	
		switch(log_state)
		{	
		case NO_FIX:
			log_init(&gps);
			log_state=READY;
			led_set(SOLID_GREEN);
		break;
		//TODO the init state might not actually be necessary - if the state is READY, then at least one 3D fix
		//has already been received. We could save the last GPS packet and start the log from there
		case INIT:
			log_state=ACTIVE;
			led_set(BLINK_GREEN);
		case ACTIVE:
		case READY:
		log_process_gps_packet(&gps);
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
