#include "i2c.h"
#include "led.h"
#include "io.h"
#include "imu.h"
#include "gps.h"
#include "log.h"
#include "calibration.h"
#include "console.h"
#include "timer.h"


//TODO make this separate file
IntervalTimer button_timer;
uint8_t button_state=0;
int button_press_start=0;
int button_press_completed=0;

void isr_button()
{
button_state=button_state<<1;
button_state|=digitalRead(33);

	if(button_state==0x7F)
	{
	button_press_completed=0;
	button_press_start=millis();
	}
	if(button_state==0x80)
	{
	button_press_start=0;
	button_press_completed=1;
	}
}

int get_button_press_duration()
{
	if(button_press_start==0)return 0;
return millis()-button_press_start;
}

int get_completed_button_press()
{
	if(button_press_completed)
	{
	button_press_completed=0;
	return 1;
	}
return 0;
}

void button_init()
{
pinMode(33, INPUT);
button_timer.begin(isr_button,10000);
}



void init()
{
//Initialize IO
	while(!io_init())
	{
	led_set(BLINK_YELLOW);
	delay(500);
	}


	while(!usb_serial->open(9600,O_READ_WRITE))
	{
	log_message(LOG_INIT|LOG_INFO,"Waiting for serial");
	delay(500);
	}

led_init();
log_message(LOG_INIT|LOG_INFO,"Initializing button");
button_init();

log_message(LOG_INIT|LOG_INFO,"Initializing IO");


//Initialize GPS
int gps_fail_count=0;
log_message(LOG_INIT|LOG_INFO,"Initializing GPS");
	while(!gps_init(gps_fail_count>1))//Debug feature - if it fails repeatedly it might be because the GPS is already configured TODO make it so configuration routine works anyway - this is a hack
	{
	led_set(BLINK_YELLOW);
	delay(500);
	log_message(LOG_INIT|LOG_INFO,"Retrying");
	gps_fail_count++;
	}


//Initialize IMU
log_message(LOG_INIT|LOG_INFO,"Initializing IMU");
i2c_init();
	while(!imu_init())
	{
	led_set(BLINK_YELLOW);
	delay(500);
	log_message(LOG_INIT|LOG_INFO,"Retrying");
	}

log_message(LOG_INIT|LOG_INFO,"Loading calibration file");
	if(!imu_load_calibration("calibration.txt"))
	{
	led_set(BLINK_YELLOW);
	delay(500);
	//Run calibration routine
	log_message(LOG_INIT|LOG_INFO,"Entering calibration mode");
	calibrate();
	}

timer_start();
led_set(SOLID_YELLOW);
}

extern "C" int main(void)
{
init();
	while (1)
	{
	update_timer_start(UPDATE_TOTAL);

	//Handle button press
		if(get_completed_button_press())
		{
			if(!log_is_running())log_start();
			else log_stop();
		}
		else if(get_button_press_duration()>2000&&!log_is_running())calibrate();

	//Log update
	update_timer_start(UPDATE_LOG);
	bool log=log_update();
		if(log)update_timer_stop(UPDATE_LOG);

	//Console update
	update_timer_start(UPDATE_CONSOLE);
	int con=console_update();
		if(con)update_timer_stop(UPDATE_CONSOLE);
//		if(con||log)update_timer_stop(UPDATE_TOTAL);
	}
}

