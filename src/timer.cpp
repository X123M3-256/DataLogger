#include <Arduino.h>
#include "timer.h"
#include "imu.h"

enum
{
TIMER_LOCK=0x1,
TIMER_TIME_SYNC=0x4,
TIMER_READ_COMPLETE=0x8,

TIMER_TIME_ERROR=0x20,
TIMER_RUNNING=0x40
};

IntervalTimer timer;
//Timestamp for last recieved GPS packet
volatile uint64_t gps_timestamp=0;
//Timestamp for last PPS signal
volatile uint64_t pps_timestamp=0;
//Current timestamp
volatile uint64_t timestamp=0;
//Flags for interrupt service routines
volatile uint8_t isr_flags=0;




bool timer_lock()
{
	if(!(isr_flags&TIMER_LOCK))
	{
	isr_flags|=TIMER_LOCK;
	return true;
	}
return false;
}

void timer_unlock()
{
isr_flags&=~TIMER_LOCK;
}

void timer_wait_on_isr(int millis_required)
{
//If the timer interrupt isn't running there's no need to wait
	if(!(isr_flags&TIMER_RUNNING))return;

//Determine number of milliseconds remaining until the next IMU interrupt
uint64_t cur_timestamp=timer_now();
uint64_t millis_remaining=10-(cur_timestamp%10);

//If we have enough time, we can proceed immediately
	if(millis_remaining>millis_required)return;

//Otherwise, wait for next IMU interrupt to take place
isr_flags&=~TIMER_READ_COMPLETE;
	while(!(isr_flags&TIMER_READ_COMPLETE));
}

void timer_wait_for_lock(int millis_required)
{
	if(millis_required>0)timer_wait_on_isr(millis_required);
	while(!timer_lock());
return;
}










void isr_timer()
{
//Increment timestamp by 10ms
timestamp+=10;

/*TODO consider using interrupt pin to sample the IMU at the native 104Hz. Then we would know what time
an IMU sample was actually collected (even if this would be rounded for computations, it may be useful 
to know if a delayed sample should be treated as a missed sample).*/

//Sample IMU
	if(timer_lock())
	{
	imu_read(timestamp);
	timer_unlock();
	}
isr_flags|=TIMER_READ_COMPLETE;

//If it has been more than 10 seconds since the last PPS, assume the interrupt timer may not be synchronized 
//to the GPS time (this is a fairly conservative number, but we want to make sure that if there are timestamp 
//errors detected then they do in fact signal a bug)
	if(timestamp>pps_timestamp+10000)
	{
	//Serial.println("Sync lost");
	//Serial.print((int)pps_timestamp);
	//Serial.println();
	isr_flags&=~TIMER_TIME_SYNC;
	}
}

void isr_pps()
{
/*There will be some time delay between the PPS signal and the corresponding GPS packet being parsed. If 
we assume that this lag falls in the range 0<=lag<1000ms, then the current time will be the last recieved GPS
timestamp rounded up to the next second.*/
uint64_t timestamp_from_packets=1000*((gps_timestamp+999)/1000);


/*Teensy clock will drift slightly relative to GPS clock -  by roughly approx 20us/s or 1ms
every 50s. Therefore, we restart the IMU timer when a PPS signal is recieved to keep them synchronized.*/
timer.begin(isr_timer,10000);

/*The clock drift could be positive or negative, so the timer interrupt corresponding to the current timestamp 
may run before or after the PPS interrupt. If we determine that the timer interrupt for the current time has not 
yet run, we invoke it here*/
	if(timestamp%1000==990)isr_timer();

/*We have no absolute time reference until the GPS has acquired a fix, so the time will start counting from zero.
If this is the first PPS interrupt since the fix was obtained, then the IMU timestamp should be re-initialized with
the timestamp obtained from the GPS data*/ 
	if(!(isr_flags&TIMER_TIME_SYNC)&&timestamp_from_packets>0)
	{
	timestamp=timestamp_from_packets;
	isr_flags|=TIMER_TIME_SYNC;
	}

//At this point, the current timestamp should match the value obtained from the GPS data. If it doesn't, then we flag 
//an error. I believe that this should never happen, so if the flag is ever set it is expected to signal a bug.
//TODO check for and log these error once logging is implemented
	if(timestamp_from_packets!=0&&timestamp!=timestamp_from_packets)
	{
	timestamp=timestamp_from_packets;//Re-synchronize
	isr_flags|=TIMER_TIME_ERROR;
	}

//Update PPS timestamp
pps_timestamp=timestamp;
}

//Return the current timestamp
uint64_t timer_now()
{
noInterrupts();
uint64_t t=timestamp;
interrupts();
return t;
}

//True if the timer is synchronized to the GPS time
bool timer_is_synced()
{

}

void timer_start()
{
//Set PPS pin
pinMode(2, INPUT);
//Attach interrupts
timer.begin(isr_timer,10000);
attachInterrupt(digitalPinToInterrupt(2),isr_pps,RISING);
isr_flags|=TIMER_RUNNING;
}

void timer_set_gps_time(uint64_t timestamp)
{
noInterrupts();
gps_timestamp=timestamp;
interrupts();
}
