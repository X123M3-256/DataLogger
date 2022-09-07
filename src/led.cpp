#include <Arduino.h>
#include "led.h"

IntervalTimer led_timer;
uint8_t led_state;

void isr_led()
{
static int blink_state=1;
	switch(led_state)
	{
	case OFF:
	digitalWriteFast(29,0);
	digitalWriteFast(32,0);
	break;
	case SOLID_YELLOW:
	digitalWriteFast(29,1);
	digitalWriteFast(32,0);
	break;
	case BLINK_YELLOW:
	digitalWriteFast(29,blink_state&1);
	digitalWriteFast(32,0);
	break;
	case SOLID_GREEN:
	digitalWriteFast(32,1);
	digitalWriteFast(29,0);
	break;
	case BLINK_GREEN:
	digitalWriteFast(32,blink_state&1);
	digitalWriteFast(29,0);
	break;
	case SOLID_BOTH:
	digitalWriteFast(29,1);
	digitalWriteFast(32,1);
	break;
	case SOLID_YELLOW_BLINK_GREEN:
	digitalWriteFast(29,1);
	digitalWriteFast(32,blink_state&1);
	break;
	case BLINK_BOTH:
	digitalWriteFast(32,blink_state&1);
	digitalWriteFast(29,blink_state&1);
	break;
	}
blink_state++;
}

void led_init()
{
//Initialize pins
pinMode(32, OUTPUT);
pinMode(29, OUTPUT);
//Start led blink timer
led_timer.begin(isr_led,500000);
led_state=SOLID_YELLOW;
}


void led_set(uint8_t state)
{
led_state=state;
}
