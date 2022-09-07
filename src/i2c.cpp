#include "i2c.h"

i2c::i2c(TwoWire* i)
	{
	interface=i;
	}

bool i2c::write(uint8_t addr,uint8_t reg, uint8_t* bytes,uint8_t num_bytes)
	{
  	interface->beginTransmission(addr);
	interface->write(reg);
		for(int i=0;i<num_bytes;i++)interface->write(bytes[i]);
		if(interface->endTransmission()!=0)return 1;
	return 0;
	}

bool i2c::read(uint8_t addr,uint8_t reg, uint8_t* bytes,uint8_t num_bytes)
	{
	interface->beginTransmission(addr);
	interface->write(reg);
		if(interface->endTransmission(false)!=0)return 1;
		if(interface->requestFrom((uint8_t)addr,num_bytes)!=num_bytes)return 2;
		for(int i=0;i<num_bytes;i++)bytes[i]=interface->read();
	return 0;
	}

i2c i2c_bus[2]={i2c(&Wire),i2c{&Wire1}};

void i2c_init()
{
Wire.begin();
Wire1.begin();
}
