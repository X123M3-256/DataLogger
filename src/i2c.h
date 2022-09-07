#include "Wire.h"

class i2c
{
public:
	i2c(TwoWire* i);
	bool read(uint8_t addr,uint8_t reg, uint8_t* bytes,uint8_t num_bytes);
	bool write(uint8_t addr,uint8_t reg, uint8_t* bytes,uint8_t num_bytes);
private:
	TwoWire* interface;
};

extern i2c i2c_bus[2];

void i2c_init();
