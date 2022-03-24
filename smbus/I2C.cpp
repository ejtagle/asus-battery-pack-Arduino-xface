
/**
 * @file I2C.cpp
 *
 * Base class for devices attached via the I2C bus.
 *
 * @todo Bus frequency changes; currently we do nothing with the value
 *       that is supplied.  Should we just depend on the bus knowing?
 */

#include "I2C.hpp"

I2C::I2C(const uint16_t address, const uint32_t frequency) 
: _addr(address), _bus(SDA_PIN, SCL_PIN, true)
{
}

I2C::~I2C()
{
}

int
I2C::init()
{
	_bus.i2c_init();
	return 0;
}

int
I2C::transfer(const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len)
{
    unsigned x;
    _bus.i2c_start((_addr<<1) + I2C_WRITE);
	for (x=0; x<send_len; ++x) {
		_bus.i2c_write(send[x]);
	}
	if (recv_len) {
		_bus.i2c_rep_start((_addr<<1) + I2C_READ); 
		for (x=0; x<recv_len-1; ++x) {                           //-1 because x=num_bytes-1 if x<y; last byte needs to be "nack"'d, x<y-1
			recv[x] = _bus.i2c_read(false);
		}
		recv[x] = _bus.i2c_read(true);		//this will nack the last byte and store it in x's num_bytes-1 address.
    }
	_bus.i2c_stop();
	return 0;
}
