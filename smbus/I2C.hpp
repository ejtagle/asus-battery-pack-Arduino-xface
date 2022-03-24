#pragma once

/**
 * @file I2C.hpp
 *
 * Base class for devices connected via I2C.
 */

#define SCL_PIN 54
#define SDA_PIN 55
 
#include "SlowSoftI2CMaster.hpp"

/**
 * Abstract class for character device on I2C
 */
class I2C
{

public:

	// no copy, assignment, move, move assignment
	I2C(const I2C &) = delete;
	I2C &operator=(const I2C &) = delete;
	I2C(I2C &&) = delete;
	I2C &operator=(I2C &&) = delete;

	/**
	 * @ Constructor
	 *
	 * @param address	I2C bus address, or zero if set_address will be used
	 * @param frequency	I2C bus frequency for the device (currently not used)
	 */
	I2C(const uint16_t address, const uint32_t frequency);
	virtual ~I2C();

	int	init();
	uint8_t get_device_address() const { return _addr; }

	/**
	 * Perform an I2C transaction to the device.
	 *
	 * At least one of send_len and recv_len must be non-zero.
	 *
	 * @param send		Pointer to bytes to send.
	 * @param send_len	Number of bytes to send.
	 * @param recv		Pointer to buffer for bytes received.
	 * @param recv_len	Number of bytes to receive.
	 * @return		OK if the transfer was successful, -errno
	 *			otherwise.
	 */
	int		transfer(const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len);

private:
	uint8_t	_addr;
	SlowSoftI2CMaster _bus;
};
