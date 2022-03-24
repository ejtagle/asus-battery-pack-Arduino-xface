/**
 * @file SMBus.cpp
 * SMBus v2.0 protocol implementation.
 *
 * @author Jacob Dahl <dahl.jakejacob@gmail.com>
 * @author Bazooka Joe <BazookaJoe1900@gmail.com>
 *
 * TODO
 *  - Add remaining SMBus protocol messages as needed
 */

#include "SMBus.hpp"
#include <errno.h>

SMBus::SMBus(uint16_t address) :
	I2C(address, 100000)
{
}

SMBus::~SMBus()
{
}

int SMBus::read_word(const uint8_t cmd_code, uint16_t &data)
{
	uint8_t buf[6];
	// 2 data bytes + pec byte
	int result = transfer(&cmd_code, 1, buf + 3, 3);

	if (result == 0) {
		data = buf[3] | ((uint16_t)buf[4] << 8);
		// Check PEC.
		uint8_t addr = get_device_address() << 1;
		buf[0] = addr | 0x00;
		buf[1] = cmd_code;
		buf[2] = addr | 0x01;

		uint8_t pec = get_pec(buf, sizeof(buf) - 1);

		if (pec != buf[sizeof(buf) - 1]) {
			result = -EINVAL;
		}
	}

	return result;
}

int SMBus::write_word(const uint8_t cmd_code, uint16_t data)
{
	// 2 data bytes + pec byte
	uint8_t buf[5];
	buf[0] = (get_device_address() << 1) | 0x10;
	buf[1] = cmd_code;
	buf[2] = data & 0xff;
	buf[3] = (data >> 8) & 0xff;

	buf[4] = get_pec(buf, 4);

	int result = transfer(&buf[1], 4, nullptr, 0);
	return result;
}

int SMBus::block_read(const uint8_t cmd_code, void *data, const uint8_t length, const bool use_pec)
{
	uint8_t byte_count = 0;
	// addr(wr), cmd_code, addr(r), byte_count, data (MAX_BLOCK_LEN bytes max), pec
	uint8_t rx_data[MAX_BLOCK_LEN + 5];

	if (length > MAX_BLOCK_LEN) {
		return -EINVAL;
	}

	int result = transfer(&cmd_code, 1, (uint8_t *)&rx_data[3], length + 2);

	if (result != 0) {
		return result;
	}

	uint8_t device_address = get_device_address();
	rx_data[0] = (device_address << 1) | 0x00;
	rx_data[1] = cmd_code;
	rx_data[2] = (device_address << 1) | 0x01;
	byte_count = rx_data[3];
	if (byte_count > MAX_BLOCK_LEN) 
		byte_count = MAX_BLOCK_LEN;

	// ensure data is not longer than given buffer
	memcpy(data, &rx_data[4], byte_count < length ? byte_count: length);

	if (use_pec) {
		uint8_t pec = get_pec(rx_data, byte_count + 4);

		if (pec != rx_data[byte_count + 4]) {
			result = -EIO;
		}
	}

	return result;
}

int SMBus::block_write(const uint8_t cmd_code, const void *data, uint8_t byte_count, const bool use_pec)
{
	// cmd code[1], byte count[1], data[byte_count] (MAX_BLOCK_LEN max), pec[1] (optional)
	uint8_t buf[MAX_BLOCK_LEN + 2];

	if (byte_count > MAX_BLOCK_LEN) {
		return -EINVAL;
	}

	buf[0] = cmd_code;
	buf[1] = (uint8_t)byte_count;
	memcpy(&buf[2], data, byte_count);

	if (use_pec) {
		uint8_t pec = get_pec(buf, byte_count + 2);
		buf[byte_count + 2] = pec;
		byte_count++;
	}

	unsigned i = 0;
	int result = 0;

	// If block_write fails, try up to 10 times.
	while (i < 10 && ((result = transfer((uint8_t *)buf, byte_count + 2, nullptr, 0)) != 0)) {
		i++;
	}

	if (i == 10 || result) {
		result = -EINVAL;
	}

	return result;
}

uint8_t SMBus::get_pec(uint8_t *buff, const uint8_t len)
{
	// TODO: use "return crc8ccitt(buff, len);"

	// Initialise CRC to zero.
	uint8_t crc = 0;
	uint8_t shift_register = 0;
	bool invert_crc;

	// Calculate crc for each byte in the stream.
	for (uint8_t i = 0; i < len; i++) {
		// Load next data byte into the shift register
		shift_register = buff[i];

		// Calculate crc for each bit in the current byte.
		for (uint8_t j = 0; j < 8; j++) {
			invert_crc = (crc ^ shift_register) & 0x80;
			crc <<= 1;
			shift_register <<= 1;

			if (invert_crc) {
				crc ^= SMBUS_PEC_POLYNOMIAL;
			}
		}
	}

	return crc;
}

