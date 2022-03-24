/**
 * @file batt_smbus.h
 *
 * Header for a battery monitor connected via SMBus (I2C).
 * Designed for BQ40Z50-R1/R2 and BQ40Z80
 *
 * @author Jacob Dahl <dahl.jakejacob@gmail.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 * @author Bazooka Joe <BazookaJoe1900@gmail.com>
 *
 */

#include "BQ40ZX0.hpp"

BQ40ZX0::BQ40ZX0() 
	: SMBus(BATT_SMBUS_ADDR)
{
}

BQ40ZX0::~BQ40ZX0()
{
}

int BQ40ZX0::get_cell_voltages(float (&cell_voltages)[4])
{
	// Temporary variable for storing SMBUS reads.
	uint16_t result = 0;
	int ret = 0;

	ret | read_word(BATT_SMBUS_BQ40Z50_CELL_1_VOLTAGE, result);
	// Convert millivolts to volts.
	cell_voltages[0] = ((float)result) / 1000.0f;

	ret |= read_word(BATT_SMBUS_BQ40Z50_CELL_2_VOLTAGE, result);
	// Convert millivolts to volts.
	cell_voltages[1] = ((float)result) / 1000.0f;

	ret |= read_word(BATT_SMBUS_BQ40Z50_CELL_3_VOLTAGE, result);
	// Convert millivolts to volts.
	cell_voltages[2] = ((float)result) / 1000.0f;

	ret |= read_word(BATT_SMBUS_BQ40Z50_CELL_4_VOLTAGE, result);
	// Convert millivolts to volts.
	cell_voltages[3] = ((float)result) / 1000.0f;

	return ret;
}

void BQ40ZX0::disable_undervoltage_protection()
{
	// Disable undervoltage protection
	uint8_t protections_a_tmp = BATT_SMBUS_ENABLED_PROTECTIONS_A_CUV_DISABLED;
	uint16_t address = BATT_SMBUS_ENABLED_PROTECTIONS_A_ADDRESS;

	dataflash_write(address, &protections_a_tmp, 1);
}

void BQ40ZX0::enable_undervoltage_protection()
{
	// Enable undervoltage protection
	uint8_t protections_a_tmp = BATT_SMBUS_ENABLED_PROTECTIONS_A_DEFAULT;
	uint16_t address = BATT_SMBUS_ENABLED_PROTECTIONS_A_ADDRESS;

	dataflash_write(address, &protections_a_tmp, 1);
}

int BQ40ZX0::dataflash_read(const uint16_t address, void *data, const unsigned length)
{
	uint8_t code = BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS;

	int ret = block_write(code, &address, 2, true);

	if (ret != 0) {
		return ret;
	}

	ret = block_read(code, data, length, true);

	return ret;
}

int BQ40ZX0::dataflash_write(const uint16_t address, void *data, const unsigned length)
{
	uint8_t code = BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS;

	uint8_t tx_buf[MAC_DATA_BUFFER_SIZE + 2] = {};

	tx_buf[0] = address & 0xff;
	tx_buf[1] = (address >> 8) & 0xff;

	if (length > MAC_DATA_BUFFER_SIZE) {
		return -1;
	}

	memcpy(&tx_buf[2], data, length);

	// code (1), byte_count (1), addr(2), data(32) + pec
	int ret = block_write(code, tx_buf, length + 2, false);

	return ret;
}

int BQ40ZX0::manufacturer_read(const uint16_t cmd_code, void *data, const unsigned length)
{
	uint8_t code = BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS;

	uint8_t address[2] = {};
	address[0] = ((uint8_t *)&cmd_code)[0];
	address[1] = ((uint8_t *)&cmd_code)[1];

	int ret = block_write(code, address, 2, false);

	if (ret != 0) {
		return ret;
	}

	ret = block_read(code, data, length, true);
	memmove(data, &((uint8_t *)data)[2], length - 2); // remove the address bytes

	return ret;
}

int BQ40ZX0::manufacturer_write(const uint16_t cmd_code, void *data, const unsigned length)
{
	uint8_t code = BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS;

	uint8_t tx_buf[MAC_DATA_BUFFER_SIZE + 2] = {};
	tx_buf[0] = cmd_code & 0xff;
	tx_buf[1] = (cmd_code >> 8) & 0xff;

	if (data != nullptr && length <= MAC_DATA_BUFFER_SIZE) {
		memcpy(&tx_buf[2], data, length);
	}

	int ret = block_write(code, tx_buf, length + 2, false);

	return ret;
}

int BQ40ZX0::unseal()
{
	// See bq40z50 technical reference.
	uint16_t keys[2] = {0x0414, 0x3672};

	int ret = write_word(BATT_SMBUS_MANUFACTURER_ACCESS, keys[0]);

	ret |= write_word(BATT_SMBUS_MANUFACTURER_ACCESS, keys[1]);

	return ret;
}

int BQ40ZX0::seal()
{
	// See bq40z50 technical reference.
	return manufacturer_write(BATT_SMBUS_SEAL, nullptr, 0);
}

int BQ40ZX0::lifetime_data_flush()
{
	return manufacturer_write(BATT_SMBUS_LIFETIME_FLUSH, nullptr, 0);
}

int BQ40ZX0::lifetime_read_block_one()
{
	uint8_t lifetime_block_one[32 + 2] = {}; // 32 bytes of data and 2 bytes of address

	if (0 != manufacturer_read(BATT_SMBUS_LIFETIME_BLOCK_1, lifetime_block_one, sizeof(lifetime_block_one))) {
		return -1;
	}

	return 0;
}

