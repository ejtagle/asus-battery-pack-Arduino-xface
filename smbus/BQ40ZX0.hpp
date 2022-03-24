/**
 * @file batt_smbus.h
 *
 * Header for a battery monitor connected via SMBus (I2C).
 * Designed for BQ40Z50-R1/R2 or BQ40Z80
 *
 * @author Jacob Dahl <dahl.jakejacob@gmail.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 * @author Bazooka Joe <BazookaJoe1900@gmail.com>
 */

#pragma once

#include "SMBus.hpp"

#define BATT_SMBUS_MEASUREMENT_INTERVAL_US              100_ms         ///< time in microseconds, measure at 10Hz

#define MAC_DATA_BUFFER_SIZE                            32

#define BATT_CELL_VOLTAGE_THRESHOLD_RTL                 0.5f            ///< Threshold in volts to RTL if cells are imbalanced
#define BATT_CELL_VOLTAGE_THRESHOLD_FAILED              1.5f            ///< Threshold in volts to Land if cells are imbalanced

#define BATT_CURRENT_UNDERVOLTAGE_THRESHOLD             5.0f            ///< Threshold in amps to disable undervoltage protection
#define BATT_VOLTAGE_UNDERVOLTAGE_THRESHOLD             3.4f            ///< Threshold in volts to re-enable undervoltage protection

#define BATT_SMBUS_ADDR                                 0x0B            ///< Default 7 bit address I2C address. 8 bit = 0x16

#define BATT_SMBUS_MANUFACTURER_ACCESS                  0x00
#define BATT_SMBUS_MODE                                 0x03 
#define BATT_SMBUS_TEMP                                 0x08            ///< temperature register
#define BATT_SMBUS_VOLTAGE                              0x09            ///< voltage register
#define BATT_SMBUS_CURRENT                              0x0A            ///< current register
#define BATT_SMBUS_AVERAGE_CURRENT                      0x0B            ///< average current register
#define BATT_SMBUS_MAX_ERROR                            0x0C            ///< max error
#define BATT_SMBUS_RELATIVE_SOC                         0x0D            ///< Relative State Of Charge
#define BATT_SMBUS_ABSOLUTE_SOC                         0x0E            ///< Absolute State of charge
#define BATT_SMBUS_REMAINING_CAPACITY                   0x0F            ///< predicted remaining battery capacity as a percentage
#define BATT_SMBUS_FULL_CHARGE_CAPACITY                 0x10            ///< capacity when fully charged
#define BATT_SMBUS_RUN_TIME_TO_EMPTY                    0x11            ///< predicted remaining battery capacity based on the present rate of discharge in min
#define BATT_SMBUS_AVERAGE_TIME_TO_EMPTY                0x12            ///< predicted remaining battery capacity based on the present rate of discharge in min
#define BATT_SMBUS_CHARGING_CURRENT                     0x14
#define BATT_SMBUS_CHARGING_VOLTAGE                     0x15
#define BATT_SMBUS_STATUS                               0x16
#define BATT_SMBUS_CYCLE_COUNT                          0x17            ///< number of cycles the battery has experienced
#define BATT_SMBUS_DESIGN_CAPACITY                      0x18            ///< design capacity register
#define BATT_SMBUS_DESIGN_VOLTAGE                       0x19            ///< design voltage register
#define BATT_SMBUS_MANUFACTURE_DATE                     0x1B            ///< manufacture date register
#define BATT_SMBUS_SERIAL_NUMBER                        0x1C            ///< serial number register
#define BATT_SMBUS_MANUFACTURER_NAME                    0x20            ///< manufacturer name
#define BATT_SMBUS_MANUFACTURER_NAME_SIZE               21              ///< manufacturer name data size
#define BATT_SMBUS_DEVICE_NAME                          0x21            ///< manufacturer name
#define BATT_SMBUS_DEVICE_NAME_SIZE                     21              ///< manufacturer name data size
#define BATT_SMBUS_MANUFACTURER_DATA                    0x23
#define BATT_SMBUS_BQ40Z50_CELL_4_VOLTAGE               0x3C
#define BATT_SMBUS_BQ40Z50_CELL_3_VOLTAGE               0x3D
#define BATT_SMBUS_BQ40Z50_CELL_2_VOLTAGE               0x3E
#define BATT_SMBUS_BQ40Z50_CELL_1_VOLTAGE               0x3F
#define BATT_SMBUS_BQ40Z80_CELL_7_VOLTAGE               0x3C
#define BATT_SMBUS_BQ40Z80_CELL_6_VOLTAGE               0x3D
#define BATT_SMBUS_BQ40Z80_CELL_5_VOLTAGE               0x3E
#define BATT_SMBUS_BQ40Z80_CELL_4_VOLTAGE               0x3F
#define BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS            0x44
#define BATT_SMBUS_STATE_OF_HEALTH                      0x4F            ///< State of Health. The SOH information of the battery in percentage of Design Capacity


// Manufacturer access command list
#define BATT_SMBUS_PF_DATA_RESET                        0x0029 /// Permanent fail data reset 
#define BATT_SMBUS_LIFETIME_FLUSH                       0x002E
#define BATT_SMBUS_SEAL                                 0x0030
#define BATT_SMBUS_SECURITY_KEYS                        0x0035
#define BATT_SMBUS_SAFETY_STATUS                        0x0051
#define BATT_SMBUS_PF_STATUS                            0x0053
#define BATT_SMBUS_OPERATION_STATUS                     0x0054
#define BATT_SMBUS_LIFETIME_BLOCK_1                     0x0060
#define BATT_SMBUS_LIFETIME_BLOCK_2                     0x0061
#define BATT_SMBUS_LIFETIME_BLOCK_3                     0x0062
#define BATT_SMBUS_DASTATUS1                            0x0071
#define BATT_SMBUS_DASTATUS2                            0x0072
#define BATT_SMBUS_DASTATUS3                            0x007B

// FLASH addresses 
#define BATT_SMBUS_ENABLED_PF_A_ADDRESS                 0x48BE
#define BATT_SMBUS_ENABLED_PF_B_ADDRESS                 0x48BF
#define BATT_SMBUS_ENABLED_PF_C_ADDRESS                 0x48C0
#define BATT_SMBUS_ENABLED_PF_D	_ADDRESS                 0x48C1

#define BATT_SMBUS_ENABLED_PROTECTIONS_A_ADDRESS        0x4938

#define BATT_SMBUS_ENABLED_PROTECTIONS_A_DEFAULT        0xcf
#define BATT_SMBUS_ENABLED_PROTECTIONS_A_CUV_DISABLED   0xce

class BQ40ZX0 : public SMBus
{
public:
	BQ40ZX0();
	~BQ40ZX0();


	/**
	 * @brief Reads data from flash.
	 * @param address The address to start the read from.
	 * @param data The returned data.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int dataflash_read(const uint16_t address, void *data, const unsigned length);

	/**
	 * @brief Writes data to flash.
	 * @param address The start address of the write.
	 * @param data The data to be written.
	 * @param length The number of bytes being written.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int dataflash_write(const uint16_t address, void *data, const unsigned length);

	/**
	 * @brief Performs a ManufacturerBlockAccess() read command.
	 * @param cmd_code The command code.
	 * @param data The returned data.
	 * @param length The number of bytes being written.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int manufacturer_read(const uint16_t cmd_code, void *data, const unsigned length);

	/**
	 * @brief Performs a ManufacturerBlockAccess() write command.
	 * @param cmd_code The command code.
	 * @param data The sent data.
	 * @param length The number of bytes being written.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int manufacturer_write(const uint16_t cmd_code, void *data, const unsigned length);

	/**
	 * @brief Unseals the battery to allow writing to restricted flash.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int unseal();

	/**
	 * @brief Seals the battery to disallow writing to restricted flash.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int seal();

	/**
	 * @brief This command flushes the RAM Lifetime Data to data flash to help streamline evaluation testing.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int lifetime_data_flush();

	/**
	 * @brief Reads the lifetime data from block 1.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int lifetime_read_block_one();

	/**
	 * @brief Reads the cell voltages.
	 * @return Returns PX4_OK on success or associated read error code on failure.
	 */
	int get_cell_voltages(float (&cell_voltages)[4]);

	/**
	 * @brief Enables or disables the cell under voltage protection emergency shut off.
	 */
	void disable_undervoltage_protection();
	void enable_undervoltage_protection();

	/**
	 * Resets and removes the permanent failure flag. The pack
	 *   should return to life. All permanent failure conditions
	 *   must be removed previously, and pack must be in UNSEALED mode.
	 *   Dont forget to reseal the battery after that!
	 */
    void reset_pf_data_flag() {
		manufacturer_write(BATT_SMBUS_PF_DATA_RESET, nullptr, 0);
	}
	
	/**
	 * Check if battery is sealed
	 */
	bool is_batt_sealed() {
		uint32_t result = 0;
		manufacturer_read(BATT_SMBUS_OPERATION_STATUS,&result,4);
		return ((result >> 8) & 3) == 3;  
	}
	uint16_t get_batt_mode() {
		uint16_t result;		
		read_word(BATT_SMBUS_MODE, result); 
		return result;
	}
	uint32_t get_batt_safety_status() {
		uint32_t result = 0;
		manufacturer_read(BATT_SMBUS_SAFETY_STATUS,&result,4);
		return result;
	}
	uint32_t get_batt_pf_status() {
		uint32_t result = 0;
		manufacturer_read(BATT_SMBUS_PF_STATUS,&result,4);
		return result;
	}
	uint32_t get_batt_operation_status() {
		uint32_t result = 0;
		manufacturer_read(BATT_SMBUS_OPERATION_STATUS,&result,4);
		return result;
	}
	uint16_t get_batt_status() {
		uint16_t result;		
		read_word(BATT_SMBUS_STATUS, result); 
		return result;
	}
	uint16_t get_batt_serial_number() {
		uint16_t result;		
		read_word(BATT_SMBUS_SERIAL_NUMBER, result); 
		return result;
	}
	uint16_t get_batt_cycle_count() {
		uint16_t result;		
		read_word(BATT_SMBUS_CYCLE_COUNT, result); 
		return result;
	}
	uint16_t get_batt_manufacture_date() {
		uint16_t result;		
		read_word(BATT_SMBUS_MANUFACTURE_DATE, result); 
		return result;
	}
	float get_batt_design_voltage() {
		uint16_t result;		
		read_word(BATT_SMBUS_DESIGN_VOLTAGE, result); 
		return result / 1000.0f;
	}
	float get_batt_voltage() {
		uint16_t result;		
		read_word(BATT_SMBUS_VOLTAGE, result); 
		return result / 1000.0f;
	}
	float get_batt_current() {
		uint16_t result;		
		read_word(BATT_SMBUS_CURRENT, result); 
		return result / 1000.0f;
	}
	float get_batt_charging_voltage() {
		uint16_t result;		
		read_word(BATT_SMBUS_CHARGING_VOLTAGE, result); 
		return result / 1000.0f;
	}
	float get_batt_charging_current() {
		uint16_t result;		
		read_word(BATT_SMBUS_CHARGING_CURRENT, result); 
		return result / 1000.0f;
	}
	float get_batt_average_current() {
		uint16_t result;		
		read_word(BATT_SMBUS_AVERAGE_CURRENT, result); 
		return result / 1000.0f;
	}
	float get_batt_run_time_to_empty() {
		uint16_t result;		
		read_word(BATT_SMBUS_RUN_TIME_TO_EMPTY, result); 
		return result / 60.0f;
	}
	float get_batt_average_time_to_empty() {
		uint16_t result;		
		read_word(BATT_SMBUS_AVERAGE_TIME_TO_EMPTY, result); 
		return result / 60.0f;
	}
	uint16_t get_batt_design_capacity() {
		uint16_t result;		
		read_word(BATT_SMBUS_DESIGN_CAPACITY, result); 
		return result;
	}
	uint16_t get_batt_full_charge_capacity() {
		uint16_t result;		
		read_word(BATT_SMBUS_FULL_CHARGE_CAPACITY, result); 
		return result;
	}
	uint16_t get_batt_remaining_capacity() {
		uint16_t result;		
		read_word(BATT_SMBUS_REMAINING_CAPACITY, result); 
		return result;
	}
	float get_batt_state_of_health() {
		uint16_t result;		
		read_word(BATT_SMBUS_STATE_OF_HEALTH, result); 
		return result;
	}
	float get_batt_relative_soc() {
		uint16_t result;		
		read_word(BATT_SMBUS_RELATIVE_SOC, result); 
		return result;
	}
	float get_batt_max_error() {
		uint16_t result;		
		read_word(BATT_SMBUS_MAX_ERROR, result); 
		return result;
	}
	float get_batt_temp() {
		uint16_t result;		
		read_word(BATT_SMBUS_TEMP, result); 
		return ((float)result / 10.0f) - 273.15;
	}

	int get_batt_manufacturer_name(char (&manufacturer_name) [BATT_SMBUS_MANUFACTURER_NAME_SIZE]) {
		int ret = block_read(BATT_SMBUS_MANUFACTURER_NAME, manufacturer_name, BATT_SMBUS_MANUFACTURER_NAME_SIZE, true);
		manufacturer_name[sizeof(manufacturer_name) - 1] = '\0';
		return ret;
	}
	
	int get_batt_device_name(char (&device_name) [BATT_SMBUS_DEVICE_NAME_SIZE]) {
		int ret = block_read(BATT_SMBUS_DEVICE_NAME, device_name, BATT_SMBUS_DEVICE_NAME_SIZE, true);
		device_name[sizeof(device_name) - 1] = '\0';
		return ret;
	}

private:

	BQ40ZX0(const BQ40ZX0 &) = delete;
	BQ40ZX0 operator=(const BQ40ZX0 &) = delete;
};
