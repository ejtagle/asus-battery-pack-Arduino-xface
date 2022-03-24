/**********************************

 * SMBus FOR 3DR SOLO
 * STAVROPOULOS
 * Code Version 0.01 beta
 *
 * MUCH OF THIS CODE WAS COPIED FROM
 * https://github.com/PowerCartel/PackProbe/blob/master/PackProbe/PackProbe.ino
 * https://github.com/ArduPilot/PX4Firmware/blob/master/src/drivers/batt_smbus/batt_smbus.cpp
 *
 **********************************/

#include "BQ40ZX0.hpp"

  #define BAUD_RATE 115200


/**********************************
 * DEFINE VARIABLES AND SMBus MAPPINGS
 **********************************/
  #define BATT_SMBUS_ADDR                     0x0B                ///< I2C address
  #define BATT_SMBUS_ADDR_MIN                 0x08                ///< lowest possible address
  #define BATT_SMBUS_ADDR_MAX                 0x7F                ///< highest possible address
  

BQ40ZX0 batt;

void setup()
{


  //INITIATE SERIAL CONSOLE
    Serial.begin(BAUD_RATE);
    Serial.flush();

    while (!Serial) {
    ;                                                       //wait for Console port to connect.
    }

    Serial.println("Console Initialized");
 
    batt.init();                                             //i2c_start initialized the I2C system.  will return false if bus is locked.
    Serial.println("I2C Inialized");
}

struct fldentry {
  uint8_t pos;
  uint8_t len;
  const char* descr;
  const char* values[8];
};
    
void dump_fields(uint32_t value,const struct fldentry* fields) {
  int i = 0;
  while (fields[i].descr) {
    Serial.print("  ");
    Serial.print(fields[i].descr);
    Serial.print(" :");
    uint32_t mask = (1<<fields[i].len) - 1;
    uint32_t n = (value >> fields[i].pos) & mask;
    Serial.println(fields[i].values[n]);
    ++i;
  };
}

void loop()
{
    if (batt.is_batt_sealed()) {
      Serial.println("Battery pack is sealed. Try to unseal it");
      // Lets try to UNSEAL the pack
      batt.unseal();
      // And reset all permanent failure flags
      batt.reset_pf_data_flag();
    }
    Serial.print("Battery Status: ");
    Serial.println(batt.is_batt_sealed()?"SEALED":"UNSEALED");

    Serial.print("Manufacturer Name: ");
    char manuf[BATT_SMBUS_MANUFACTURER_NAME_SIZE];
    batt.get_batt_manufacturer_name(manuf); 
    Serial.println(manuf);

    Serial.print("Device Name: ");
    char dev[BATT_SMBUS_DEVICE_NAME_SIZE];
    batt.get_batt_device_name(dev); 
    Serial.println(dev);

    Serial.print("Design Capacity: " );
    Serial.println(batt.get_batt_design_capacity());
 
    Serial.print("Design Voltage: " );
    Serial.println(batt.get_batt_design_voltage());

    Serial.print("Charging Voltage: " );
    Serial.println(batt.get_batt_charging_voltage());

    Serial.print("Serial Number: ");
    Serial.println(batt.get_batt_serial_number());
 
    Serial.print("Voltage: ");
    Serial.println(batt.get_batt_voltage());
 
    Serial.print("Full Charge Capacity: " );
    Serial.println(batt.get_batt_full_charge_capacity());
 
    Serial.print("Remaining Capacity: " );
    Serial.println(batt.get_batt_remaining_capacity());
 
    Serial.print("Temp: ");
    Serial.println(batt.get_batt_temp());
 
    Serial.print("Current (A): " );
    Serial.println(batt.get_batt_current());

    Serial.print("Charging Current (A): " );
    Serial.println(batt.get_batt_charging_current());

    Serial.print("Average Current (A): " );
    Serial.println(batt.get_batt_average_current());

    String formatted_date = "Manufacture Date (Y-M-D): ";
    int mdate = batt.get_batt_manufacture_date();
    int mday = B00011111 & mdate;
    int mmonth = mdate>>5 & B00001111;
    int myear = 1980 + (mdate>>9 & B01111111);
    formatted_date += myear;
    formatted_date += "-";
    formatted_date += mmonth;
    formatted_date += "-";
    formatted_date += mday;
    Serial.println(formatted_date);
 
    Serial.print("Cycle Count: " );
    Serial.println(batt.get_batt_cycle_count());
 
    Serial.print("Relative Charge(%): ");
    Serial.println(batt.get_batt_relative_soc());
  
 
    // These aren't part of the standard, but work with some packs.
    // They don't work with the Lenovo and Dell packs we've tested
    float cellvoltages[4];
    batt.get_cell_voltages(cellvoltages);
    Serial.print("Cell 1 Voltage: ");
    Serial.println(cellvoltages[0]);
    Serial.print("Cell 2 Voltage: ");
    Serial.println(cellvoltages[1]);
    Serial.print("Cell 3 Voltage: ");
    Serial.println(cellvoltages[2]);
    Serial.print("Cell 4 Voltage: ");
    Serial.println(cellvoltages[3]);
 
    Serial.print("State of Health: ");
    Serial.println(batt.get_batt_state_of_health());

    Serial.print("Battery Mode (BIN): 0b");
    uint16_t mode = batt.get_batt_mode();
    Serial.println(mode,BIN);
    static const fldentry batt_mode_fields[] = {
      {15,1,"CAPACITY Mode reports in",{"mA/mAh","10mW/10mWh"}},
      {14,1,"CHARGER Mode",{"NO","YES"}},
      {13,1,"Alarm Mode",{"NO","YES"}},
      { 9,1,"Primary Battery",{"NO","YES"}},
      { 8,1,"Charge Controller Enabled",{"NO","YES"}},
      { 7,1,"Condition Flag",{"Battery OK","Conditioning Cycle Requested"}},
      { 1,1,"Primary Battery Support"   ,{"NO","YES"}},
      { 0,1,"Internal Charge Controller",{"NO","YES"}},
      {0}
    };
    dump_fields(mode,batt_mode_fields);

    Serial.print("Safety Status (BIN): 0b");
    uint32_t safety_status = batt.get_batt_safety_status(); 
    Serial.println(safety_status,BIN);
    static const fldentry batt_safety_status_fields[] = {
      {27,1,"Undertemperature During Discharge",{"NO","YES"}},
      {26,1,"Undertemperature During Charge",{"NO","YES"}},
      {25,1,"Over Precharge Current",{"NO","YES"}},
      {24,1,"Overcharging Voltage",{"NO","YES"}},
      {23,1,"Overcharging Current",{"NO","YES"}},
      {22,1,"Overcharge",{"NO","YES"}},
      {20,1,"Charge Timeout",{"NO","YES"}},
      {18,1,"Precharge Timeout",{"NO","YES"}},
      {16,1,"Overtemperature FET",{"NO","YES"}},
      {14,1,"Cell Undervoltage Compensated",{"NO","YES"}},
      {13,1,"Overtemperature During Discharge",{"NO","YES"}},
      {12,1,"Overtemperature During Charge",{"NO","YES"}},
      {11,1,"Short Circuit During Discharge Latch",{"NO","YES"}},
      {10,1,"Short Circuit During Discharge",{"NO","YES"}},
      { 9,1,"Short Circuit During Charge Latch",{"NO","YES"}},
      { 8,1,"Short Circuit During Charge",{"NO","YES"}},
      { 7,1,"Overload During Discharge Latch",{"NO","YES"}},
      { 6,1,"Overload During Discharge",{"NO","YES"}},
      { 5,1,"Overcurrent During Discharge 2",{"NO","YES"}},
      { 4,1,"Overcurrent During Discharge 1",{"NO","YES"}},
      { 3,1,"Overcurrent During Charge 2",{"NO","YES"}},
      { 2,1,"Overcurrent During Charge 1",{"NO","YES"}},
      { 1,1,"Cell Overvoltage"   ,{"NO","YES"}},
      { 0,1,"Cell Undervoltage",{"NO","YES"}},
      {0}
    };
    dump_fields(safety_status,batt_safety_status_fields);

    Serial.print("PF Status (BIN): 0b");
    uint32_t pf_status = batt.get_batt_pf_status();
    Serial.println(pf_status,BIN);
    static const fldentry batt_pf_status_fields[] = {
      {31,1,"Open Thermistor TS4 Failure",{"NO","YES"}},
      {30,1,"Open Thermistor TS3 Failure",{"NO","YES"}},
      {29,1,"Open Thermistor TS2 Failure",{"NO","YES"}},
      {28,1,"Open Thermistor TS1 Failure",{"NO","YES"}},
      {26,1,"Data Flash Wearout Failure",{"NO","YES"}},
      {25,1,"Open Cell Tab Connection Failure",{"NO","YES"}},
      {24,1,"Instruction FLASH Checksum Failure",{"NO","YES"}},
      {23,1,"OTC Failure",{"NO","YES"}},
      {22,1,"Second Level Protector Failure",{"NO","YES"}},
      {21,1,"AFE Communication Failure",{"NO","YES"}},
      {20,1,"AFE Register Failure",{"NO","YES"}},
      {19,1,"Chemical Fuse Failure",{"NO","YES"}},
      {17,1,"Discharge FET Failure",{"NO","YES"}},
      {16,1,"Charge FET Failure",{"NO","YES"}},
      {12,1,"Voltage Imbalance While Package Is Active Failure",{"NO","YES"}},
      {11,1,"Voltage Imbalance While Package At Rest Failure",{"NO","YES"}},
      {10,1,"Capacity Degradation Failure",{"NO","YES"}},
      { 9,1,"Impedance Failure",{"NO","YES"}},
      { 8,1,"Cell Balancing Failure",{"NO","YES"}},
      { 7,1,"QMax Imbalance Failure",{"NO","YES"}},
      { 6,1,"Safety Overtemperature FET Failute",{"NO","YES"}},
      { 4,1,"Safety Overtemperature Cell Failure",{"NO","YES"}},
      { 3,1,"Safety Overcurrent In Discharge",{"NO","YES"}},
      { 2,1,"Safety Overcurrent In Charge",{"NO","YES"}},
      { 1,1,"Safety Cell Overvoltage Failure",{"NO","YES"}},
      { 0,1,"Safety Cell Undervoltage Failure",{"NO","YES"}},
      {0}
    };
    dump_fields(pf_status,batt_pf_status_fields);

    Serial.print("Operation Status (BIN): 0b");
    uint32_t operation_status = batt.get_batt_operation_status();
    Serial.println(operation_status,BIN);
    static const fldentry batt_operation_status_fields[] = {
      {29,1,"Emergency Shutdown",{"NO","YES"}},
      {28,1,"Cell Balancing Status",{"NO","YES"}},
      {27,1,"CC Measurement in Sleep Mode",{"NO","YES"}},
      {26,1,"ADC Measurement in Sleep Mode",{"NO","YES"}},
      {24,1,"Initialization after Full Reset",{"NO","YES"}},
      {23,1,"Sleep Mode Triggered via Command",{"NO","YES"}},
      {22,1,"400khz SMBus Mode",{"NO","YES"}},
      {21,1,"Calibration Output Offset",{"NO","YES"}},
      {20,1,"Calibration Output",{"NO","YES"}},
      {19,1,"Auto CC Offset Calibration",{"NO","YES"}},
      {18,1,"Authentication In Progress",{"NO","YES"}},
      {17,1,"LED Displsy",{"NO","YES"}},
      {16,1,"Shutdown Triggered Via Command",{"NO","YES"}},
      {15,1,"Sleep Mode Conditions Met",{"NO","YES"}},
      {14,1,"Charging Disabled",{"NO","YES"}},
      {13,1,"Discharging Disabled",{"NO","YES"}},
      {12,1,"PERMANENT FAILURE Mode Status",{"NO","YES"}},
      {11,1,"SAFETY Mode Status",{"NO","YES"}},
      {10,1,"Shutdown triggered via low pack voltage",{"NO","YES"}},
      { 8,2,"SECURITY Mode",{"Reserved","Full Access","Unsealed","Sealed"}},
      { 7,1,"Battery Trip Point Interrupt",{"NO","YES"}},
      { 5,1,"Fuse Status",{"OFF","ON"}},
      { 3,1,"Precharge FET Status",{"OFF","ON"}},
      { 2,1,"CHG FET Status",{"OFF","ON"}},
      { 1,1,"DSG FET Status",{"OFF","ON"}},
      { 0,1,"System Present LOW",{"NO","YES"}},
      {0}
    };
    dump_fields(operation_status,batt_operation_status_fields);

    Serial.print("Battery Status (BIN): 0b");
    Serial.println(batt.get_batt_status(),BIN);

 
    Serial.println(".");
    delay(600000);
}
