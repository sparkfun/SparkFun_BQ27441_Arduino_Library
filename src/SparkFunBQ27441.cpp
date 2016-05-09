/******************************************************************************
SparkFunBQ27441.cpp
BQ27441 Arduino Library Main Source File
Jim Lindblom @ SparkFun Electronics
May 9, 2016
https://github.com/sparkfun/SparkFun_BQ27441_Arduino_Library

Implementation of all features of the BQ27441 LiPo Fuel Gauge.

Hardware Resources:
- Arduino Development Board
- SparkFun Battery Babysitter

Development environment specifics:
Arduino 1.6.7
SparkFun Battery Babysitter v1.0
Arduino Uno (any 'duino should do)
******************************************************************************/

#include "Arduino.h"
#include <Wire.h>
#include "SparkFunBQ27441.h"
#include "BQ27441_Definitions.h"

#define monitor Serial

/****************************** Public Functions *****************************/

BQ27441::BQ27441() : _deviceAddress(BQ72441_I2C_ADDRESS)
{
}

bool BQ27441::begin(void)
{
	uint16_t deviceID = 0;
	
	Wire.begin(); // Initialize I2C master
	
	deviceID = deviceType(); // Read deviceType from BQ27441
	
	if (deviceID == BQ27441_DEVICE_ID)
	{
		return true; // If device ID is valid, return true
	}
	
	return false; // Otherwise return false
}

bool BQ27441::setCapacity(uint16_t capacity)
{
	// Write to STATE subclass (82) of BQ27441 extended memory.
	// Offset 0x0A (10)
	// Design capacity is a 2-byte piece of data - MSB first
	uint8_t capMSB = capacity >> 8;
	uint8_t capLSB = capacity & 0x00FF;
	uint8_t capacityData[2] = {capMSB, capLSB};
	writeExtendedData(BQ27441_ID_STATE, 10, capacityData, 2);
	// Even though we check a flag, this still seems to take settling time:
	delay(1500);
}

uint16_t BQ27441::temperature(void)
{
	return readWord(BQ27441_COMMAND_TEMP);
}

uint16_t BQ27441::voltage(void)
{
	return readWord(BQ27441_COMMAND_VOLTAGE);
}

//! Give this a parameter, return based on parameter
//! default to average.
int16_t BQ27441::current(void)
{
	return averageCurrent();
}

//! Give this a parameter, return based on parameter
//! default to remaining.
uint16_t BQ27441::capacity(void)
{
	return remainingCapacity();
}

uint16_t BQ27441::nominalAvailableCapacity(void)
{
	return readWord(BQ27441_COMMAND_NOM_CAPACITY);
}

uint16_t BQ27441::fullAvailableCapacity(void)
{
	return readWord(BQ27441_COMMAND_AVAIL_CAPACITY);
}

uint16_t BQ27441::remainingCapacity(void)
{
	return readWord(BQ27441_COMMAND_REM_CAPACITY);
}

uint16_t BQ27441::fullChargeCapacity(void)
{
	return readWord(BQ27441_COMMAND_FULL_CAPACITY);
}

int16_t BQ27441::averageCurrent(void)
{
	return (int16_t) readWord(BQ27441_COMMAND_AVG_CURRENT);
}

int16_t BQ27441::standbyCurrent(void)
{
	return (int16_t) readWord(BQ27441_COMMAND_STDBY_CURRENT);
}

int16_t BQ27441::maxLoadCurrent(void)
{
	return (int16_t) readWord(BQ27441_COMMAND_MAX_CURRENT);
}

int16_t BQ27441::averagePower(void)
{
	return (int16_t) readWord(BQ27441_COMMAND_AVG_POWER);
}

uint16_t BQ27441::soc(void)
{
	return readWord(BQ27441_COMMAND_SOC);
}

uint16_t BQ27441::soh(void)
{
	return readWord(BQ27441_COMMAND_SOH);
}

uint16_t BQ27441::internalTemperature(void)
{
	return readWord(BQ27441_COMMAND_INT_TEMP);
}

uint16_t BQ27441::remainingCapacityUnfilitered(void)
{
	return readWord(BQ27441_COMMAND_REM_CAP_UNFL);
}

uint16_t BQ27441::remainingCapacityFiltered(void)
{
	return readWord(BQ27441_COMMAND_REM_CAP_FIL);
}

uint16_t BQ27441::fullChargeCapacityUnfiltered(void)
{
	return readWord(BQ27441_COMMAND_FULL_CAP_UNFL);
}

uint16_t BQ27441::fullChargeCapacityFiltered(void)
{
	return readWord(BQ27441_COMMAND_FULL_CAP_FIL);
}

uint16_t BQ27441::socUnfiltered(void)
{
	return readWord(BQ27441_COMMAND_SOC_UNFL);
}

uint16_t BQ27441::flags(void)
{
	return readWord(BQ27441_COMMAND_FLAGS);
}

uint16_t BQ27441::status(void)
{
	return readControlWord(BQ27441_CONTROL_STATUS);
}

uint16_t BQ27441::deviceType(void)
{
	return readControlWord(BQ27441_CONTROL_DEVICE_TYPE);
}

bool BQ27441::sealed(void)
{
	uint16_t stat = status();
	return stat & BQ27441_STATUS_SS;
}

bool BQ27441::seal(void)
{
	return readControlWord(BQ27441_CONTROL_SEALED);
}

bool BQ27441::unseal(void)
{
	// To unseal the BQ27441, write the key to the control
	// command. Then immediately write the same key to control again.
	if (readControlWord(BQ27441_UNSEAL_KEY))
	{
		return readControlWord(BQ27441_UNSEAL_KEY);
	}
	return false;
}

bool BQ27441::enterConfig(void)
{
	if (executeControlWord(BQ27441_CONTROL_SET_CFGUPDATE))
	{
		int16_t timeout = BQ72441_I2C_TIMEOUT;
		while ((timeout--) && (!(status() & BQ27441_FLAG_CFGUPMODE)))
			delay(1);
		
		if (timeout > 0)
			return true;
	}
	
	return false;
}

bool BQ27441::exitConfig(bool resim)
{
	// There are two methods for exiting config mode:
	//    1. Execute the EXIT_CFGUPDATE command
	//    2. Execute the SOFT_RESET command
	// EXIT_CFGUPDATE exits config mode _without_ an OCV (open-circuit voltage)
	// measurement, and without resimulating to update unfiltered-SoC and SoC.
	// If a new OCV measurement or resimulation is desired, SOFT_RESET or
	// EXIT_RESIM should be used to exit config mode.
	if (resim)
	{
		if (softReset())
		{
			int16_t timeout = BQ72441_I2C_TIMEOUT;
			while ((timeout--) && ((flags() & BQ27441_FLAG_CFGUPMODE)))
				delay(1);
			if (timeout > 0)
				return true;
		}
		return false;
	}
	else
	{
		return executeControlWord(BQ27441_CONTROL_EXIT_CFGUPDATE);
	}	
}

bool BQ27441::softReset(void)
{
	return executeControlWord(BQ27441_CONTROL_SOFT_RESET);
}

uint16_t BQ27441::readOpConfig(void)
{
	return readWord(BQ27441_EXTENDED_OPCONFIG);
}

uint16_t BQ27441::readDesignCapacity(void)
{
	return readWord(BQ27441_EXTENDED_CAPACITY);
}

bool BQ27441::blockDataControl(void)
{
	uint8_t enableByte = 0x00;
	return i2cWriteBytes(BQ27441_EXTENDED_CONTROL, &enableByte, 1);
}

//! TODO: id should be an enum type, limited to possible states
bool BQ27441::blockDataClass(uint8_t id)
{
	return i2cWriteBytes(BQ27441_EXTENDED_DATACLASS, &id, 1);
}

//! Should add a catch so as not to write offset out of bounds
bool BQ27441::blockDataOffset(uint8_t offset)
{
	return i2cWriteBytes(BQ27441_EXTENDED_DATABLOCK, &offset, 1);
}

uint8_t BQ27441::blockDataChecksum(void)
{
	uint8_t csum;
	i2cReadBytes(BQ27441_EXTENDED_CHECKSUM, &csum, 1);
	return csum;
}

bool BQ27441::writeExtendedData(uint8_t classID, uint8_t offset, uint8_t * data, uint8_t len)
{
	//! Add catch to make sure len isn't > 32
	//! Add catch to make sure classID is valid (should be an enum)
	bool sealFlag = sealed();
	if (sealFlag) unseal(); // Must be unsealed before making changes

	if (!enterConfig()) // Enter config mode, and wait for CFGUPDATE flag to set
		return false; // If enter config mode time's out, return fail

	if (!blockDataControl()) // // enable block data memory control
		return false; // Return false if enable fails
	
	if (!blockDataClass(BQ27441_ID_STATE)) // Write class ID using DataBlockClass()
		return false;
	
	blockDataOffset(offset / 32); // Write 32-bit block offset (usually 0)
	computeBlockChecksum(); // Compute checksum going in
	uint8_t oldCsum = blockDataChecksum();

	// Write data bytes:
	for (int i = 0; i < len; i++)
	{
		// Write to offset, mod 32 if offset is greater than 32
		// The blockDataOffset above sets the 32-bit block
		writeBlockData((offset % 32) + i, data[i]);
	}
	
	// Write new checksum using BlockDataChecksum (0x60)
	uint8_t newCsum = computeBlockChecksum(); // Compute the new checksum
	writeBlockChecksum(newCsum);

	exitConfig(); // Exit config mode by sending a soft_reset
	if (sealFlag) seal(); // Seal back up if we IC was sealed coming in
}

uint8_t BQ27441::readBlockData(uint8_t offset)
{
	uint8_t ret;
	uint8_t address = offset + BQ27441_EXTENDED_BLOCKDATA;
	i2cReadBytes(address, &ret, 1);
	return ret;
}

bool BQ27441::writeBlockData(uint8_t offset, uint8_t data)
{
	uint8_t address = offset + BQ27441_EXTENDED_BLOCKDATA;
	return i2cWriteBytes(address, &data, 1);
}

uint8_t BQ27441::computeBlockChecksum(void)
{
	uint8_t data[32];
	i2cReadBytes(BQ27441_EXTENDED_BLOCKDATA, data, 32);

	uint8_t csum = 0;
	for (int i=0; i<32; i++)
	{
		csum += data[i];
	}
	csum = 255 - csum;
	return csum;
}

bool BQ27441::writeBlockChecksum(uint8_t csum)
{
	return i2cWriteBytes(BQ27441_EXTENDED_CHECKSUM, &csum, 1);	
}

float BQ27441::convertVoltage(uint16_t rawVoltage)
{
	return ((float)rawVoltage) / 1000.0;
}

/***************************** Private Functions *****************************/

uint16_t BQ27441::readWord(uint16_t subAddress)
{
	uint8_t data[2];
	i2cReadBytes(subAddress, data, 2);
	return ((uint16_t) data[1] << 8) | data[0];
}

uint16_t BQ27441::readControlWord(uint16_t function)
{
	uint8_t subCommandMSB = (function >> 8);
	uint8_t subCommandLSB = (function & 0x00FF);
	uint8_t command[2] = {subCommandLSB, subCommandMSB};
	uint8_t data[2] = {0, 0};
	
	i2cWriteBytes((uint8_t) 0, command, 2);
	
	if (i2cReadBytes((uint8_t) 0, data, 2))
	{
		return ((uint16_t)data[1] << 8) | data[0];
	}
	
	return false;
}

bool BQ27441::executeControlWord(uint16_t function)
{
	uint8_t subCommandMSB = (function >> 8);
	uint8_t subCommandLSB = (function & 0x00FF);
	uint8_t command[2] = {subCommandLSB, subCommandMSB};
	uint8_t data[2] = {0, 0};
	
	if (i2cWriteBytes((uint8_t) 0, command, 2))
		return true;
	
	return false;
}

int16_t BQ27441::i2cReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	int16_t timeout = BQ72441_I2C_TIMEOUT;	
	//monitor.println("= Reading " + String(count) + " bytes from 0x" + String(subAddress, HEX));
	Wire.beginTransmission(_deviceAddress);
	Wire.write(subAddress);
	Wire.endTransmission(true);
	
	Wire.requestFrom(_deviceAddress, count);
	while ((Wire.available() < count) && timeout--)
		delay(1);
	if (timeout)
	{
		for (int i=0; i<count; i++)
		{
			dest[i] = Wire.read();
			//monitor.println("|== " + String(i) + ": 0x" + String(dest[i], HEX));
		}
	}
	
	return timeout;
}

uint16_t BQ27441::i2cWriteBytes(uint8_t subAddress, uint8_t * src, uint8_t count)
{
	//monitor.println("= Writing to 0x" + String(subAddress, HEX));
	Wire.beginTransmission(_deviceAddress);
	Wire.write(subAddress);
	for (int i=0; i<count; i++)
	{
		Wire.write(src[i]);
		//monitor.println("|== 0x" + String(src[i], HEX));
	}	
	Wire.endTransmission(true);
	
	return true;	
}

BQ27441 lipo;