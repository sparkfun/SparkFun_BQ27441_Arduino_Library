/******************************************************************************
SparkFunBQ27441.h
BQ27441 Arduino Library Main Header File
Jim Lindblom @ SparkFun Electronics
May 9, 2016
https://github.com/sparkfun/SparkFun_BQ27441_Arduino_Library

Definition of the BQ27441 library, which implements all features of the
BQ27441 LiPo Fuel Gauge.

Hardware Resources:
- Arduino Development Board
- SparkFun Battery Babysitter

Development environment specifics:
Arduino 1.6.7
SparkFun Battery Babysitter v1.0
Arduino Uno (any 'duino should do)
******************************************************************************/

#ifndef SparkFunBQ27441_h
#define SparkFunBQ27441_h

#include "Arduino.h"
#include "BQ27441_Definitions.h"

#define BQ72441_I2C_TIMEOUT 2000

class BQ27441 {
public:
	BQ27441();
	bool begin(void);
	bool setCapacity(uint16_t capacity);
	
	uint16_t temperature(void);
	uint16_t voltage(void);
	int16_t current(void);
	uint16_t capacity(void);
	
	uint16_t nominalAvailableCapacity(void);
	uint16_t fullAvailableCapacity(void);
	uint16_t remainingCapacity(void);
	uint16_t fullChargeCapacity(void);
	
	int16_t averageCurrent(void);
	int16_t standbyCurrent(void);
	int16_t maxLoadCurrent(void);
	int16_t averagePower(void);
	
	uint16_t soc(void);
	uint16_t soh(void);
	
	uint16_t internalTemperature(void);
	
	uint16_t remainingCapacityUnfilitered(void);
	uint16_t remainingCapacityFiltered(void);
	uint16_t fullChargeCapacityUnfiltered(void);
	uint16_t fullChargeCapacityFiltered(void);
	uint16_t socUnfiltered(void);
	
	uint16_t flags(void);
	
	//////////////////////////
	// Control Sub-commands //
	//////////////////////////
	uint16_t status(void);
	uint16_t deviceType(void);
	
	bool sealed(void);
	bool seal(void);
	bool unseal(void);
	
	bool enterConfig(void);
	bool exitConfig(bool resim = true);
	bool softReset(void);
	
	uint16_t readOpConfig(void);
	uint16_t readDesignCapacity(void);
	
	bool blockDataControl(void);
	bool blockDataClass(uint8_t id);
	bool blockDataOffset(uint8_t offset);
	uint8_t blockDataChecksum(void);
	uint8_t readBlockData(uint8_t offset);
	bool writeBlockData(uint8_t offset, uint8_t data);
	uint8_t computeBlockChecksum(void);
	bool writeBlockChecksum(uint8_t csum);
	bool writeExtendedData(uint8_t classID, uint8_t offset, uint8_t * data, uint8_t len);
	
	float convertVoltage(uint16_t rawVoltage);
	
//private:
	uint8_t _deviceAddress;
	
	uint16_t readWord(uint16_t subAddress);
	uint16_t readControlWord(uint16_t function);
	bool executeControlWord(uint16_t function);
	
	int16_t i2cReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);
	uint16_t i2cWriteBytes(uint8_t subAddress, uint8_t * src, uint8_t count);
};

extern BQ27441 lipo;

#endif
