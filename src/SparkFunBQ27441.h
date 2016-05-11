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

typedef enum {
	AVG,
	STBY,
	MAX
} current_measure;

typedef enum {
	REMAIN,
	FULL,
	AVAIL,
	AVAIL_FULL,
	REMAIN_F,
	REMAIN_UF,
	FULL_F,
	FULL_UF,
	DESIGN
} capacity_measure;

typedef enum {
	FILTERED,
	UNFILTERED
} soc_measure;

typedef enum {
	PERCENT,
	SOH_STAT
} soh_measure;

typedef enum {
	BATTERY,
	INTERNAL_TEMP
} temp_measure;

typedef enum {
	SOC_INT,
	BAT_LOW
} gpout_function;

class BQ27441 {
public:
	//////////////////////////////
	// Initialization Functions //
	//////////////////////////////
	BQ27441();
	bool begin(void);
	bool setCapacity(uint16_t capacity);
	
	/////////////////////////////
	// Battery Characteristics //
	/////////////////////////////
	uint16_t voltage(void);
	int16_t current(current_measure type = AVG);
	uint16_t capacity(capacity_measure type = REMAIN);
	int16_t power(void);
	uint16_t soc(soc_measure type = FILTERED);
	uint8_t soh(soh_measure type = PERCENT);
	uint16_t temperature(temp_measure type = BATTERY);
	
	////////////////////////////	
	// GPOUT Control Commands //
	////////////////////////////
	bool GPOUTPolarity(void);
	bool setGPOUTPolarity(bool activeHigh);
	bool GPOUTFunction(void);
	bool setGPOUTFunction(gpout_function function);
	uint8_t SOC1SetThreshold(void);
	uint8_t SOC1ClearThreshold(void);
	bool setSOC1Thresholds(uint8_t set, uint8_t clear);
	uint8_t SOCFSetThreshold(void);
	uint8_t SOCFClearThreshold(void);
	bool setSOCFThresholds(uint8_t set, uint8_t clear);
	uint8_t sociDelta(void);
	bool setSOCIDelta(uint8_t delta);
	bool pulseGPOUT(void);
	
	//////////////////////////
	// Control Sub-commands //
	//////////////////////////
	uint16_t deviceType(void);
	
	bool enterConfig(bool userControl = true);
	bool exitConfig(bool resim = true);
	uint16_t flags(void);
	uint16_t status(void);
	
private:
	uint8_t _deviceAddress;
	bool _sealFlag;
	bool _userConfigControl;
	
	bool sealed(void);
	bool seal(void);
	bool unseal(void);
		
	uint16_t opConfig(void);
	bool writeOpConfig(uint16_t value);
	
	bool softReset(void);
		
	uint16_t readWord(uint16_t subAddress);
	uint16_t readControlWord(uint16_t function);
	bool executeControlWord(uint16_t function);
	
	bool blockDataControl(void);
	bool blockDataClass(uint8_t id);
	bool blockDataOffset(uint8_t offset);
	uint8_t blockDataChecksum(void);
	uint8_t readBlockData(uint8_t offset);
	bool writeBlockData(uint8_t offset, uint8_t data);
	uint8_t computeBlockChecksum(void);
	bool writeBlockChecksum(uint8_t csum);
	
	uint8_t readExtendedData(uint8_t classID, uint8_t offset);
	bool writeExtendedData(uint8_t classID, uint8_t offset, uint8_t * data, uint8_t len);
	
	int16_t i2cReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);
	uint16_t i2cWriteBytes(uint8_t subAddress, uint8_t * src, uint8_t count);
};

extern BQ27441 lipo;

#endif
