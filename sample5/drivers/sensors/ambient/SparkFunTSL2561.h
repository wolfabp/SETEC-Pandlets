/*
SFE_TSL2561 illumination sensor library for Arduino
Mike Grusin, SparkFun Electronics

This library provides functions to access the TAOS TSL2561
Illumination Sensor.

Our example code uses the "beerware" license. You can do anything
you like with this code. No really, anything. If you find it useful,
buy me a beer someday.

version 1.0 2013/09/20 MDG initial version
Updated to Arduino 1.6.4 5/2015
*/

#ifndef SparkFunTSL2561_h
#define SparkFunTSL2561_h
#include <stdbool.h>
#include <stdint.h>



uint32_t SparkFunTSL2561_init(void);

uint32_t SparkFunTSL2561_bring_the_light(uint32_t *lux);
void SFE_TSL2561_init(void);
	// SFE_TSL2561 object
	
bool SFE_TSL2561_begin(void);
	// Initialize TSL2561 library with default address (0x39)
	// Always returns true

bool SFE_TSL2561_setPowerUp(void);
	// Turn on TSL2561, begin integration
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)

bool SFE_TSL2561_setPowerDown(void);
	// Turn off TSL2561
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)

//bool SFE_TSL2561_setTiming(bool gain, unsigned char time);
	// If gain = false (0), device is set to low gain (1X)
	// If gain = high (1), device is set to high gain (16X)
	// If time = 0, integration will be 13.7ms
	// If time = 1, integration will be 101ms
	// If time = 2, integration will be 402ms
	// If time = 3, use manual start / stop
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)

bool SFE_TSL2561_setTiming(bool gain, unsigned char time, uint32_t *ms);
	// Identical to above command, except ms is set to selected integration time
	// If gain = false (0), device is set to low gain (1X)
	// If gain = high (1), device is set to high gain (16X)
	// If time = 0, integration will be 13.7ms
	// If time = 1, integration will be 101ms
	// If time = 2, integration will be 402ms
	// If time = 3, use manual start / stop (ms = 0)
	// ms will be set to requested integration time
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)

bool SFE_TSL2561_manualStart(void);
	// Starts a manual integration period
	// After running this command, you must manually stop integration with manualStop()
	// Internally sets integration time to 3 for manual integration (gain is unchanged)
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)

bool SFE_TSL2561_manualStop(void);
	// Stops a manual integration period
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)

bool SFE_TSL2561_getData(uint8_t *CH0, uint8_t *CH1);
	// Retrieve raw integration results
	// data0 and data1 will be set to integration results
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
	
bool SFE_TSL2561_getLux(unsigned char gain, uint32_t ms, unsigned int CH0, unsigned int CH1, uint32_t *lux);
	// Convert raw data to lux
	// gain: 0 (1X) or 1 (16X), see setTiming()
	// ms: integration time in ms, from setTiming() or from manual integration
	// CH0, CH1: results from getData()
	// lux will be set to resulting lux calculation
	// returns true (1) if calculation was successful
	// RETURNS false (0) AND lux = 0.0 IF EITHER SENSOR WAS SATURATED (0XFFFF)

bool SFE_TSL2561_setInterruptControl(unsigned char control, unsigned char persist);
	// Sets up interrupt operations
	// If control = 0, interrupt output disabled
	// If control = 1, use level interrupt, see setInterruptThreshold()
	// If persist = 0, every integration cycle generates an interrupt
	// If persist = 1, any value outside of threshold generates an interrupt
	// If persist = 2 to 15, value must be outside of threshold for 2 to 15 integration cycles
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)

bool SFE_TSL2561_setInterruptThreshold(unsigned int low, unsigned int high);
	// Set interrupt thresholds (channel 0 only)
	// low, high: 16-bit threshold values
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)

bool SFE_TSL2561_clearInterrupt(void);
	// Clears an active interrupt
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)

bool SFE_TSL2561_getID(unsigned char *ID);
	// Retrieves part and revision code from TSL2561
	// Sets ID to part ID (see datasheet)
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
	
char SFE_TSL2561_getError(void);
	// If any library command fails, you can retrieve an extended
	// error code using this command. Errors are from the wire library: 
	// 0 = Success
	// 1 = Data too long to fit in transmit buffer
	// 2 = Received NACK on transmit of address
	// 3 = Received NACK on transmit of data
	// 4 = Other error

bool SFE_TSL2561_readByte(unsigned char address, unsigned char *value);
	// Reads a byte from a TSL2561 address
	// Address: TSL2561 address (0 to 15)
	// Value will be set to stored byte
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() above)

bool SFE_TSL2561_writeByte(unsigned char address, unsigned char value);
	// Write a byte to a TSL2561 address
	// Address: TSL2561 address (0 to 15)
	// Value: byte to write to address
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() above)

bool SFE_TSL2561_readUInt(unsigned char address, uint8_t *value);
	// Reads an unsigned integer (16 bits) from a TSL2561 address (low byte first)
	// Address: TSL2561 address (0 to 15), low byte first
	// Value will be set to stored unsigned integer
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() above)

bool SFE_TSL2561_writeUInt(unsigned char address, unsigned int value);
	// Write an unsigned integer (16 bits) to a TSL2561 address (low byte first)
	// Address: TSL2561 address (0 to 15), low byte first
	// Value: unsigned int to write to address
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() above)
	
char _i2c_address;
unsigned char _error;



#endif
