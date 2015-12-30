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

#include <SparkFunTSL2561.h>
#include "twi_master.h"
#include <stdbool.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "init.h"
#include "general_error_codes.h"
#include <stdlib.h>

char _i2c_address;
unsigned char ID, time=0;
uint8_t error=0,gain, data0, data1, good=0;
uint32_t  ms=1000;
unsigned char _error;
uint8_t TSL2561_ADDR_0 = 0x29; // address with '0' shorted on board
uint8_t TSL2561_ADDR   = 0x39; // default address
uint8_t TSL2561_ADDR_1 = 0x49; // address with '1' shorted on board

// TSL2561 registers

uint8_t TSL2561_CMD           = 0x80;
uint8_t TSL2561_CMD_CLEAR     = 0xC0;
uint8_t	TSL2561_REG_CONTROL   = 0x00;
uint8_t	TSL2561_REG_TIMING    = 0x01;
uint8_t	TSL2561_REG_THRESH_L  = 0x02;
uint8_t	TSL2561_REG_THRESH_H  = 0x04;
uint8_t	TSL2561_REG_INTCTL    = 0x06;
uint8_t	TSL2561_REG_ID        = 0x0A;
uint8_t	TSL2561_REG_DATA_0    = 0x0C;
uint8_t	TSL2561_REG_DATA_1    = 0x0E;

void SFE_TSL2561_init(void)
	// SFE_TSL2561 object
{
}

bool begin(char i2c_address)
	// Initialize TSL2561 library to arbitrary address or:
	// TSL2561_ADDR_0 (0x29 address with '0' shorted on board)
	// TSL2561_ADDR   (0x39 default address)
	// TSL2561_ADDR_1 (0x49 address with '1' shorted on board)
	// Always returns true
{
	_i2c_address = i2c_address;
	twi_master_init();
	return(true);
}

uint32_t SparkFunTSL2561_init(){
	printf("SparkFunTSL2561_init\n");
	gain = 0;

	// Enter main loop: always returns true	
	SFE_TSL2561_begin();

	nrf_delay_ms(5);
	
	//Try to get the id
	if (!SFE_TSL2561_getID(&ID)){
		printf("Error getID\n");
		return SparkFunTSL2561_INIT_NOK_ID;
	}

	nrf_delay_ms(5);

	// To start taking measurements, power up the sensor:
	if(!SFE_TSL2561_setPowerUp()){
		printf("Erro powerup\n");
		return SparkFunTSL2561_INIT_NOK_PW;
	}	
	
	nrf_delay_ms(5);
	// Set the timing to the sensor:
	if(!SFE_TSL2561_setTiming(gain,time, &ms)){
		printf("Erro setTiming\n");
		return SparkFunTSL2561_INIT_NOK_ST;
		}
	nrf_delay_ms(5);

	return NRF_SUCCESS;
}


uint32_t SparkFunTSL2561_bring_the_light(uint32_t *lux){

	data0=data1=0;
	printf("SparkFunTSL2561_bring_the_light\n");
	if (!SFE_TSL2561_getData(&data0,&data1))
	{
		printf("ERRO DATA NOK\n");
		return SparkFunTSL2561_DATA_NOK;
	}

	good = SFE_TSL2561_getLux(gain,ms,data0,data1,lux);
	return NRF_SUCCESS;
}

bool SFE_TSL2561_begin(void)
	// Initialize TSL2561 library with default address (0x39)
	// Always returns true
{
	return(begin(TSL2561_ADDR));
}



bool SFE_TSL2561_setPowerUp(void)
	// Turn on TSL2561, begin integrations
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
{
	// Write 0x03 to command byte (power on)
	return(SFE_TSL2561_writeByte(TSL2561_REG_CONTROL,0x03));
}


bool SFE_TSL2561_setPowerDown(void)
	// Turn off TSL2561
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
{
	// Clear command byte (power off)
	return(SFE_TSL2561_writeByte(TSL2561_REG_CONTROL,0x00));
}


bool setTiming(bool gain, unsigned char time)
	// If gain = false (0), device is set to low gain (1X)
	// If gain = high (1), device is set to high gain (16X)
	// If time = 0, integration will be 13.7ms
	// If time = 1, integration will be 101ms
	// If time = 2, integration will be 402ms
	// If time = 3, use manual start / stop
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
{
	unsigned char timing;

	// Get timing byte
	if (SFE_TSL2561_readByte(TSL2561_REG_TIMING,&timing))
	{	
		//printf("timing %d",timing);
		// Set gain (0 or 1)
		if (gain)
			timing |= 0x10;
		else
			timing &= ~0x10;

		// Set integration time (0 to 3)
		timing &= ~0x03;
		timing |= (time & 0x03);

		// Write modified timing byte back to device
		nrf_delay_ms(5);
		if (SFE_TSL2561_writeByte(TSL2561_REG_TIMING,timing))
			return true;
	}
	return(false);
}


bool SFE_TSL2561_setTiming(bool gain, unsigned char time, uint32_t *ms)
	// If gain = false (0), device is set to low gain (1X)
	// If gain = high (1), device is set to high gain (16X)
	// If time = 0, integration will be 13.7ms
	// If time = 1, integration will be 101ms
	// If time = 2, integration will be 402ms
	// If time = 3, use manual start / stop (ms = 0)
	// ms will be set to integration time
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
{
	// Calculate ms for user
	switch (time)
	{
		case 0: (*ms) = 14; break;
		case 1: (*ms) = 101; break;
		case 2: (*ms) = 402; break;
		default: (*ms) = 0;
	}
	// Set integration using base function
	//printf("MS:= %d", (*ms));
	return(setTiming(gain,time));
}


bool SFE_TSL2561_manualStart(void)
	// Starts a manual integration period
	// After running this command, you must manually stop integration with manualStop()
	// Internally sets integration time to 3 for manual integration (gain is unchanged)
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
{
	unsigned char timing;
	
	// Get timing byte
	if (SFE_TSL2561_readByte(TSL2561_REG_TIMING,&timing))
	{
		// Set integration time to 3 (manual integration)
		timing |= 0x03;

		if (SFE_TSL2561_writeByte(TSL2561_REG_TIMING,timing))
		{
			// Begin manual integration
			timing |= 0x08;

			// Write modified timing byte back to device
			if (SFE_TSL2561_writeByte(TSL2561_REG_TIMING,timing))
				return(true);
		}
	}
	return(false);
}


bool SFE_TSL2561_manualStop(void)
	// Stops a manual integration period
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
{
	unsigned char timing;
	
	// Get timing byte
	if (SFE_TSL2561_readByte(TSL2561_REG_TIMING,&timing))
	{
		// Stop manual integration
		timing &= ~0x08;

		// Write modified timing byte back to device
		if (SFE_TSL2561_writeByte(TSL2561_REG_TIMING,timing))
			return(true);
	}
	return(false);
}


bool SFE_TSL2561_getData(uint8_t *data0, uint8_t *data1)
	// Retrieve raw integration results
	// data0 and data1 will be set to integration results
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
{
	// Get data0 and data1 out of result registers

	if (SFE_TSL2561_readUInt(TSL2561_REG_DATA_0,(data0)) && SFE_TSL2561_readUInt(TSL2561_REG_DATA_1,(data1))) {
		
		return(true);
	}
	
	return(false);
}


bool SFE_TSL2561_getLux(unsigned char gain, uint32_t ms, unsigned int CH0, unsigned int CH1, uint32_t *lux)
	// Convert raw data to lux
	// gain: 0 (1X) or 1 (16X), see setTiming()
	// ms: integration time in ms, from setTiming() or from manual integration
	// CH0, CH1: results from getData()
	// lux will be set to resulting lux calculation
	// returns true (1) if calculation was successful
	// RETURNS false (0) AND lux = 0.0 IF EITHER SENSOR WAS SATURATED (0XFFFF)
{
	uint32_t ratio, d0, d1;
	//printf("ch0:=%d\n",(int)CH0);
	//printf(" ch1:=%d\n",(int)CH1);
	// Determine if either sensor saturated (0xFFFF)
	// If so, abandon ship (calculation will not be accurate)
	if ((CH0 == 0xFFFF) || (CH1 == 0xFFFF))
	{
		(*lux) = 0;
		return(false);
	}

	// Convert from unsigned integer to floating point
	d0 = (uint32_t)CH0; d1 = (uint32_t)CH1;

	// We will need the ratio for subsequent calculations
	ratio = (uint32_t)d1 / (uint32_t)d0 ;

	// Normalize for integration time
	d0 *= (402.0/ms);
	d1 *= (402.0/ms);

	// Normalize for gain
	if (!gain)
	{
		d0 *= 16;
		d1 *= 16;
	}

	// Determine lux per datasheet equations:
	if (ratio < 0.5)
	{
		(*lux) = (uint32_t)(0.0304 * d0 - 0.062 * d0 * pow(ratio,1.4));
		printf("ratio<0.5\n");
		return(true);
	}

	if (ratio < 0.61)
	{
		(*lux) = (uint32_t)(0.0224 * d0 - 0.031 * d1);
		printf("ratio<0.61\n");
		return(true);
	}

	if (ratio < 0.80)
	{
		(*lux) =(uint32_t)(0.0128 * d0 - 0.0153 * d1);
		printf("ratio<0.8\n");		
		return(true);
	}

	if (ratio < 1.30)
	{
		(*lux) = (uint32_t)(0.00146 * d0 - 0.00112 * d1);
		printf("ratio<1.3\n");
		return(true);
	}
	
	printf("ratio>1.3\n");
	(*lux) = 0.0;
	return(true);
}


bool SFE_TSL2561_setInterruptControl(unsigned char control, unsigned char persist)
		// Sets up interrupt operations
	// If control = 0, interrupt output disabled
	// If control = 1, use level interrupt, see setInterruptThreshold()
	// If persist = 0, every integration cycle generates an interrupt
	// If persist = 1, any value outside of threshold generates an interrupt
	// If persist = 2 to 15, value must be outside of threshold for 2 to 15 integration cycles
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
{
	// Place control and persist bits into proper location in interrupt control register
	if (SFE_TSL2561_writeByte(TSL2561_REG_INTCTL,((control | 0B00000011) << 4) & (persist | 0B00001111)))
		return(true);
		
	return(false);
}


bool SFE_TSL2561_setInterruptThreshold(unsigned int low, unsigned int high)
	// Set interrupt thresholds (channel 0 only)
	// low, high: 16-bit threshold values
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
{
	// Write low and high threshold values
	if (SFE_TSL2561_writeUInt(TSL2561_REG_THRESH_L,low) && SFE_TSL2561_writeUInt(TSL2561_REG_THRESH_H,high))
		return(true);
		
	return(false);
}


bool SFE_TSL2561_clearInterrupt(void)
	// Clears an active interrupt
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
{
	// Set up command byte for interrupt clear
	/*Wire.beginTransmission(_i2c_address);
	Wire.write(TSL2561_CMD_CLEAR);
	_error = Wire.endTransmission();
	*/	

		if(!twi_master_transfer((TSL2561_ADDR << 1), &TSL2561_CMD_CLEAR, 1, TWI_ISSUE_STOP)){
			_error = -1;
			return(false);	
			}
	return(true);
}


bool SFE_TSL2561_getID(unsigned char *ID)
	// Retrieves part and revision code from TSL2561
	// Sets ID to part ID (see datasheet)
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() below)
{
	// Get ID byte from ID register
	if (SFE_TSL2561_readByte(TSL2561_REG_ID,ID))
		return(true);

	return(false);
}


char SFE_TSL2561_getError(void)
	// If any library command fails, you can retrieve an extended
	// error code using this command. Errors are from the wire library: 
	// 0 = Success
	// 1 = Data too long to fit in transmit buffer
	// 2 = Received NACK on transmit of address
	// 3 = Received NACK on transmit of data
	// 4 = Other error
{
	return(_error);
}

// Private functions:

bool SFE_TSL2561_readByte(unsigned char address, unsigned char *value)
	// Reads a byte from a TSL2561 address
	// Address: TSL2561 address (0 to 15)
	// Value will be set to stored byte
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() above)
{
	// Set up command byte for read
	/*Wire.beginTransmission(_i2c_address);
	Wire.write((address & 0x0F) | TSL2561_CMD);
	_error = Wire.endTransmission();*/
	uint8_t bam=(address & 0x0F) | TSL2561_CMD;
	if(!twi_master_transfer((_i2c_address << 1), &bam, 1, TWI_ISSUE_STOP)){
			_error = -1;
			//printf("Erro SFE_TSL2561_readByte init transfer\n");
			return(false);
		}


	// Read requested byte
	
	if(!twi_master_transfer(((_i2c_address << 1) | TWI_READ_BIT), value, 1, TWI_ISSUE_STOP)){
		_error = -1;
		//printf("Erro SFE_TSL2561_readByte read\n");
		return(false);
		}
return(true);
	
}


bool SFE_TSL2561_writeByte(unsigned char address, unsigned char value)
	// Write a byte to a TSL2561 address
	// Address: TSL2561 address (0 to 15)
	// Value: byte to write to address
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() above)
{
	// Set up command byte for write
	/*Wire.beginTransmission(_i2c_address);
	Wire.write((address & 0x0F) | TSL2561_CMD);
	// Write byte
	Wire.write(value);
	_error = Wire.endTransmission();
	if (_error == 0)
		return(true);*/
	uint8_t bam[2];
		bam[0] = (address & 0x0F) | TSL2561_CMD;
		bam[1] = value;	
	if(!twi_master_transfer((_i2c_address << 1), bam, 2, TWI_ISSUE_STOP)){
			_error = -1;
			printf("error_writeByte\n");
			return(false);
	}

	return(true);
}


bool SFE_TSL2561_readUInt(unsigned char address, uint8_t (*value))
	// Reads an unsigned integer (16 bits) from a TSL2561 address (low byte first)
	// Address: TSL2561 address (0 to 15), low byte first
	// Value will be set to stored unsigned integer
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() above)
{

	// Set up command byte for read

	/*Wire.beginTransmission(_i2c_address);
	Wire.write((address & 0x0F) | TSL2561_CMD);
	// Write byte
	Wire.write(value);
	_error = Wire.endTransmission();
	if (_error == -1)
		return(false);*/
	uint8_t bam = (address & 0x0F) | TSL2561_CMD;
	
	if(!twi_master_transfer((TSL2561_ADDR << 1), &bam, 1, TWI_ISSUE_STOP)){
			_error = -1;
			return(false);
	}
	_error = 0;
	nrf_delay_ms(20);
	if(!twi_master_transfer(((TSL2561_ADDR << 1) | TWI_READ_BIT), (value), 2, TWI_ISSUE_STOP)){
			_error = -1;
			return(false);
	}

	/*nrf_delay_ms(20);	
	if(!twi_master_transfer(((TSL2561_ADDR << 1) | TWI_READ_BIT), (value), 1, TWI_ISSUE_STOP)){
			_error = -1;
			return(false);
	}
	*/

	return(true);
}


bool SFE_TSL2561_writeUInt(unsigned char address, unsigned int value)
	// Write an unsigned integer (16 bits) to a TSL2561 address (low byte first)
	// Address: TSL2561 address (0 to 15), low byte first
	// Value: unsigned int to write to address
	// Returns true (1) if successful, false (0) if there was an I2C error
	// (Also see getError() above)
{
	// Split int into lower and upper bytes, write each byte
	unsigned char low ,high;

	low = value & 0x00FF;
	high = value & 0xFF00;
	if (SFE_TSL2561_writeByte(address,low) 
		&& SFE_TSL2561_writeByte(address + 1,high))
		return(true);

	return(false);
}
