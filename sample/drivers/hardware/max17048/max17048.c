/*
 * max17048.c
 *
 *  Created on: Mar 5, 2015
 *      Author: João Oliveira
 */

#include "max17048.h"

static uint8_t       m_device_address;                     // !< Device address in bits [7:1]

uint32_t max17048_write_data(uint8_t register_address, uint8_t *values, uint8_t number_of_bytes){
	//Size of values plus the start register address
	uint8_t buffer[number_of_bytes + 1];
	uint8_t i;

	buffer[0] = register_address;

	for(i = 0; i < number_of_bytes; i ++)
		buffer[i + 1] = values[i];

 	if(!twi_master_transfer(m_device_address, buffer, number_of_bytes, TWI_ISSUE_STOP)){
		return MAX17048_WRITE_FAILED;
	}

	return NRF_SUCCESS;
}

uint32_t max17048_read_data(uint8_t * destination, uint8_t register_address, uint8_t number_of_bytes){
	if(!twi_master_transfer(m_device_address, &register_address, 1, TWI_ISSUE_STOP))
		return MAX17048_WRITE_FAILED;

	if(!twi_master_transfer(m_device_address|TWI_READ_BIT, destination, number_of_bytes, TWI_ISSUE_STOP)){
		return MAX17048_READ_FAILED;
	}

	nrf_delay_us(TWI_MASTER_READ_DELAY);
	return NRF_SUCCESS;
}


uint32_t max17048_clear_alert(){
	uint8_t buffer[2];

	//Clear ALRT bit
	if(max17048_read_data(buffer, REG_CONFIG, 2) != NRF_SUCCESS) //Read register configs
		return MAX17048_CLEAR_FAILED;

	buffer[1] &= 0b11011111; //Just clear ALRT bit

	if(max17048_write_data(REG_CONFIG, buffer, 2) != NRF_SUCCESS)
		return MAX17048_CLEAR_FAILED;


	//Clear all status flags
	buffer[0] = 0x00;
	buffer[1] = 0x00;

	if(max17048_write_data(REG_STATUS, buffer, 2) != NRF_SUCCESS)
		return MAX17048_CLEAR_FAILED;

	return NRF_SUCCESS;
}


uint32_t max17048_init(uint8_t device_address){

	m_device_address = (device_address << 1);

	uint8_t buffer[2];

	//Read ID register
	if(max17048_read_data(buffer, REG_VRESET_ID, 2) != NRF_SUCCESS)
		return MAX17048_INIT_FAILED;

	//Read VERSION register
	if(max17048_read_data(buffer, REG_VERSION, 2) != NRF_SUCCESS)
		return MAX17048_INIT_FAILED;

	//Configure MODE with quick-start off, sleep off
	buffer[0] = 0x00;
	buffer[1] = 0x00;

	if(max17048_write_data(REG_MODE, buffer, 2) != NRF_SUCCESS)
		return MAX17048_INIT_FAILED;

	//Configure hibernation to always on
	buffer[0] = 0x00;
	buffer[1] = 0x00;

	if(max17048_write_data(REG_HIBRT, buffer, 2) != NRF_SUCCESS)
		return MAX17048_INIT_FAILED;

	//Set battery removal threshold to 2.5V, enable analog comparator
	buffer[0] = 0b01111100;
	buffer[1] = 0xFF;

	if(max17048_write_data(REG_VRESET_ID, buffer, 2) != NRF_SUCCESS)
		return MAX17048_INIT_FAILED;

	//Enable SOC change alert
	buffer[0] = 0x97; //Keep this variable at default
	buffer[1] = 0b01011100; //Enable SOC change alert, enable SOC below 4% alert

	if(max17048_write_data(REG_CONFIG, buffer, 2) != NRF_SUCCESS)
		return MAX17048_INIT_FAILED;

	//Disable VMAX and VMIN alerts
	buffer[0] = 0x00; //Set VMIN to 0x00
	buffer[1] = 0xFF; //Set VMAX to 0xFF

	if(max17048_write_data(REG_VALRT, buffer, 2) != NRF_SUCCESS)
		return MAX17048_INIT_FAILED;

	return NRF_SUCCESS;
}


uint32_t max17048_read_volt(uint8_t *buf){

	if(max17048_read_data(buf, REG_VCELL, 2) != NRF_SUCCESS)
		return MAX17048_READ_VOLT_FAILED;

	return NRF_SUCCESS;
}


uint32_t max17048_read_soc(uint8_t *buf){

	if(max17048_read_data(buf, REG_SOC, 2) != NRF_SUCCESS)
			return MAX17048_READ_SOC_FAILED;

	return NRF_SUCCESS;
}


uint32_t max17048_get_alert(gauge_alert *alert){
	uint8_t buffer[2];

	if(max17048_read_data(buffer, REG_STATUS, 2) != NRF_SUCCESS)
		return MAX17048_READ_STATUS_FAILED;

	if(((buffer[0] & 0b00000001) >> 0))
		*alert = RESET;
	else if(((buffer[0] & 0b00000010) >> 1))
		*alert = VOLTAGE_HIGH;
	else if(((buffer[0] & 0b00000100) >> 2))
		*alert = VOLTAGE_LOW;
	else if(((buffer[0] & 0b00001000) >> 3))
		*alert = VOLTAGE_RESET;
	else if(((buffer[0] & 0b00010000) >> 4))
		*alert = SOC_LOW;
	else if(((buffer[0] & 0b00100000) >> 5)){
		*alert = SOC_CHANGE;
	}

	return max17048_clear_alert();
}



