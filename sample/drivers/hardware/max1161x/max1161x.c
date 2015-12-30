#include "max1161x.h"

static uint8_t       m_device_address;          // !< Device address in bits [7:1]

uint32_t max1161x_write_data(uint8_t * command, uint8_t number_of_bytes){
	if(!twi_master_transfer(m_device_address, command, number_of_bytes, TWI_ISSUE_STOP)){
		return MAX1161X_WRITE_FAILED;
	}

	return NRF_SUCCESS;
}

uint32_t max1161x_read_data(uint8_t * destination, uint8_t number_of_bytes){
	if(!twi_master_transfer(m_device_address|TWI_READ_BIT, destination, number_of_bytes, TWI_ISSUE_STOP)){
		return MAX1161X_READ_FAILED;
	}

	nrf_delay_us(TWI_MASTER_READ_DELAY);

	return NRF_SUCCESS;
}


uint32_t max1161x_init(uint8_t device_address){

	m_device_address = (device_address << 1);

	// Bit 7 (REG) = set setup byte (1)
	// Bit 6, 5, 4 (SEL 2, 1, 0) = set VDD as reference and REF pin as analog input (000)
	// Bit 3 (CLK) = set internal clock (0)
	// Bit 2 (BIP/UNO) = set output as unipolar (0)
	// Bit 1 (RST) = set reset to no action (1)
	// Bit 0 (X) = don't care
	uint8_t setup_packet = 0b10000010;

	if(max1161x_write_data(&setup_packet, 1) != NRF_SUCCESS)
		return MAX1161X_INIT_FAILED;

	return NRF_SUCCESS;
}


uint32_t max1161x_read_channel(uint8_t ch, uint8_t * buffer){
	// Buffer must always be 2 bytes long

	if ((ch > 11) || (ch < 0))
		return MAX1161X_INVALID_CHANNEL;

	// Bit 7 (REG) = set configuration byte (0)
	// Bit 6, 5 (SCAN 1, 0) = set to read only one channel (11)
	// Bit 4, 3, 2, 1 (CS) = select channel (mapping is 0000 for ch0, 0001 for ch1, etc)
	// Bit 0 (SGL/DIF) = set as single-ended (1)
	uint8_t config_packet = 0b01111111;
	uint8_t cs = ((ch << 1) | 0b11100001);

	config_packet &= cs;

	if(max1161x_write_data(&config_packet, 1) != NRF_SUCCESS){
		return MAX1161X_READ_CHANNEL_FAILED;
	}

	if(max1161x_read_data(buffer, 2) != NRF_SUCCESS){
		return MAX1161X_READ_CHANNEL_FAILED;
	}

	return NRF_SUCCESS;
}





