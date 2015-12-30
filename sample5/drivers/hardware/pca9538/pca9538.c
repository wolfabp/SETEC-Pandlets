#include "pca9538.h"
#include "utils.h"
static uint8_t       m_device_address;          // !< Device address in bits [7:1]

static uint8_t channels_config;
static bool initialized = 0;

uint32_t pca9538_read(uint8_t reg, uint8_t * buffer){
	nrf_delay_us(TWI_MASTER_READ_DELAY);

	if(!twi_master_transfer(m_device_address, &reg, 1, TWI_ISSUE_STOP)){
		return PCA9538_WRITE_FAILED;
	}	
	nrf_delay_us(TWI_MASTER_READ_DELAY);
	
	if(!twi_master_transfer(m_device_address|TWI_READ_BIT, buffer, 1, TWI_ISSUE_STOP)){
		return PCA9538_READ_FAILED;
	}
	nrf_delay_us(TWI_MASTER_READ_DELAY);

	return NRF_SUCCESS;
}

uint32_t pca9538_write(uint8_t reg, uint8_t data){
	uint8_t packet[2] = {reg, data};
	
	if(!twi_master_transfer(m_device_address, packet, 2, TWI_ISSUE_STOP)){
		return PCA9538_WRITE_FAILED;
	}

	nrf_delay_us(TWI_MASTER_READ_DELAY);

	return NRF_SUCCESS;
}

/*
 * funcitons to be accessed externally
 */
uint32_t pca9538_init(uint8_t device_address){
	if (initialized) {
		return PCA9538_ALREADY_INIT;
		return PCA9538_ALREADY_INIT;
	}

	m_device_address = (device_address << 1);

	// Bit 0 .. 7 (C0...C7) = sets configuration as input for all pins
	channels_config = 0xFF;
	
	if(pca9538_write(PCA9538_CONFIGURATION_REGISTER, channels_config) != NRF_SUCCESS){
		return PCA9538_INIT_FAILED;
	}

	if(pca9538_write(PCA9538_POLARITY_INV_REGISTER, 0x00) != NRF_SUCCESS){
		return PCA9538_INIT_FAILED;
	}
	
	if(pca9538_write(PCA9538_OUTPUT_PORT_REGISTER, 0x00) != NRF_SUCCESS){
		return PCA9538_INIT_FAILED;
	}

	initialized = 1;
	return NRF_SUCCESS;
}

uint32_t pca9538_read_input_channel(uint8_t channel, uint8_t * buffer){
	// validate channel
	if(channel < 0 || channel > 7)
		return PCA9538_INVALID_CHANNEL;
		
	channel = 1 << channel;

	if(pca9538_read(PCA9538_INPUT_PORT_REGISTER, buffer) != NRF_SUCCESS){
		return PCA9538_READ_INPUT_CHANNEL_FAILED;
	}
	(*buffer) &= channel;
	
	return NRF_SUCCESS;
}

uint32_t pca9538_read_input_port(uint8_t * buffer){
	if(pca9538_read(PCA9538_INPUT_PORT_REGISTER, buffer) != NRF_SUCCESS){
		return PCA9538_READ_INPUT_PORT_FAILED;
	}

	return NRF_SUCCESS;
}

uint32_t pca9538_write_output_channel(uint8_t channel, bool data){
	// validate channel
	if(channel < 0 || channel > 7)
		return PCA9538_INVALID_CHANNEL;
		
	channel = 1 << channel;
	if((channel & channels_config) != 0)
		return PCA9538_CHANNEL_CONFIGURED_AS_INPUT;

	// read output content;
	uint8_t output_content;
	if(pca9538_read(PCA9538_OUTPUT_PORT_REGISTER, &output_content) != NRF_SUCCESS){
		return PCA9538_READ_OUTPUT_CHANNEL_FAILED;
	}

	// configure as HIGH
	if(data == PCA9538_HIGH){
		output_content |= channel;
	}
	// configure as LOW
	if(data == PCA9538_LOW){
		output_content &= ~channel;
	}

	if(pca9538_write(PCA9538_OUTPUT_PORT_REGISTER, output_content) != NRF_SUCCESS){
		return PCA9538_WRITE_OUTPUT_CHANNEL_FAILED;
	}

	return NRF_SUCCESS;
}

uint32_t pca9538_config_channel(uint8_t channel, bool dir){
	// validate channel
	if(channel < 0 || channel > 7)
		return PCA9538_INVALID_CHANNEL;

	channel = 1 << channel;

	if(dir == PCA9538_HIGH)	// configure as input
		channels_config |= channel;
	if(dir == PCA9538_LOW)		// configure as output
		channels_config &= ~channel;
	
	if(pca9538_write(PCA9538_CONFIGURATION_REGISTER, channels_config) != NRF_SUCCESS){
		return PCA9538_WRITE_CONFIG_CHANNEL_FAILED;
	}
	
	return NRF_SUCCESS;
}
