#ifndef PCA9538_H__
#define PCA9538_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "twi_master.h"
#include "twi_master_config.h"
#include "app_error.h"

// function return errors
#define PCA9538_READ_INPUT_PORT_FAILED			0x4001
#define PCA9538_READ_INPUT_CHANNEL_FAILED		0x4002
#define PCA9538_WRITE_OUTPUT_CHANNEL_FAILED		0x4003
#define PCA9538_READ_OUTPUT_CHANNEL_FAILED		0x4004
#define PCA9538_WRITE_CONFIG_CHANNEL_FAILED		0x0005
#define PCA9538_ALREADY_INIT					0x4006
#define PCA9538_INIT_FAILED						0x4007
#define PCA9538_CHANNEL_CONFIGURED_AS_OUTPUT	0x4008
#define PCA9538_CHANNEL_CONFIGURED_AS_INPUT		0x4009

// PCA9538 errors
#define PCA9538_INVALID_CHANNEL					0x4015
#define PCA9538_WRITE_FAILED					0x4016
#define PCA9538_READ_FAILED						0x4017

// GPIO Extender Register Map
#define PCA9538_INPUT_PORT_REGISTER				0x00
#define PCA9538_OUTPUT_PORT_REGISTER			0x01
#define PCA9538_POLARITY_INV_REGISTER			0x02
#define PCA9538_CONFIGURATION_REGISTER			0x03

//
#define PCA9538_LOW							0x00
#define PCA9538_HIGH							0x01

uint32_t pca9538_init(uint8_t device_address);
uint32_t pca9538_read_input_channel(uint8_t channel, uint8_t * buffer);
uint32_t pca9538_read_input_port(uint8_t * buffer);
uint32_t pca9538_write_output_channel(uint8_t channel, bool data);
uint32_t pca9538_config_channel(uint8_t channel, bool dir);

#endif /* PCA9538_H__ */
