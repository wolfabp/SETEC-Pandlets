/*
 * max17048.h
 *
 *  Created on: Mar 5, 2015
 *      Author: João Oliveira
 */

#ifndef MAX17048_H_
#define MAX17048_H_

#include <stdint.h>
#include <stdbool.h>
#include "twi_master.h"
#include "twi_master_config.h"
#include "nrf_delay.h"

/* Error codes includes */
#include "general_error_codes.h"

//Registers
#define REG_VCELL        			 0x02
#define REG_SOC          			 0x04
#define REG_MODE       				 0x06
#define REG_VERSION      			 0x08
#define REG_HIBRT        			 0x0A
#define REG_CONFIG       			 0x0C
#define REG_VALRT        			 0x14
#define REG_CRATE        			 0x16
#define REG_VRESET_ID    			 0x18
#define REG_STATUS       			 0x1A
#define REG_TABLE        			 0x40
#define REG_CMD          			 0xFE

#define CHARGE_VOLT_MEASURES         0x05 //Number of measures to check for charging state

#define MAX17048_BAT_FULL            0x5F //represents 95%, the upper byte least-significant bit has units of 1%.
#define MAX17048_BAT_EMPTY           0x0F //represents 15%,  the upper byte least-significant bit has units of 1%.

typedef enum {
	RESET,
	VOLTAGE_HIGH,
	VOLTAGE_LOW,
	VOLTAGE_RESET,
	SOC_LOW,
	SOC_CHANGE
} gauge_alert;


uint32_t max17048_init(uint8_t device_address);

uint32_t max17048_read_soc(uint8_t *buf);

uint32_t max17048_read_volt(uint8_t *buf);

bool max17048_is_charging();

uint32_t max17048_get_alert(gauge_alert *alert);

#endif /* MAX17048_H_ */
