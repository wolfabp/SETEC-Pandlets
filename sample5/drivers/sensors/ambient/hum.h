/*
 *  Humidity sensor.
 *
 *  Created on: Abr 17, 2015
 *      Author: João Oliveira
 */

#ifndef HUM_H_
#define HUM_H_

#include "ambient_service_config.h"
#include "board_config.h"

#if HUM_ENABLED

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/* Utilities includes */
#include "utils.h"

#include "bme280.h"              //BME280 device driver

/* Error codes includes */
#include "general_error_codes.h"

/* Service includes */
#include "ble_ambient.h"

#if HUM_DEBUG
#define hum_printf RTT_PRINTF
#else
#define hum_printf RTT_NOP
#endif

typedef struct
{
	ble_ambient_t *m_amb;

	uint64_t ticks; //Ticks of base timer
	uint64_t timer_count;

	bool    IS_HUM_ENABLED;
} hum_t;


hum_t m_hum;


uint32_t hum_init(ble_ambient_t *m_amb_init);

uint32_t hum_configs_update();

uint32_t hum_timer_handler();

uint32_t hum_values_handler();

uint32_t hum_reset_configs();

#endif /* HUM_ENABLED */

#endif /* HUM_H_ */
