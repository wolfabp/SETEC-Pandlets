/*
 *  Luminosity sensor.
 *
 *  Created on: Abr 17, 2015
 *      Author: João Oliveira
 */

#ifndef LUM_H_
#define LUM_H_

#include "ambient_service_config.h"
#include "board_config.h"

#if LUM_ENABLED

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/* Utilities includes */
#include "utils.h"

#include "bme280.h"              //BME280 device driver
#include <SparkFunMS1.h>
#include <SparkFunTSL2561.h>

/* Error codes includes */
#include "general_error_codes.h"

/* Service includes */
#include "ble_ambient.h"

#if LUM_DEBUG
#define lum_printf RTT_PRINTF
#else
#define lum_printf RTT_NOP
#endif

typedef struct
{
	ble_ambient_t *m_amb;

	uint64_t ticks; //Ticks of base timer
	uint64_t timer_count;

	bool    IS_LUM_ENABLED;
} lum_t;


lum_t m_lum;


uint32_t lum_init(ble_ambient_t *m_amb_init);

uint32_t lum_configs_update();

uint32_t lum_timer_handler();

uint32_t lum_values_handler();

uint32_t lum_reset_configs();

#endif /* LUM_ENABLED */

#endif /* LUM_H_ */
