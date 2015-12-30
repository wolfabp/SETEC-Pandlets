/*
 *  Humidity soil sensor.
 *
 *  Created on: Abr 17, 2015
 *      Author: João Oliveira
 */

#ifndef HUMSOLO_H_
#define HUMSOLO_H_

#include "ambient_service_config.h"
#include "board_config.h"

#if HUMSOLO_ENABLED

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

#if HUMSOLO_DEBUG
#define humsolo_printf RTT_PRINTF
#else
#define humsolo_printf RTT_NOP
#endif

typedef struct
{
	ble_ambient_t *m_amb;

	uint64_t ticks; //Ticks of base timer
	uint64_t timer_count;

	bool    IS_HUMSOLO_ENABLED;
} humsolo_t;


humsolo_t m_humsolo;


uint32_t humsolo_init(ble_ambient_t *m_amb_init);

uint32_t humsolo_configs_update();

uint32_t humsolo_timer_handler();

uint32_t humsolo_values_handler();

uint32_t humsolo_reset_configs();

#endif /* HUMSOLO_ENABLED */

#endif /* HUMSOLO_H_ */
