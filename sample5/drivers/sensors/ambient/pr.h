/*
 *  Pressure sensor.
 *
 *  Created on: Mar 10, 2015
 *      Author: João Oliveira
 */

#ifndef PR_H_
#define PR_H_

#include "ambient_service_config.h"
#include "board_config.h"

#if PR_ENABLED == 1

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

#if PR_DEBUG
#define pr_printf RTT_PRINTF
#else
#define pr_printf RTT_NOP
#endif

typedef struct
{
	ble_ambient_t *m_amb;

	uint64_t ticks; //Ticks of base timer
	uint64_t timer_count;

	bool    IS_PR_ENABLED;
} pr_t;


pr_t m_pr;


uint32_t pr_init(ble_ambient_t *m_amb_init);

uint32_t pr_configs_update();

uint32_t pr_timer_handler();

uint32_t pr_values_handler();

uint32_t pr_reset_configs();

#endif /* PR_ENABLED */

#endif /* PR_H_ */
