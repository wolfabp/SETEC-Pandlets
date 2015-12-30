#ifndef TEMP_H__
#define TEMP_H__

#include "ambient_service_config.h"
#include "board_config.h"

#if TEMP_ENABLED == 1

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

#if TEMP_DEBUG
#define temp_printf RTT_PRINTF
#else
#define temp_printf RTT_NOP
#endif

typedef struct
{
	ble_ambient_t *m_amb;

	uint64_t ticks; //Ticks of base timer
	uint64_t timer_count;

	bool    IS_TEMP_ENABLED;
	bool    IS_READING;
} temp_t;


temp_t m_temp;

uint32_t temp_init(ble_ambient_t *m_amb_init);

uint32_t temp_configs_update();

uint32_t temp_timer_handler();

uint32_t temp_values_handler();

uint32_t temp_reset_configs();

#endif /* TEMP_ENABLED */

#endif /* TEMP_H__ */
