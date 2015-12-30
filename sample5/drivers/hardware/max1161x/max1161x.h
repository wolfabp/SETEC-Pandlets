#ifndef MAX1161X_H__
#define MAX1161X_H__

#include <stdint.h>
#include <stdbool.h>
#include "nrf_delay.h"
#include "twi_master.h"
#include "twi_master_config.h"

/* Error codes includes */
#include "general_error_codes.h"

uint32_t max1161x_init(uint8_t device_address);

uint32_t max1161x_read_channel(uint8_t ch, uint8_t * buffer);

#endif /* MAX1161X_H__ */
