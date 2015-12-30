/*
 * battery.h
 *
 *  Created on: Mar 4, 2015
 *      Author: João Oliveira
 */

#ifndef BATTERY_H_
#define BATTERY_H_

#include <string.h>

#include "ambient_service_config.h"
#include "board_config.h"
#include "nrf_error.h"
#include "app_error.h"
#include "nordic_common.h"

#include "max17048.h"
#include "utils.h"

#include "fhp_gpio.h"

bool low_battery;
bool high_voltage_on;

uint32_t battery_init();

void battery_charge_handler(uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low);

void gauge_timer_handler(void * p_context);

void enable_high_voltage(bool enable);

#endif /* BATTERY_H_ */
