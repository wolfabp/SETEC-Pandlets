/*
 * battery.c
 *
 *  Created on: Mar 4, 2015
 *      Author: João Oliveira
 */

#include "battery.h"

static bool induction_charge_on = false;
static bool wired_charge_on = false;

uint32_t battery_init(){
	low_battery = false;
	high_voltage_on = true;

	return NRF_SUCCESS;
}

void induction_charging(bool state){
	if(state){
		printf("Induction charging on.\n");
		induction_charge_on = true;
		low_battery = false;

		fhp_gpio_fade(LED_GREEN, 3, LED_MEDIUM_INTENSITY);
	}
	else{
		printf("Induction charging off.\n");
		induction_charge_on = false;

		fhp_gpio_clear(LED_GREEN);
	}
}

void wired_charging(bool state){
	if(state){
		printf("Wired charging on.\n");
		wired_charge_on = true;
		low_battery = false;

		fhp_gpio_fade(LED_GREEN, 3, LED_MEDIUM_INTENSITY);
	}
	else{
		printf("Wired charging off.\n");
		wired_charge_on = false;

		fhp_gpio_clear(LED_GREEN);
	}
}

void battery_charge_handler(uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low){
	if (event_pins_low_to_high & IND_CH_STAT_BITMASK){  //Induction charging off
		induction_charging(false);
	}

	if (event_pins_high_to_low & IND_CH_STAT_BITMASK){  //Induction charging on
		induction_charging(true);
	}

	if (event_pins_low_to_high & USB_CH_STAT_BITMASK){  //Wired charging off
		wired_charging(false);
	}

	if (event_pins_high_to_low & USB_CH_STAT_BITMASK){  //Wired charging on
		wired_charging(true);
	}

	//TODO update the gauge logic
	if (event_pins_high_to_low & GAUGE_INT_BITMASK){  //ALERT in fuel gauge
		printf("Received a gauge alert.\n");
		gauge_alert alert;
		
		APP_ERROR_CHECK(max17048_get_alert(&alert));
	}
}

//This function is associated with a timer that runs continuously with an interval of GAUGE_TIMER ms
void gauge_timer_handler(void * p_context){
	UNUSED_PARAMETER(p_context);

	//Check for charging. This is helpful if the battery died and we were woken (so the no interrupt will be generated!)
	if(!nrf_gpio_pin_read(IND_CH_STAT) != induction_charge_on)
		induction_charging(!induction_charge_on);

	if(!nrf_gpio_pin_read(USB_CH_STAT) != wired_charge_on)
		wired_charging(!wired_charge_on);

	if(!twi_busy){
		printf("Reading battery value...\r\n");

		//Read state of charge
		uint8_t value[2];
		uint32_t err_code = max17048_read_soc(value);

		if(err_code == NRF_SUCCESS){
			printf("Battery value: %d. Charging status: %d.\r\n", value[1], (induction_charge_on || wired_charge_on) ? 1 : 0);
		}
		else
			APP_ERROR_CHECK(err_code);
	}

	//enable_high_voltage(false); //Try to disable the high voltage
}

//This function checks if the high voltage can be enabled or disabled
void enable_high_voltage(bool enable){
	if(enable){
		nrf_gpio_pin_set(EN_PIN_3V3);
		high_voltage_on = true;
	}
	else{
		bool can_disable = false;
		//can_disable &= !induction_charge_on;
		//can_disable &= !wired_charge_on;

		//can_disable &= !fhp_gpio_is_being_used(); //LEDs are connected to the high voltage

		if(can_disable){
			nrf_gpio_pin_clear(EN_PIN_3V3);
			high_voltage_on = false;
		}
	}
}

