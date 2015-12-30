/*
 * fhp_gpio.c
 *
 *  Created on: Aug 19, 2015
 *      Author: safetysensor
 */

#include "fhp_gpio.h"

static nrf_pwm_config_t pwm_config = PWM_DEFAULT_CONFIG;
static uint8_t pin_in_use[PWM_MAX_CHANNELS] = {255, 255, 255, 255};

static void timer_handler(void * p_context);

uint32_t fhp_gpio_init(){

	for(uint8_t i = 0; i < PWM_MAX_CHANNELS; i++){
	    if(app_timer_create(&pwm_config.pin_cfg[i].timer, APP_TIMER_MODE_REPEATED, timer_handler)){
	    	return GPIO_INIT_TIMER_ERROR;
	    }
	}

	return NRF_SUCCESS;
}

static void clear_pwm_pin(uint8_t pin){
	//Clean up PWM channels
	for(uint8_t i = 0; i < PWM_MAX_CHANNELS; i++){
		if(pin == pwm_config.gpio_num[i]){
			nrf_pwm_set_enabled(false);

			app_timer_stop(pwm_config.pin_cfg[i].timer);

			for(; i < PWM_MAX_CHANNELS - 1; i++){
				pwm_config.gpio_num[i] = pwm_config.gpio_num[i + 1];
				pwm_config.pin_cfg[i] = pwm_config.pin_cfg[i + 1];
			}

			pwm_config.gpio_num[PWM_MAX_CHANNELS - 1]           = 255;
			pwm_config.pin_cfg[PWM_MAX_CHANNELS - 1].mode       = FHP_MODE_NONE;
			pwm_config.pin_cfg[PWM_MAX_CHANNELS - 1].pwm        = 0;
			pwm_config.pin_cfg[PWM_MAX_CHANNELS - 1].pwm_rising = true;
			pwm_config.pin_cfg[PWM_MAX_CHANNELS - 1].intensity  = 0;
		    app_timer_create(&pwm_config.pin_cfg[PWM_MAX_CHANNELS - 1].timer, APP_TIMER_MODE_REPEATED, timer_handler);
			pwm_config.num_channels -= 1;

			nrf_pwm_init(&pwm_config);

			for(uint8_t t = 0; t < PWM_MAX_CHANNELS; t++){
				if(pin_in_use[t] == pin){
					pin_in_use[t] = 255;
					break;
				}
			}

			return;
		}
	}
}

uint32_t fhp_gpio_set(uint8_t pin){
	clear_pwm_pin(pin);

    enable_high_voltage(true);
	nrf_gpio_cfg_output(pin);
	nrf_gpio_pin_set(pin);

	return NRF_SUCCESS;
}

uint32_t fhp_gpio_clear(uint8_t pin){
	clear_pwm_pin(pin);

    enable_high_voltage(false);
	nrf_gpio_cfg_output(pin);
	nrf_gpio_pin_clear(pin);

	return NRF_SUCCESS;
}

//rate in 100ms, intensity 255 is maximum light
static uint32_t fhp_gpio_config(uint8_t pin, uint8_t rate, uint8_t intensity, uint8_t pin_mode){
	clear_pwm_pin(pin);

	nrf_gpio_cfg_output(pin);
	nrf_gpio_pin_clear(pin);

	for(uint8_t i = 0; i < PWM_MAX_CHANNELS; i++){
		if(pwm_config.gpio_num[i] == 255){
			uint8_t t = 0;
			uint32_t timeout = (pin_mode == FHP_MODE_BLINK) ? (rate * 100 / 2) : (rate * 100 / (2 * (intensity / PWM_STEP_SIZE)));

			nrf_pwm_set_enabled(false);

			pwm_config.gpio_num[i] = pin;
		    pwm_config.num_channels += 1;
			pwm_config.pin_cfg[i].mode = pin_mode;
			pwm_config.pin_cfg[i].intensity = intensity;

			for(; t < PWM_MAX_CHANNELS; t++){
				if(pin_in_use[t] == 255){
					pin_in_use[t] = pin;
					break;
				}
			}

			if((nrf_pwm_init(&pwm_config))){ //Re-init pwm module
				clear_pwm_pin(pin);
				return GPIO_TIMER_ERROR;
			}

			if(app_timer_start(pwm_config.pin_cfg[i].timer, APP_TIMER_TICKS(timeout, APP_TIMER_PRESCALER), (void *)&pin_in_use[t])){
				//Reset stuff
				clear_pwm_pin(pin);
				return GPIO_TIMER_ERROR;
			}

			if((pin == LED_GREEN) || (pin == LED_RED)){
				enable_high_voltage(true);
			}

			nrf_pwm_set_enabled(true);
			printf("Configured pin %u in mode %u \n", (unsigned int)pin, (unsigned int)pin_mode);
			return NRF_SUCCESS;
		}
	}

	return GPIO_MAX_PIN_REACHED_ERROR;
}

//Rate comes in hundreds of ms for a full cycle
uint32_t fhp_gpio_blink(uint8_t pin, uint8_t rate, uint8_t intensity){
	clear_pwm_pin(pin);

	nrf_gpio_cfg_output(pin);
	nrf_gpio_pin_clear(pin);

	return fhp_gpio_config(pin, rate, intensity, FHP_MODE_BLINK);
}

//Rate comes in seconds for a full cycle
//intensity = 255 represents fade to maximum light
uint32_t fhp_gpio_fade(uint8_t pin, uint8_t rate, uint8_t intensity) {
	clear_pwm_pin(pin);

	nrf_gpio_cfg_output(pin);
	nrf_gpio_pin_clear(pin);

	return fhp_gpio_config(pin, (rate * 10), intensity, FHP_MODE_FADE);
}

static void timer_handler(void * p_context){
	uint8_t pin = *((uint8_t*)p_context);

	//Find the correct index
	uint8_t i = 0;

	for(; i < PWM_MAX_CHANNELS; i++){
		if(pwm_config.gpio_num[i] == pin){
			pin = i;
		}
	}

	if(pwm_config.pin_cfg[pin].mode == FHP_MODE_BLINK){
		if(pwm_config.pin_cfg[pin].pwm == 0){
			pwm_config.pin_cfg[pin].pwm = pwm_config.pin_cfg[pin].intensity;
		}
		else {
			pwm_config.pin_cfg[pin].pwm = 0;
		}

		nrf_pwm_set_value(pin, pwm_config.pin_cfg[pin].pwm);
	}
	else if(pwm_config.pin_cfg[pin].mode == FHP_MODE_FADE){
		if(pwm_config.pin_cfg[pin].pwm_rising && ((pwm_config.pin_cfg[pin].pwm + PWM_STEP_SIZE) >= pwm_config.pin_cfg[pin].intensity)){
			pwm_config.pin_cfg[pin].pwm = pwm_config.pin_cfg[pin].intensity;
			pwm_config.pin_cfg[pin].pwm_rising = false;
		}
		else if(!pwm_config.pin_cfg[pin].pwm_rising && ((pwm_config.pin_cfg[pin].pwm - PWM_STEP_SIZE) <= 0)){
			pwm_config.pin_cfg[pin].pwm = 0;
			pwm_config.pin_cfg[pin].pwm_rising = true;
		}
		else{
			pwm_config.pin_cfg[pin].pwm += ((pwm_config.pin_cfg[pin].pwm_rising) ? 4 : (-4));

		}

		nrf_pwm_set_value(pin, pwm_config.pin_cfg[pin].pwm);
	}
}

bool fhp_gpio_is_being_used(){
	for(uint8_t i = 0; i < PWM_MAX_CHANNELS; i++){
		if((pwm_config.gpio_num[i] == LED_GREEN) || (pwm_config.gpio_num[i] == LED_RED)){
			return true;
		}
	}

	return false;
}
