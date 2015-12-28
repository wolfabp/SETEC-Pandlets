/*
 * fhp_gpio.h
 *
 *  Created on: Aug 19, 2015
 *      Author: safetysensor
 */

#ifndef FHP_GPIO_H__
#define FHP_GPIO_H__

#include <stdint.h>
#include <stdbool.h>

#include "board_config.h"
#include "nrf_pwm.h"
#include "utils.h"
#include "nrf_gpio.h"
#include "battery.h"
#include "general_error_codes.h"

#define PWM_STEP_SIZE 4

#define LED_MAX_INTENSITY     255
#define LED_MEDIUM_INTENSITY  128
#define LED_LOW_INTENSITY     64

uint32_t fhp_gpio_init();
uint32_t fhp_gpio_set(uint8_t pin);
uint32_t fhp_gpio_clear(uint8_t pin);
uint32_t fhp_gpio_blink(uint8_t pin, uint8_t rate, uint8_t intensity); //rate in hundreds of ms
uint32_t fhp_gpio_fade(uint8_t pin, uint8_t rate, uint8_t intensity);  //rate in s, intensity 255 is maximum light
bool     fhp_gpio_is_being_used(); //checks if the module is being used

#endif /* FHP_GPIO_H__ */

