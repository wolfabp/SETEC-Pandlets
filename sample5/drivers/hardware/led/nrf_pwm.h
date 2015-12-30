#ifndef __NRF_PWM_H__
#define __NRF_PWM_H__

#include <stdint.h>
#include <stdbool.h>
#include "app_timer.h"

// The maximum number of channels supported by the library. Should NOT be changed! 
#define PWM_MAX_CHANNELS        4

// Set this to 1 if the application uses a SoftDevice, 0 otherwise
#define USE_WITH_SOFTDEVICE     1

// To change the timer used for the PWM library replace the three defines below
#define PWM_TIMER               NRF_TIMER2
#define PWM_IRQHandler          TIMER2_IRQHandler
#define PWM_IRQn                TIMER2_IRQn
#define PWM_IRQ_PRIORITY        3

// For 3-4 PWM channels a second timer is necessary
#define PWM_TIMER2              NRF_TIMER1

#define FHP_MODE_NONE   0
#define FHP_MODE_BLINK  1
#define FHP_MODE_FADE   2

#define FHP_DEFAULT_CONFIG  {.mode          = FHP_MODE_NONE,        \
                             .pwm           = 0,                    \
                             .pwm_rising    = true,                 \
                             .intensity     = 0,                    \
                             .timer         = 0}


#define PWM_DEFAULT_CONFIG  {.num_channels   = 0,                    \
                             .gpio_num       = {255,255,255,255},    \
							 .pin_cfg        = {FHP_DEFAULT_CONFIG,  \
                                                FHP_DEFAULT_CONFIG,  \
												FHP_DEFAULT_CONFIG,  \
												FHP_DEFAULT_CONFIG}, \
                             .ppi_channel    = {1,2,3,4,5,6,7},      \
                             .gpiote_channel = {2,3,0,1},            \
                             .mode           = PWM_MODE_LED_255};

typedef enum
{
    PWM_MODE_LED_100,   // 0-100 resolution, 156Hz PWM frequency, 32kHz timer frequency (prescaler 9)
    PWM_MODE_LED_255,   // 8-bit resolution, 122Hz PWM frequency, 32kHz timer frequency (prescaler 9)
    PWM_MODE_LED_1000,  // 0-1000 resolution, 125Hz PWM frequency, 250kHz timer frequency (prescaler 6)
    
    PWM_MODE_MTR_100,   // 0-100 resolution, 20kHz PWM frequency, 2MHz timer frequency (prescaler 3)
    PWM_MODE_MTR_255,   // 8-bit resolution, 31kHz PWM frequency, 8MHz timer frequency (prescaler 1)
    
    PWM_MODE_BUZZER_255  // 8-bit resolution, 62.5kHz PWM frequency, 16MHz timer frequency (prescaler 0)
} nrf_pwm_mode_t;

typedef struct
{
    uint8_t         mode;
    uint8_t         pwm;
    bool            pwm_rising;
    uint8_t         intensity;
    app_timer_id_t  timer;

} fhp_pin_cfg_t;

typedef struct
{
    uint8_t         num_channels;
    uint8_t         gpio_num[PWM_MAX_CHANNELS];
    fhp_pin_cfg_t   pin_cfg[PWM_MAX_CHANNELS];
    uint8_t         ppi_channel[8];
    uint8_t         gpiote_channel[PWM_MAX_CHANNELS];
    uint8_t         mode;

} nrf_pwm_config_t; 


uint32_t nrf_pwm_init(nrf_pwm_config_t *config);

void nrf_pwm_set_value(uint32_t pwm_channel, uint32_t pwm_value);

void nrf_pwm_set_values(uint32_t pwm_channel_num, uint32_t *pwm_values);

void nrf_pwm_set_max_value(uint32_t max_value);

void nrf_pwm_set_enabled(bool enabled);

#endif
