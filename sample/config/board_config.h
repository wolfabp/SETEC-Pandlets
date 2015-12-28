#ifndef MAIN_CONFIG_H__
#define MAIN_CONFIG_H__

/*************************************************
* Define the board here!                         *
*************************************************/
#define BORON 																	//Sensing+ v4.0.1.BORON

/*************************************************
* Board specific defines                         *
**************************************************/
/* General debug enable */
#define GENERAL_DEBUG 					  1										//enable or disable app flow debug
#define SD_LOG        					  1										//enable or disable SD card error logging

/* GPIOS */
#define APP_GPIOTE_MAX_USERS              4										// Maximum number of users of the GPIOTE handler. Battery, base timer, watchdog and SD

/* TIMERS */
#define APP_TIMER_PRESCALER               2                                     // Value of the RTC1 PRESCALER register. 92us resolution
#define APP_TIMER_MAX_TIMERS              10                                    // Maximum number of simultaneously created timers.
#define APP_TIMER_OP_QUEUE_SIZE           20                                    // Size of timer operation queues.

#define BASE_TIMER_FREQ                   1 		 							//ms, defines the frequency of the timer for the sensor rate

/* BLE */
#define BOND_DELETE_ALL_BUTTON_ID         GPIO_1								// If pressed during initialization, removes all bonded devices.

#define IS_SRVC_CHANGED_CHARACT_PRESENT   0                                     // Include or not the service_changed characteristic.

#ifdef BORON
#define DEVICE_NAME                       "SETEC-15/16"							// Name of device. Will be included in the advertising data.
#endif

#define MANUFACTURER_NAME                 "FhP-AICOS"							// Manufacturer. Will be passed to Device Information Service.
#define APPEARANCE                        0		         					      // Appearance. Used for whatever is needed. Random value for now.

#define APP_ADV_INTERVAL                  160                                     // The advertising interval (in units of 0.625 ms)
#define APP_ADV_TIMEOUT_IN_SECONDS        0                                          // The advertising timeout in units of seconds.
#define ADV_BTN_TIME       				  APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)  // Time that the button needs to be pressed before starting advertising in ticks (2s)

#define MIN_CONN_INTERVAL                 100                                     // Minimum acceptable connection interval (ms).
#define MAX_CONN_INTERVAL                 200                                     // Maximum acceptable connection interval (ms).
#define SLAVE_LATENCY                     0                                       // Slave latency. Number of connection periods that the slave can ignore.
#define CONN_SUP_TIMEOUT                  2000                                    // Connection supervisory timeout (2 second). Time to consider the connection as lost. (ms)

#define FIRST_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)   // Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 second).
#define NEXT_CONN_PARAMS_UPDATE_DELAY     APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)  // Time between each call to sd_ble_gap_conn_param_update after the first (30 seconds).
#define MAX_CONN_PARAMS_UPDATE_COUNT      5                                       // Number of attempts before giving up the connection parameter negotiation.

#define SEC_PARAM_TIMEOUT                 30                                      // Timeout for Pairing Request or Security Request (in seconds).
#define SEC_PARAM_BOND                    1                                       // Perform bonding.
#define SEC_PARAM_MITM                    0                                       // Man In The Middle protection not required.
#define SEC_PARAM_IO_CAPABILITIES         BLE_GAP_IO_CAPS_NONE                    // No I/O capabilities.
#define SEC_PARAM_OOB                     0                                       // Out Of Band data not available.
#define SEC_PARAM_MIN_KEY_SIZE            7                                       // Minimum encryption key size.
#define SEC_PARAM_MAX_KEY_SIZE            16                                      // Maximum encryption key size.


/* SCHEDULER */
#define SCHED_MAX_EVENT_DATA_SIZE         MAX(APP_TIMER_SCHED_EVT_SIZE,\
                                             BLE_STACK_HANDLER_SCHED_EVT_SIZE)    // Maximum size of scheduler events.
#define SCHED_QUEUE_SIZE                  20                                      // Maximum number of events in the scheduler queue.

/* WATCHDOG */
#define WATCHDOG_TIMER                   5000								      // ms
#define WATCHDOG_TIMEOUT                 10 * 32768 							  //10 secs

/*************************************************
* Board specific defines                         *
**************************************************/
//pandlet
#ifdef BORON
#include "boron_config.h"
#endif

#define FIRMWARE_VERSION			 BOARD_FIRMWARE_VERSION + 10 				  //FW v. X.0.1.X

#include <stdbool.h>
bool twi_busy;

/*************************************************
* Board specific services                         *
**************************************************/
#include "ambient_service_config.h"

#endif /* MAIN_CONFIG_H__ */
