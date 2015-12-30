/** @file
 * 
 * @brief BLE Pandlets initializations
 *
 * @details This file contains all the needed configuration functions needed
 * for the correct functioning of the Pandlets firmware.
 *
 */


/** Nordic Includes **/
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_hci.h"
#include "ble_dis.h"
#include "softdevice_handler.h"
#include "device_manager.h"
#include "pstorage.h"
#include "app_util_platform.h"
#include "app_timer.h"
#include "app_gpiote.h"
#include "app_scheduler.h"

/** FhP Includes **/
#include "board_config.h"
#include "utils.h"
#include "general_error_codes.h"   //Error codes shared by the various modules
#include "fhp_gpio.h"

//DFU bootloader code
#define BOOTLOADER_DFU_START 0xB1

dm_application_instance_t               m_app_handle;                                   // Application identifier allocated by device manager
app_timer_id_t				            m_base_timer_id;							    // Base timer for reads
app_timer_id_t				            m_watchdog_timer_id;		        		    // Watchdog timer for reloads

/*******************************************************************************
 *                   Peripheral and general drivers
 * *****************************************************************************/

///***************** I2C ***********************/
#if (AMBIENT_SERVICE_ENABLED == 1)
#include "twi_master.h" 																//I2C driver
#endif

///***************** SD Card ***********************/
#if SD_LOG || ACCELEROMETER_SD_CARD_LOG
#include "ff.h"                                                                         //SD card functions and includes
FATFS sd_card;                                                                          // Work area (file system object) for logical drive
#endif

///***************** Battery ***********************/
#include "battery.h"                                                                    //Battery status and charging functions
app_gpiote_user_id_t                    m_gpiote_user_ch;   							// Application identifier allocated by GPIOTE manager
app_timer_id_t				            m_gauge_timer_id;							    // Timer to use gauge as charging indicator

///***************** ADC ***********************/
#include "max1161x.h"

///************* GPIO EXTENDER *****************/
#include "pca9538.h"


///***************** Ambient Sensing ***********************/
#include "bme280.h"

///***************** Motion Sensing ***********************/
#include "mpu9x50.h"

/*******************************************************************************
 *                  Ambient Service configs
 * *****************************************************************************/
#include "ambient_service_config.h" 													//Where all Ambient service configurations are stored

//Variables
#if AMBIENT_SERVICE_ENABLED == 1
#include "ble_ambient.h" 																// Our service
ble_ambient_t                           m_amb;										    // Ambient Service struct


//***************** TEMP ***********************/
#if TEMP_ENABLED == 1
#include "temp.h"      																	//device driver for Temperature sensor
#endif

//***************** PR ***********************/
#if PR_ENABLED == 1
#include "pr.h"      																	//device driver for Pressure sensor
#endif

//***************** HUM ***********************/
#if HUM_ENABLED == 1
#include "hum.h"      																	//device driver for Humidity sensor
#endif

#endif /**AMBIENT_SERVICE_ENABLED*/


/*******************************************************************************
 *                             Functions
 * *****************************************************************************/
/**@brief Function that initializes the necessary pins
 * of the Pandlet board.
 *
 * @details Enables 2V8, 5V0 and Induction Charger pins.
 */
void gpio_init(void);


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
void leds_init(void);


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
void timers_init(void);


/**@brief Function for initializing the GPIOTE handler module.
 */
void gpiote_init(void);


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
void ble_stack_init(void);


/**@brief Function for the Event Scheduler initialization.
 */
void scheduler_init(void);


/**@brief Function for initializing security parameters.
 */
void device_manager_init(void);


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
void gap_params_init(void);


/**@brief Function for initializing services that will be used by the application.
 */
void services_init(void);


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
void advertising_init();
                                    
                                    
/**@brief Function for initializing the Connection Parameters module.
 */
void conn_params_init(void);


/**@brief Function to initialize the device drivers.
 */
void drivers_init(void);


/**@brief Function to initialize the sensors.
 */
void sensors_init(void);

/**@brief Resets MPU configurations.
 * It will be removed in the near future.
 */
void reset_configs(int id);


/**@brief Checks for errors in ble services.
 *
 * @details If code is different than BLE_ERROR_GATTS_SYS_ATTR_MISSING,
 * BLE_ERROR_NO_TX_BUFFERS, NRF_ERROR_INVALID_STATE or NRF_SUCCESS,
 * app_error_handler is called.
 */
void check_ble_service_err_code(uint32_t err_code);


/**@brief Function for starting the application timers.
 */
void application_work_start(void *data, uint16_t size);


/**@brief Function for stopping the application timers.
 */
void application_work_stop();


void gap_params_update();


void watchdog_init();

/***********************************************************************
 *       Functions that need to be implemented in user app             *
 * ********************************************************************/
  
 /**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
void ble_evt_dispatch(ble_evt_t * p_ble_evt);


/**@brief Function for dispatching a SYS event to all modules with a SYS event handler.
 */
void sys_evt_dispatch(uint32_t sys_evt);


/**@brief Function for handling the Connection Parameters Module.
 */
void on_conn_params_evt(ble_conn_params_evt_t * p_evt);


/**@brief Function for handling the Device Manager events.
 *
 * @param[in]   p_evt   Data associated to the device manager event.
 */
uint32_t device_manager_evt_handler(dm_handle_t const    * p_handle,
                                    dm_event_t const     * p_event,
                                    api_result_t           event_result);
                                    
                                    
/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
void conn_params_error_handler(uint32_t nrf_error);


#if AMBIENT_SERVICE_ENABLED == 1
/**@brief Function to handle the ble_amb service events.
 *
 * @param[in]   p_amb   The ble_amb service object.
 * @param[in]   p_evt   The ble_amb event.
 */
void ble_amb_evt(ble_ambient_t * p_amb, ble_ambient_evt_t * p_evt);

#endif

/**@brief Function to handle low battery event.
 *
 */
void on_low_bat_evt();

/**@brief Base timer interrupt used to trigger the reads.
* The frequency is based of BASE_TIMER_FREQ.
*
* @param[in]   p_context   Not used.
*/
void base_timer_handler(void * p_context);


void watchdog_timer_handler(void * p_context);


void advertising_start(void);
