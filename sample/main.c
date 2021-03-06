/** @file
 * 
 * @brief Pandlets BLE app.
 *
 */

#include "init.h"

#define DEAD_BEEF                       0xDEADBEEF                                  // Value used as error code on stack dump, can be used to identify stack location on stack unwind.

static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    // Handle of the current connection.

/**@brief Function for error handling, which is called when an error has occurred. 
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name){
    
	//Prepare string to be written
	printf("Error occurred: %x at line %u in file ", (unsigned int)error_code, (unsigned int)line_num);
	printf("%s\n", p_file_name);

	#if SD_LOG == 1
	char buffer[128];
	sprintf(buffer, "Error occurred: %x at line %u in file %s\n", (unsigned int)error_code, (unsigned int)line_num, p_file_name);

	if(sd_card.fs_type == 0) { //SD card not mounted
		//Mount the SD card
		if(f_mount(&sd_card, "", 1) == 0){
			log_to_sd("pandlet.txt", buffer, strlen(buffer));
			f_mount(NULL, "", 1);
		}
	}
	else { //SD card already mounted, someone is logging to it!
		log_to_sd("pandlet.txt", buffer, strlen(buffer));
	}
	#endif /* SD_LOG */
}


/**
 * Called when there is an Hardfault. Just resets the MCU.
 */
void HardFault_Handler(void)  {
	printf("An Hardfault occured. Reseting in 5s...\r\n");

	nrf_delay_ms(5000);
	NVIC_SystemReset();
}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name){
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
	printf("An assert on the Softdevice occured. Reseting in 5s...\r\n");

	nrf_delay_ms(5000);
	NVIC_SystemReset();//In a softdevice assert, nothing can be done to recover other than reset.
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
void conn_params_error_handler(uint32_t nrf_error){
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
void on_conn_params_evt(ble_conn_params_evt_t * p_evt){
    
    //If can't set the needed connection interval, we should disconnect :(
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
        printf("BLE_HCI_CONN_INTERVAL_UNACCEPTABLE, some samples will be lost!\r\n");
}


/**@brief Function for starting advertising.
 */
void advertising_start(void){
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;

	advertising_init	();
    
    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));
    
    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;

    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);

	if(err_code == NRF_SUCCESS){
		printf("Advertising...\r\n");
		fhp_gpio_blink(LED_GREEN, 10, LED_MAX_INTENSITY);
	}
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt){
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        	printf("BLE_GAP_EVT_CONNECTED \r\n");
        	fhp_gpio_clear(LED_GREEN);
        	fhp_gpio_set(LED_GREEN);

            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            //*(uint32_t *)~0 = 4; //use this to force an hardfault

            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
        	//Upon disconnection, reset all sensors (for energy saving)

        	application_work_stop(); //Stop base timer.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            // set LED greeen
        	printf("BLE_GAP_EVT_DISCONNECTED \r\n");
			fhp_gpio_clear(LED_GREEN);
 
            /********* AMBIENT SERVICE ********/
			#if AMBIENT_SERVICE_ENABLED
            m_amb.conn_handle = m_conn_handle;

           	#if TEMP_ENABLED
            APP_ERROR_CHECK(temp_reset_configs());
			#endif

			#if PR_ENABLED
            APP_ERROR_CHECK(pr_reset_configs());
			#endif

			#if HUM_ENABLED
			APP_ERROR_CHECK(hum_reset_configs());
			#endif
            #endif /*AMBIENT_SERVICE_ENABLED*/


            enable_high_voltage(false);
            advertising_start();         //Restart advertising
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT){ 
				printf("BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT \r\n");
            }
            break;
 
		case BLE_GAP_EVT_CONN_PARAM_UPDATE:
			//Used only for debug, does nothing.
			printf("BLE_GAP_EVT_CONN_PARAM_UPDATE \r\n");
            break;

        default:
            break;
    }
}


void on_low_bat_evt(){
	//Low battery detected. Disable all functionality!
	printf("on_low_bat_evt() \r\nReseting all configurations...\r\n");

	application_work_stop(); //Stop base timer.

     /********* AMBIENT SERVICE ********/
	#if AMBIENT_SERVICE_ENABLED
 	#if TEMP_ENABLED
    APP_ERROR_CHECK(temp_reset_configs());
	#endif

	#if PR_ENABLED
    APP_ERROR_CHECK(pr_reset_configs());
	#endif

	#if HUM_ENABLED
	APP_ERROR_CHECK(hum_reset_configs());
	#endif
    #endif /*AMBIENT_SERVICE_ENABLED*/

    printf("Configurations reseted. \r\n");
    printf("From now on, only reads are allowed until the battery is charged. \r\n");

	enable_high_voltage(false);
}


/**@brief Function for handling the Device Manager events.
 * Used for pairing.
 *
 * @param[in]   p_evt   Data associated to the device manager event.
 */
uint32_t device_manager_evt_handler(dm_handle_t const    * p_handle,
								    dm_event_t const     * p_event,
								    api_result_t           event_result){
    APP_ERROR_CHECK(event_result);

    switch(p_event->event_id){
        default:
            // No event implemented.
            break;
    }

    return NRF_SUCCESS;
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
void ble_evt_dispatch(ble_evt_t * p_ble_evt){
    dm_ble_evt_handler(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);

	#if AMBIENT_SERVICE_ENABLED == 1
	ble_ambient_on_ble_evt(&m_amb, p_ble_evt); //Send event to ambient service handler.
	#endif
}


/**@brief Function for handling the SYS events.
 * Not used.
 *
 * @param[in]   sys_evt   Data associated to the device manager event.
 */
static void on_sys_evt(uint32_t sys_evt){}


/**@brief Function for dispatching a SYS event to all modules with a SYS event handler.
 */
void sys_evt_dispatch(uint32_t sys_evt){
    pstorage_sys_event_handler(sys_evt);
    on_sys_evt(sys_evt);
}

#if AMBIENT_SERVICE_ENABLED
/**@brief Function to handle the ble_amb service events.
 * 
 * @param[in]   p_amb   The ble_amb service object.
 * @param[in]   p_evt   The ble_amb event.
 */
void ble_amb_evt(ble_ambient_t * p_amb, ble_ambient_evt_t * p_evt){
	printf("ble_amb_evt() \r\n");

	application_work_stop(); //Stop base timer.

    switch (p_evt->evt_type)
    {
		#if TEMP_ENABLED == 1
        case BLE_AMBIENT_EVT_TEMP_CONFIG_CHANGED:
        	printf("BLE_AMBIENT_EVT_TEMP_CONFIG_CHANGED: 0x%x\n", m_amb.temp_configuration);
			APP_ERROR_CHECK(temp_configs_update()); //Update Temp configurations.
            break;
		#endif

		#if PR_ENABLED == 1
        case BLE_AMBIENT_EVT_PR_CONFIG_CHANGED:
        	printf("BLE_AMBIENT_EVT_PR_CONFIG_CHANGED: 0x%x\n", m_amb.pr_configuration);
			APP_ERROR_CHECK(pr_configs_update()); //Update Pr configurations.
            break;
		#endif

		#if HUM_ENABLED == 1
		case BLE_AMBIENT_EVT_HUM_CONFIG_CHANGED:
        	printf("BLE_AMBIENT_EVT_HUM_CONFIG_CHANGED: 0x%x\n", m_amb.hum_configuration);
        	APP_ERROR_CHECK(hum_configs_update()); //Update Hum configurations.
			break;
		#endif

        default:
            // No implementation needed.
            break;
    }

	enable_high_voltage(false);     //try to disable high voltage sensors

    //The scheduler doesn't have higher priority than BLE, so, in a event where multiple configs
    //are changed, the scheduled events only occur in the end of the BLE events, reducing multiple
    //timer starts and stops.
    app_sched_event_put(NULL, 0, application_work_start); //Start base timer.
}
#endif /* AMBIENT_SERVICE_ENABLED */


/**@brief Function to handle the base timer interrupt.
 *
 * @param[in]   p_context   not used.
 */
void base_timer_handler(void * p_context){	
	UNUSED_PARAMETER(p_context);

#if AMBIENT_SERVICE_ENABLED == 1

	/*********** TEMP *************/
	#if TEMP_ENABLED == 1
	APP_ERROR_CHECK(temp_timer_handler()); //Call handler for Temp sensor
	#endif

	/*********** PR *************/
	#if PR_ENABLED == 1
	APP_ERROR_CHECK(pr_timer_handler()); //Call handler for Pr sensor
	#endif

	/*********** HUM *************/
	#if HUM_ENABLED == 1
	APP_ERROR_CHECK(hum_timer_handler()); //Call handler for Hum sensor
	#endif

#endif

	enable_high_voltage(false); //try to disable 5V sensors (checks if they are on)
}


/**@brief Handles the watchdog reload.
 */
void watchdog_timer_handler(void * p_context){
	NRF_WDT->RR[0] = WDT_RR_RR_Reload; //Reload watchdog register 0
}


/**@brief Function for entering sleep. 
 */
static void power_manage(void){
	
    uint32_t err_code = sd_app_evt_wait(); //Go to sleep and wait for events.
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initialization.
 */
static void setup(void){

	gpio_init();
	printf("GPIO inits ok!\r\n");

	leds_init();
	printf("LEDS ok!\r\n");
	
	timers_init();
	printf("timers ok!\r\n");
	
	watchdog_init();
	printf("watchdog_init ok!\r\n");

    gpiote_init();
    printf("gpiote ok!\r\n");
	
	ble_stack_init();
	printf("ble_stack ok!\r\n");
	
	scheduler_init();
	printf("scheduler_init ok!\r\n");
	
	device_manager_init();
	printf("sec_params ok!\r\n");
	
	gap_params_init();
	printf("gap_parameters_init ok!\r\n");
	
	services_init();
	printf("services ok!\r\n");
	
	advertising_init();
	printf("advertising ok!\r\n");
	
	conn_params_init();
	printf("conn_params ok!\r\n");
	
	drivers_init();
	printf("drivers_init ok!\r\n");

	sensors_init();
	printf("sensors_init ok!\r\n");

	nrf_gpio_pin_clear(LED_RED);
	nrf_gpio_pin_clear(LED_GREEN);

	gauge_timer_handler(NULL); //Read battery right away!
}

int log2sd(char* message, char *filename){ // returns -1 if SD_LOG is disabled
 
#if SD_LOG == 1
        char buffer[128];
        sprintf(buffer, message);
 
        if(sd_card.fs_type == 0) { //SD card not mounted
                //Mount the SD card
                if(f_mount(&sd_card, "", 1) == 0){
                printf("CHUPAINDE-ME!\n");
                        log_to_sd(filename, buffer, strlen(buffer));
                        f_mount(NULL, "", 1);
                }
        }
        else { //SD card already mounted, someone is logging to it!
                log_to_sd(filename, buffer, strlen(buffer));
        }
       
        return 0;
#endif /* SD_LOG */
 
        return -1;
}

/**@brief Function for application main entry.
 */
int main(void){
    // Initialize
    //printf("Chupa-ma gaita\n");
    setup();

    // Start execution
    advertising_start();

    log2sd("sua puta do caralho sua vaca de merda lava-me essa boca\n", "juju.txt");
	

    // Enter main loop
    for (;;){
		app_sched_execute();
        power_manage(); //go to sleep
    }
}
