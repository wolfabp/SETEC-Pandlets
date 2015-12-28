#include "init.h"

/**@brief Function that initializes the necessary pins
 * of the Pandlet board.
 *
 * @details Enables 2V8, 5V0 and Induction Charger pins.
 */
void gpio_init(void){

	//Enable 3V3
	nrf_gpio_cfg_output(EN_PIN_3V3);
	nrf_gpio_pin_set(EN_PIN_3V3);

	//If this pin is LOW, Battery is charging
	nrf_gpio_cfg_input(USB_CH_STAT, GPIO_PIN_CNF_PULL_Pullup);

	//Pull CTRL pin to GND (disables the use of this pin)
	nrf_gpio_cfg_input(IND_CH_CTRL, GPIO_PIN_CNF_PULL_Pulldown);
	//nrf_gpio_cfg_input(IND_CH_CTRL, GPIO_PIN_CNF_PULL_Disabled);

	//If this pin is LOW, Battery is charging
	nrf_gpio_cfg_input(IND_CH_STAT, GPIO_PIN_CNF_PULL_Pullup);

	//Disable Wired Charging Source if Wireless charging is present
	nrf_gpio_cfg_output(IND_CH_EN);
	nrf_gpio_pin_set(IND_CH_EN);

	//ALERT pin of the fuel gauge
	nrf_gpio_cfg_input(GAUGE_INT, GPIO_PIN_CNF_PULL_Pullup);

	//Configure GPIO'S
	nrf_gpio_cfg_input(GPIO_1, GPIO_PIN_CNF_PULL_Pullup);
	nrf_gpio_cfg_output(GPIO_2);

	nrf_gpio_cfg_output(GPIO_3);
	nrf_gpio_cfg_output(GPIO_4);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
void leds_init(void){
    nrf_gpio_cfg_output(LED_RED);
    nrf_gpio_cfg_output(LED_GREEN);
    
    nrf_gpio_pin_set(LED_RED);
	nrf_gpio_pin_set(LED_GREEN);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
void timers_init(void){

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
    APP_ERROR_CHECK(app_timer_create(&m_base_timer_id, APP_TIMER_MODE_REPEATED, base_timer_handler));
    APP_ERROR_CHECK(app_timer_create(&m_gauge_timer_id, APP_TIMER_MODE_REPEATED, gauge_timer_handler));
    APP_ERROR_CHECK(app_timer_create(&m_watchdog_timer_id, APP_TIMER_MODE_REPEATED, watchdog_timer_handler));
}


/**@brief Function for initializing the GPIOTE handler module.
 */
void gpiote_init(void){

    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
    NVIC_SetPriority(GPIOTE_IRQn, APP_IRQ_PRIORITY_LOW);

    //Wired/Wireless charger and Gauge interrupt
    APP_ERROR_CHECK(app_gpiote_user_register(&m_gpiote_user_ch, IND_CH_STAT_BITMASK | GAUGE_INT_BITMASK | USB_CH_STAT_BITMASK,
    			IND_CH_STAT_BITMASK | GAUGE_INT_BITMASK | USB_CH_STAT_BITMASK, battery_charge_handler));

	APP_ERROR_CHECK(app_gpiote_user_enable(m_gpiote_user_ch));
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
void ble_stack_init(void){
	uint32_t err_code;
    
	uint32_t opt_id = BLE_COMMON_OPT_RADIO_CPU_MUTEX;
	ble_opt_t cpu_blocking_enabled;
	cpu_blocking_enabled.common_opt.radio_cpu_mutex.enable = 0;

	SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);

	// Enable BLE stack
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;

    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
    
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_tx_power_set(4);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_opt_set(opt_id, &cpu_blocking_enabled);;
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Event Scheduler initialization.
 */
void scheduler_init(void){
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**@brief Function for initializing security parameters.
 */
void device_manager_init(void){
	uint32_t                err_code;
	dm_init_param_t         init_data;
    dm_application_param_t  register_param;
    
    // Initialize persistent storage module. 
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    // Clear all bonded centrals if the Bonds Delete button is pushed.
    init_data.clear_persistent_data = (nrf_gpio_pin_read(BOND_DELETE_ALL_BUTTON_ID) == 0);
    
    err_code = dm_init(&init_data);
    APP_ERROR_CHECK(err_code);
    
	memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));
    
    register_param.sec_param.timeout      = SEC_PARAM_TIMEOUT;
    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
void gap_params_init(void){
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(APPEARANCE);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MSEC_TO_UNITS(MIN_CONN_INTERVAL, UNIT_1_25_MS);
    gap_conn_params.max_conn_interval = MSEC_TO_UNITS(MAX_CONN_INTERVAL, UNIT_1_25_MS);
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = MSEC_TO_UNITS(CONN_SUP_TIMEOUT, UNIT_10_MS);

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 */
void services_init(void){
    uint32_t err_code;

//Only add services that are connected
#if AMBIENT_SERVICE_ENABLED
	//Ambient service
    ble_ambient_init_t amb_init; //create the struct need for the service

	memset(&m_amb, 0, sizeof(ble_ambient_t));
    memset(&amb_init, 0, sizeof(ble_ambient_init_t));

    amb_init.evt_handler = ble_amb_evt;
    amb_init.support_notification = true;

	#if TEMP_ENABLED
    amb_init.temp_init_configuration = TEMP_INITIAL_CONFIG;
	#endif

	#if PR_ENABLED
    amb_init.pr_init_configuration = PR_INITIAL_CONFIG;
	#endif

	#if HUM_ENABLED
    amb_init.hum_init_configuration = HUM_INITIAL_CONFIG;
	#endif

    err_code = ble_ambient_init(&m_amb, &amb_init); //perform service initiation
    APP_ERROR_CHECK(err_code);
#endif

    ble_dis_init_t dis_init;

    // Initialize Device Information Service
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
void advertising_init(){
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;

    memset(&scanrsp, 0, sizeof(scanrsp));
    err_code = ble_advdata_set(&advdata, &scanrsp);

    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Connection Parameters module.
 */
void conn_params_init(void){
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the device drivers.
 */
void drivers_init(void){
	//Configures battery driver
	APP_ERROR_CHECK(battery_init());

	//Configures gpio driver
	APP_ERROR_CHECK(fhp_gpio_init());

	//Startup the TWI
	if(!twi_master_init())
		APP_ERROR_CHECK(TWI_INIT_ERROR);

	//Max17048 is used for battery monitoring
	APP_ERROR_CHECK(max17048_init(GAUGE_ADDRESS));

	//Max1161x is used for sensors ADC based.
	APP_ERROR_CHECK(max1161x_init(ADC_MUX_ADDRESS));

	//PCA938 is used for GPIO extender.
	APP_ERROR_CHECK(pca9538_init(GPIO_EXT_ADDRESS));

	//BME280 is used for temperature, pressure and humidity sensors.
	APP_ERROR_CHECK(bme280_init_nrf51(BME_ADDRESS));

	//Even if not used, it is present in the Pandlet board
	//Init it to put it to sleep.
	mpu9x50_init(MPU_ADDRESS);
}


/**@brief Function to initialize the sensors.
 */
void sensors_init(void){
	//let all sensors power up
	nrf_delay_ms(2000);

#if AMBIENT_SERVICE_ENABLED

	/* Load the sensors for the Ambient Service */
	#if TEMP_ENABLED
	APP_ERROR_CHECK(temp_init(&m_amb));
	APP_ERROR_CHECK(temp_configs_update());
	#endif /* TEMP_ENABLED */

	#if PR_ENABLED
	APP_ERROR_CHECK(pr_init(&m_amb));
	APP_ERROR_CHECK(pr_configs_update());
	#endif /* PR_ENABLED */

	#if HUM_ENABLED
	APP_ERROR_CHECK(hum_init(&m_amb));
	APP_ERROR_CHECK(hum_configs_update());
	#endif /* HUM_ENABLED */

#endif

	twi_busy = false;
	enable_high_voltage(false); //disable all high voltage sensors to save energy

	#ifndef RUN_DEEP_SLEEP_TEST
    APP_ERROR_CHECK(app_timer_start(m_gauge_timer_id, APP_TIMER_TICKS(GAUGE_TIMER, APP_TIMER_PRESCALER), NULL)); //enable gauge timer, GAUGE_TIMER s between reads
	#endif
}

void watchdog_init(){
	NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) | ( WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
	NRF_WDT->CRV = WATCHDOG_TIMEOUT;
	NRF_WDT->RREN |= WDT_RREN_RR0_Msk; //Enable reload register 0
	NRF_WDT->TASKS_START = 1;

    APP_ERROR_CHECK(app_timer_start(m_watchdog_timer_id, APP_TIMER_TICKS(WATCHDOG_TIMER, APP_TIMER_PRESCALER), NULL));
}


/***********************************************************************
 *                     End of init functions                           *
 * ********************************************************************/

/**@brief Function for starting the application timers.
 */
void application_work_start(void *data, uint16_t size){
	UNUSED_PARAMETER(data);
	UNUSED_PARAMETER(size);

    bool flag = false;

#if AMBIENT_SERVICE_ENABLED == 1
    // Start application timers
	#if TEMP_ENABLED == 1
	flag |= (m_temp.IS_TEMP_ENABLED);
	#endif

	#if PR_ENABLED == 1
	flag |= (m_pr.IS_PR_ENABLED);
	#endif

	#if HUM_ENABLED == 1
	flag |= (m_hum.IS_HUM_ENABLED);
	#endif

	if(flag){
		APP_ERROR_CHECK(app_timer_start(m_base_timer_id,
			APP_TIMER_TICKS(BASE_TIMER_FREQ, APP_TIMER_PRESCALER), NULL));

		printf("application_work_start() timer started! \r\n");
	}
#endif
}


/**@brief Function for stopping the application timers.
 */
void application_work_stop(){

    APP_ERROR_CHECK(app_timer_stop(m_base_timer_id));

	printf("application_work_stop() stopped! \r\n");
}

//GAP parameters
static float min_conn_setted = 0;
static float max_conn_setted = 0;
static float conn_timeout_setted = 0;

/**@brief Function to update GAP parameters.
 */
void gap_params_update(){

	//These are the predefined values of the connection interval
	//They are more than enough for the Ambient and Actuation services
	//but not enough for the Motion Service
	float min_conn = MIN_CONN_INTERVAL;
	float max_conn = MAX_CONN_INTERVAL;
	float conn_timeout = CONN_SUP_TIMEOUT;
	uint16_t latency = SLAVE_LATENCY;

	ble_gap_conn_params_t new_params;
	uint32_t err_code;

	uint8_t highest_rate = 0;

	switch(highest_rate){
		case 0b000: //8 Hz
			min_conn = 75; //ms
			max_conn = 80; //ms
			break;
		case 0b001: //50 Hz
			min_conn = 7.5; //ms
			max_conn = 10; //ms
			break;
		case 0b010: //100 Hz
			min_conn = 7.5; //ms
			max_conn = 7.5; //ms
			break;
		//case 0b011: //250Hz
		//case 0b100: //500 Hz
		//case 0b111: //1 KHz
		//	min_conn = 7.5; //ms
		//	max_conn = 7.5; //ms
		//	break;

		default: //just set max rate
			min_conn = 7.5; //ms
			max_conn = 7.5; //ms
			break;
	}

	conn_timeout = max_conn * 20; //BLE specification states that it should be at least 8x

	if(!((min_conn != min_conn_setted) || (max_conn != max_conn_setted) || (conn_timeout != conn_timeout_setted)))
		return;

	//Set new parameters
	memset(&new_params, 0, sizeof(ble_gap_conn_params_t));
	new_params.min_conn_interval = MSEC_TO_UNITS(min_conn, UNIT_1_25_MS);
	new_params.max_conn_interval = MSEC_TO_UNITS(max_conn, UNIT_1_25_MS);
	new_params.slave_latency     = latency;
	new_params.conn_sup_timeout  = MSEC_TO_UNITS(conn_timeout, UNIT_10_MS);
	err_code = ble_conn_params_change_conn_params(&new_params);

	APP_ERROR_CHECK(err_code);

	min_conn_setted = min_conn;
	max_conn_setted = max_conn;
	conn_timeout_setted = conn_timeout;


	printf("Setted interval connection to min: %hd max: %hd (in units of 0.1 ms)\n", (uint16_t)(min_conn * 10), (uint16_t)(max_conn * 10));
}




