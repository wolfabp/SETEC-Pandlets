#include "temp.h"

#if TEMP_ENABLED


uint32_t temp_init(ble_ambient_t *m_amb_init){
	temp_printf("temp_init() \r\n");

	m_temp.timer_count           = 0;
	m_temp.m_amb                 = m_amb_init;
	m_temp.IS_TEMP_ENABLED       = false;

	return NRF_SUCCESS;
}

uint32_t temp_configs_update(){
	temp_printf("\r\nExternal Temperature Configurations Update\r\n");

	uint8_t configuration = (m_temp.m_amb)->temp_configuration;

	//Reset count
	m_temp.timer_count  = 0;

	//Sensors enable configs
	m_temp.IS_TEMP_ENABLED = (configuration & AMB_ENABLE_BIT);

	//Rate configs
	switch(((configuration & AMB_RATE_BITS) >> 5)){
		case 0b000:
			m_temp.ticks = msec_to_ticks(10000000); //0.0001 Hz
			break;
		case 0b001:
			m_temp.ticks = msec_to_ticks(1000000);  //0.001 Hz
			break;
		case 0b010:
			m_temp.ticks = msec_to_ticks(100000);   //0.01 Hz
			break;
		case 0b011:
			m_temp.ticks = msec_to_ticks(10000);    //0.1 Hz
			break;
		case 0b111:
		default: //If not recognized set max rate
			m_temp.ticks = msec_to_ticks(2000);     //0.5 Hz
			break;
	}

	temp_printf("temp_configs_update() Ok!\r\n\r\n");
	return NRF_SUCCESS;
}



uint32_t temp_values_handler() {
	uint32_t  err_code = NRF_SUCCESS;
	int32_t temp_buffer;
	//char buffer[128];

	err_code = bme280_read_temperature(&temp_buffer);

	if (err_code != NRF_SUCCESS) {
		temp_printf("temp: bme280_read_temperature() failed.\r\n");
		return err_code;
	}
//	log2sd(sprintf(buffer, "%d", temp_buffer), "mdjunio.txt");
	
	temp_printf("Temperature: %d\n", (int)temp_buffer);
	err_code = ble_ambient_sensor_update(m_temp.m_amb, (uint8_t *) &temp_buffer,
	AMB_TEMP_MAX_PACKET_VALUE, BLE_AMBIENT_TEMP);
	check_ble_service_err_code(err_code);

	return NRF_SUCCESS;
}


uint32_t temp_timer_handler() {
	uint32_t err_code = NRF_SUCCESS;

	if (m_temp.IS_TEMP_ENABLED) {

		//Increment timer
		m_temp.timer_count++;

		//Clear timer if finished
		if(m_temp.timer_count >= m_temp.ticks)
			m_temp.timer_count = 0;

		//Start reading
		if ((m_temp.timer_count == 1) && !twi_busy) {

			//Get values, parse values and send them through BLE
			err_code = temp_values_handler();

			if (err_code != NRF_SUCCESS) {
				temp_printf("temp_values_handler() failed.\r\n");
				return err_code;
			}
		}
		else if ((m_temp.timer_count == 1) && twi_busy) {
			m_temp.timer_count--;
		}
	}

	return NRF_SUCCESS;
}


uint32_t temp_reset_configs(){
	uint32_t   err_code = NRF_SUCCESS;

	//Set service to default configurations
	err_code = ble_ambient_config_update(m_temp.m_amb, TEMP_INITIAL_CONFIG, BLE_AMBIENT_TEMP);

	if(err_code != NRF_SUCCESS){
		temp_printf("ble_ambient_config_update() for TEMP failed.\r\n");
		return err_code;
	}

	err_code = temp_configs_update();

	if(err_code != NRF_SUCCESS){
		temp_printf("temp_configs_update() failed.\r\n");
		return err_code;
	}

	//Reset sensor values
	uint8_t temp_buffer[AMB_TEMP_MAX_PACKET_VALUE];
	for(uint8_t i = 0; i < AMB_TEMP_MAX_PACKET_VALUE; i++) temp_buffer[i] = INVALID_SENSOR_VALUE;

	check_ble_service_err_code(ble_ambient_sensor_update(m_temp.m_amb, temp_buffer, AMB_TEMP_MAX_PACKET_VALUE, BLE_AMBIENT_TEMP));

	return NRF_SUCCESS;
}
#endif
