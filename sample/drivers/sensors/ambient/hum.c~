/*
 * hum.c
 *
 *  Created on: Apr 17, 2015
 *      Author: João Oliveira
 */


#include "hum.h"

#if HUM_ENABLED == 1

uint32_t hum_init(ble_ambient_t *m_amb_init){
	hum_printf("hum_init() \r\n");

	m_hum.timer_count           = 0;
	m_hum.m_amb                 = m_amb_init;
	m_hum.IS_HUM_ENABLED        = false;

	return NRF_SUCCESS;
}


uint32_t hum_configs_update(){
	hum_printf("\r\nHumidity Configurations Update\r\n");

	uint8_t configuration = (m_hum.m_amb)->hum_configuration;

	//Reset count
	m_hum.timer_count  = 0;

	//Sensors enable configs
	m_hum.IS_HUM_ENABLED = (configuration & AMB_ENABLE_BIT);

	//Rate configs
	switch(((configuration & AMB_RATE_BITS) >> 5)){
		case 0b000:
			m_hum.ticks = msec_to_ticks(10000000); //0.0001 Hz
			break;
		case 0b001:
			m_hum.ticks = msec_to_ticks(1000000);  //0.001 Hz
			break;
		case 0b010:
			m_hum.ticks = msec_to_ticks(100000);   //0.01 Hz
			break;
		case 0b011:
			m_hum.ticks = msec_to_ticks(10000);    //0.1 Hz
			break;
		case 0b111:
		default: //If not recognized set max rate
			m_hum.ticks = msec_to_ticks(2000);     //0.5 Hz
			break;
	}

	hum_printf("hum_configs_update() Ok!\r\n\r\n");
	return NRF_SUCCESS;
}



uint32_t hum_values_handler() {
	uint32_t  err_code = NRF_SUCCESS;
	int32_t hum_buffer;

	err_code = bme280_read_humidity(&hum_buffer);
	if (err_code != NRF_SUCCESS) {
		hum_printf("hum: bme280_read_pressure() failed.\r\n");
		return err_code;
	}

	hum_printf("Humidity: %d\r\n", (int)hum_buffer);

	err_code = ble_ambient_sensor_update(m_hum.m_amb, (uint8_t *) &hum_buffer,
			AMB_HUM_MAX_PACKET_VALUE, BLE_AMBIENT_HUM);
	check_ble_service_err_code(err_code);

	return NRF_SUCCESS;
}


uint32_t hum_timer_handler() {
	uint32_t err_code = NRF_SUCCESS;

	if (m_hum.IS_HUM_ENABLED) {

		//Increment timer
		m_hum.timer_count++;

		//Clear timer if finished
		if(m_hum.timer_count >= m_hum.ticks)
			m_hum.timer_count = 0;

		//Start reading
		if ((m_hum.timer_count == 1) && !twi_busy) {

			//Get values, parse values and send them through BLE
			err_code = hum_values_handler();

			if (err_code != NRF_SUCCESS) {
				hum_printf("hum_values_handler() failed.\r\n");
				return err_code;
			}
		}
		else if ((m_hum.timer_count == 1) && twi_busy) {
			m_hum.timer_count--;
		}
	}

	return NRF_SUCCESS;
}


uint32_t hum_reset_configs(){
	uint32_t   err_code = NRF_SUCCESS;

	//Set service to default configurations
	err_code = ble_ambient_config_update(m_hum.m_amb, HUM_INITIAL_CONFIG, BLE_AMBIENT_HUM);

	if(err_code != NRF_SUCCESS){
		hum_printf("ble_ambient_config_update() for HUM failed.\r\n");
		return err_code;
	}

	err_code = hum_configs_update();

	if(err_code != NRF_SUCCESS){
		hum_printf("hum_configs_update() failed.\r\n");
		return err_code;
	}

	//Reset sensor values
	uint8_t hum_buffer[AMB_HUM_MAX_PACKET_VALUE];
	for(uint8_t i = 0; i < AMB_HUM_MAX_PACKET_VALUE; i++) hum_buffer[i] = INVALID_SENSOR_VALUE;

	check_ble_service_err_code(
			ble_ambient_sensor_update(m_hum.m_amb, hum_buffer, AMB_HUM_MAX_PACKET_VALUE, BLE_AMBIENT_HUM));

	return NRF_SUCCESS;
}

#endif /* HUM_ENABLED */
