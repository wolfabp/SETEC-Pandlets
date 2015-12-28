/*
 * pr.c
 *
 *  Created on: Mar 10, 2015
 *      Author: João Oliveira
 */


#include "pr.h"

#if PR_ENABLED == 1

uint32_t pr_init(ble_ambient_t *m_amb_init){
	pr_printf("pr_init() \r\n");

	m_pr.timer_count           = 0;
	m_pr.m_amb                 = m_amb_init;
	m_pr.IS_PR_ENABLED         = false;

	return NRF_SUCCESS;
}


uint32_t pr_configs_update(){
	pr_printf("\r\nPressure Configurations Update\r\n");

	uint8_t configuration = (m_pr.m_amb)->pr_configuration;

	//Reset count
	m_pr.timer_count  = 0;

	//Sensors enable configs
	m_pr.IS_PR_ENABLED = (configuration & AMB_ENABLE_BIT);

	//Rate configs
	switch(((configuration & AMB_RATE_BITS) >> 5)){
		case 0b000:
			m_pr.ticks = msec_to_ticks(10000000); //0.0001 Hz
			break;
		case 0b001:
			m_pr.ticks = msec_to_ticks(1000000);  //0.001 Hz
			break;
		case 0b010:
			m_pr.ticks = msec_to_ticks(100000);   //0.01 Hz
			break;
		case 0b011:
			m_pr.ticks = msec_to_ticks(10000);    //0.1 Hz
			break;
		case 0b100:
			m_pr.ticks = msec_to_ticks(1000);     //1 Hz
			break;
//		case 0b101:
//			m_pr.ticks = msec_to_ticks(200);      //5 Hz
//			break;
		case 0b111:
		default: //If not recognized set max rate
//			m_pr.ticks = msec_to_ticks(200);      //5 Hz
			m_pr.ticks = msec_to_ticks(1000);      //1 Hz
			break;
	}

	pr_printf("pr_configs_update() Ok!\r\n\r\n");
	return NRF_SUCCESS;
}



uint32_t pr_values_handler() {
	uint32_t  err_code = NRF_SUCCESS;
	int32_t pr_buffer;

	err_code = bme280_read_pressure(&pr_buffer);

	if (err_code != NRF_SUCCESS) {
		pr_printf("pr: bme280_read_pressure() failed.\r\n");
		return err_code;
	}

	pr_printf("Pressure: %d\r\n", (int)pr_buffer);
	err_code = ble_ambient_sensor_update(m_pr.m_amb, (uint8_t *) &pr_buffer,
			AMB_PR_MAX_PACKET_VALUE, BLE_AMBIENT_PR);
	check_ble_service_err_code(err_code);

	return NRF_SUCCESS;
}


uint32_t pr_timer_handler() {
	uint32_t err_code = NRF_SUCCESS;

	if (m_pr.IS_PR_ENABLED) {

		//Increment timer
		m_pr.timer_count++;

		//Clear timer if finished
		if(m_pr.timer_count >= m_pr.ticks)
			m_pr.timer_count = 0;

		//Start reading
		if ((m_pr.timer_count == 1) && !twi_busy) {

			//Get values, parse values and send them through BLE
			err_code = pr_values_handler();

			if (err_code != NRF_SUCCESS) {
				pr_printf("pr_values_handler() failed.\r\n");
				return err_code;
			}
		}
		else if ((m_pr.timer_count == 1) && twi_busy) {
			m_pr.timer_count--;
		}
	}

	return NRF_SUCCESS;
}


uint32_t pr_reset_configs(){
	uint32_t   err_code = NRF_SUCCESS;

	//Set service to default configurations
	err_code = ble_ambient_config_update(m_pr.m_amb, PR_INITIAL_CONFIG, BLE_AMBIENT_PR);

	if(err_code != NRF_SUCCESS){
		pr_printf("ble_ambient_config_update() for PR failed.\r\n");
		return err_code;
	}

	err_code = pr_configs_update();

	if(err_code != NRF_SUCCESS){
		pr_printf("pr_configs_update() failed.\r\n");
		return err_code;
	}

	//Reset sensor values
	uint8_t pr_buffer[AMB_PR_MAX_PACKET_VALUE];
	for(uint8_t i = 0; i < AMB_PR_MAX_PACKET_VALUE; i++) pr_buffer[i] = INVALID_SENSOR_VALUE;

	check_ble_service_err_code(ble_ambient_sensor_update(m_pr.m_amb, pr_buffer, AMB_PR_MAX_PACKET_VALUE, BLE_AMBIENT_PR));

	return NRF_SUCCESS;
}

#endif /* PR_ENABLED */
