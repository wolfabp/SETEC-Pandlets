#include "utils.h"

uint64_t msec_to_ticks(uint32_t msec){
	return msec/BASE_TIMER_FREQ;
}


/**@brief Function for checking ble_acl error codes.
 *
 * @param[in]   err_code   Error code containing information about what went wrong.
 */
void check_ble_service_err_code(uint32_t err_code){
	if(err_code == BLE_ERROR_NO_TX_BUFFERS)
		printf("BLE_ERROR_NO_TX_BUFFERS\r\n");

	if ((err_code != NRF_SUCCESS) &&  						  //value saved and notification sent
			(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) && //value saved and notification not enabled
				(err_code != BLE_ERROR_NO_TX_BUFFERS) &&      //still need to handle this...
					(err_code != NRF_ERROR_INVALID_STATE)){   //no-one is connected!
        APP_ERROR_HANDLER(err_code);
    }
}



#if SD_LOG
//Should only be used for error logging
//It will take around 40 ms to complete!
uint32_t log_to_sd(const char * filename, const char * data, uint32_t buffer_size){
	FIL file;       // File object

	//Open file
	if(f_open(&file, filename, FA_CREATE_NEW | FA_WRITE) != FR_OK){ //Could be that the file already exists
		if(f_open(&file, filename, FA_WRITE) != FR_OK) //Try to open file
			return SD_LOG_FAILED;
	}

	//Move to the end of the file and append file_name_size
	if(f_lseek(&file, f_size(&file) + buffer_size) != FR_OK)
		return SD_LOG_FAILED;

	//Move pointer to the beginning of the new data
	if(f_lseek(&file, f_size(&file) - buffer_size) != FR_OK)
		return SD_LOG_FAILED;

	UINT aux = 0;

	if((f_write(&file, data, buffer_size, &aux) != FR_OK))
		return SD_LOG_FAILED;

	f_close(&file);
}
#endif

#if ACCELEROMETER_SD_CARD_LOG
//TODO add append buffer!
uint32_t open_file(FIL *file, const char * filename){
	//Open file
	if(f_open(file, filename, FA_CREATE_NEW | FA_WRITE) != FR_OK){ //Could be that the file already exists
		if(f_open(file, filename, FA_WRITE) != FR_OK) //Try to open file
			return SD_OPEN_FILE_FAILED;
	}

	return SD_SUCCESS;
}

uint32_t extend_file(FIL *file, uint32_t size){
	//Move to the end of the file and append file_name_size
	if(f_lseek(file, f_size(file) + size) != FR_OK)
		return SD_WRITE_FILE_FAILED;

	//Move pointer to the beginning of the new data
	if(f_lseek(file, f_size(file) - size) != FR_OK)
		return SD_WRITE_FILE_FAILED;

	return SD_SUCCESS;
}

uint32_t write_to_sd(FIL *file, const char * data, uint32_t buffer_size){
	UINT aux = 0;

	if(f_write(file, data, buffer_size, &aux) != FR_OK)
		return SD_WRITE_FILE_FAILED;

	return SD_SUCCESS;
}

uint32_t close_file(FIL *file){
	if(f_close(file) != FR_OK)
		return SD_CLOSE_FILE_FAILED;

	return SD_SUCCESS;
}

#endif /* ACCELEROMETER_SD_CARD_LOG */


char * itoa_embedded (int32_t value, char *result, int base, const int value_type){
	switch(value_type){
		case I_16:
			if(value >> 15)
				value = (0xFFFF0000 | value);
		break;
	}

    // check that the base if valid
    if (base < 2 || base > 36) { *result = '\0'; return result; }

    char* ptr = result, *ptr1 = result, tmp_char;
    int tmp_value;

    do {
        tmp_value = value;
        value /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
    } while ( value );

    // Apply negative sign
    if ((tmp_value < 0) && !(value_type == U_32))  *ptr++ = '-';
    *ptr-- = '\0';
    while (ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
    }
    return result;
}
