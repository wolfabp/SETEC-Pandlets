#ifndef __UTILS_H__
#define __UTILS_H__

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "board_config.h"

#if GENERAL_DEBUG
#include "SEGGER_RTT.h"
#endif

#if GENERAL_DEBUG
#define printf RTT_PRINTF
#else
#define printf RTT_NOP
#endif

#define RTT_PRINTF(...) \
do { \
     char str[128]; \
     sprintf(str, __VA_ARGS__); \
     SEGGER_RTT_WriteString(0, str); \
} while(0)

#define RTT_NOP(...) \
do { \
} while(0)


#if SD_LOG
#include "ff.h"                                                                         //SD card functions and includes

//TODO test append buffer!
//#define FILE_APPEND_BUFFER 0x1D448 //1 minute of data @ 333 Hz (333 * 60 * 6)
#endif

#include "ble.h"
#include "app_error.h"

#define SD_SUCCESS               0x00000000
#define SD_LOG_FAILED            0x00010000
#define SD_OPEN_FILE_FAILED      0x00010001
#define SD_WRITE_FILE_FAILED     0x00010002
#define SD_GET_FILE_FAILED       0x00010003
#define SD_CLOSE_FILE_FAILED     0x00010004

//itoa_embedded types
#define U_8                      0x00
#define U_16                     0x01
#define U_32                     0x02
#define I_16                     0x03
#define I_32                     0x04

uint64_t msec_to_ticks(uint32_t msec);
char * itoa_embedded (int32_t value, char *result, int base, const int value_type);
void check_ble_service_err_code(uint32_t err_code);


#if SD_LOG
uint32_t log_to_sd(const char * filename, const char * data, uint32_t buffer_size);
#endif

#endif /* __UTILS_H__ */
