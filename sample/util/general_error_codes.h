#ifndef GENERAL_ERROR_CODES_H__
#define GENERAL_ERROR_CODES_H__

#include "app_error.h"

/********************************************************
 * 				Peripherals Error codes
 ********************************************************/
//I2C Error codes
#define TWI_INIT_ERROR                   0x1001
#define TWI_REGISTER_READ_ERROR          0x1002
#define TWI_REGISTER_WRITE_ERROR         0x1003

//MAX11618 Error codes
#define MAX1161X_WRITE_FAILED            0x3001
#define MAX1161X_READ_FAILED             0x3002
#define MAX1161X_INIT_FAILED             0x3003
#define MAX1161X_INVALID_CHANNEL         0x3004
#define MAX1161X_READ_CHANNEL_FAILED     0x3005

//MAX17048 Error codes
#define MAX17048_WRITE_FAILED            0x4001
#define MAX17048_READ_FAILED             0x4002
#define MAX17048_INIT_FAILED             0x4003
#define MAX17048_CLEAR_FAILED            0x4004
#define MAX17048_READ_SOC_FAILED         0x4005
#define MAX17048_READ_STATUS_FAILED      0x4006
#define MAX17048_READ_VOLT_FAILED        0x4007

//BME Error codes
#define BME280_WRITE_FAILED              0x2001
#define BME280_READ_FAILED               0x2002
#define BME280_INIT_FAILED               0x2003

//GPIO Error codes
#define GPIO_INVALID_PIN_ERROR           0x6001
#define GPIO_MAX_PIN_REACHED_ERROR       0x6003
#define GPIO_INIT_TIMER_ERROR            0x6004
#define GPIO_TIMER_ERROR                 0x6005

#define INDI_INIT_FAILED                 0x7001


/********************************************************
 * 				Sensor Error codes
 ********************************************************/

//Pressure Error codes
#define PR_INIT_ERROR                    0x15001
#define PR_GET_VALUES_ERROR              0x15002

//Temperature Error codes
#define TEMP_INIT_ERROR                  0x16001
#define TEMP_START_READ_ERROR            0x16002
#define TEMP_GET_VALUES_ERROR            0x16003

#endif /* GENERAL_ERROR_CODES_H__ */
