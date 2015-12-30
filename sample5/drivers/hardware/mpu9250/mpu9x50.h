#ifndef MPU9x50_H__
#define MPU9x50_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "mpu9x50_registers.h"
#include "app_error.h"
#include "twi_master.h"
#include "twi_master_config.h"
#include "nrf_delay.h"

//Just renamed array positions
#define X_AXIS                 0
#define Y_AXIS                 1
#define Z_AXIS                 2

//Default offset values used for calibration
#define ACC_DEFAULT_OFF      8000
#define GYRO_DEFAULT_OFF     100


//Calibration constants
#define CAL_ITER               128 //Number of samples per cycle of calibration
#define CAL_THRESHOLD          5   //Maximum accepted error

#define CAL_OFF_ROUGH          100 //Offset change in rough calibration
#define CAL_OFF_MEDIUM         10  //Offset change in medium calibration
#define CAL_OFF_FINE           1   //Offset change in fine calibration


typedef enum {
	ACCELEROMETER,
	GYROSCOPE
} cal_axis_type;

typedef enum {
	ROUGH,
	MEDIUM,
	FINE,
	DONE
} cal_type;

/**@brief Calibration object. */
typedef struct
{
    cal_axis_type cal_sensor_type;

    //3 axis, 0 is X, 1 is Y and 2 is Z
    cal_type cal_type[3];   //holds the type of calibration (ROUGH, MEDIUM, FINE)
    int32_t sum[3];         //holds the sum of the sensor values
    int32_t cnt[3];         //holds the current number of samples
    int16_t offsets[3];     //holds the offsets
    int16_t conv_values[3]; //holds the desired end values

} mpu_cal_t;


//User functions
uint8_t mpu9x50_getDeviceID();
uint8_t ak8963_getDeviceID();
bool mpu9x50_isReady();
bool ak8963_isReady();
void mpu9x50_init(uint8_t mpu_address);
void mpu9x50_initMagnetometer();
void mpu9x50_initDMP();
void mpu9x50_enableAccelerometer(bool enable);
void mpu9x50_enableGyroscope(bool enable);
void mpu9x50_enableMagnetometer(bool enable);
void mpu9x50_setRate(uint8_t rate);
void mpu9x50_setMagRate(uint8_t rate);
void mpu9x50_setAccOffsets(int16_t x_off, int16_t y_off, int16_t z_off);
void mpu9x50_setGyroOffsets(int16_t x_off, int16_t y_off, int16_t z_off);
bool mpu9x50_calibrate(mpu_cal_t *m_cal, uint8_t *sample);

//PWR_MGMT_1 Control Register
void mpu9x50_reset();
void mpu9x50_sleep(bool state);
void mpu9x50_cycleMode(bool state);
void mpu9x50_disableTemperatureSensor(bool state);
void mpu9x50_clockSelect(uint8_t clk);

//PWR_MGMT_2 Control Register
void mpu9x50_setCycleWakeFrequency(uint8_t value);
void mpu9x50_setStandbyAccX( bool value );
void mpu9x50_setStandbyAccY( bool value );
void mpu9x50_setStandbyAccZ( bool value );
void mpu9x50_setStandbyGyroX( bool value );
void mpu9x50_setStandbyGyroY( bool value );
void mpu9x50_setStandbyGyroZ( bool value );

//SMPRT_DI Sample Rate Divider
void mpu9x50_setSampleRateDivider(uint8_t value);
void mpu9x50_getSampleRateDivider(uint8_t *buff);

//CONFIG
void mpu9x50_setExternalFrameSync(uint8_t value);
void mpu9x50_setGyroscopeDigitalLowPassFilter(uint8_t value);
void mpu9x50_setAccelerometerDigitalLowPassFilter(uint8_t value);

//GYRO_CONFIG
void mpu9x50_setGyroSelfTest(bool value);
void mpu9x50_setGyroFullScaleRange(uint8_t value);

//ACCEL_CONFIG
void mpu9x50_setAccelSelfTest(bool value);
void mpu9x50_setAccelFullScaleRange(uint8_t value);

//GYRO_CAL
int16_t mpu9x50_getXGyroOffset();
int16_t mpu9x50_getYGyroOffset();
int16_t mpu9x50_getZGyroOffset();

void mpu9x50_setXGyroOffset(int16_t value);
void mpu9x50_setYGyroOffset(int16_t value);
void mpu9x50_setZGyroOffset(int16_t value);

//ACCEL_CAL
int16_t mpu9x50_getXAccelOffset();
int16_t mpu9x50_getYAccelOffset();
int16_t mpu9x50_getZAccelOffset();

void mpu9x50_setXAccelOffset(int16_t value);
void mpu9x50_setYAccelOffset(int16_t value);
void mpu9x50_setZAccelOffset(int16_t value);

//FIFO_EN
void mpu9x50_setTemperatureFifo(bool value);
void mpu9x50_setGyroFifo(bool value);
void mpu9x50_setAccelFifo(bool value);
void mpu9x50_setSlave2Fifo(bool value);
void mpu9x50_setSlave1Fifo(bool value);
void mpu9x50_setSlave0Fifo(bool value);

//I2C_MST_CTRL
void mpu9x50_setMultiMaster(bool value);
void mpu9x50_setWaitForExternalSensor(bool value);
void mpu9x50_setSlave3Fifo(bool value);
void mpu9x50_setMasterStartStop(bool value);
void mpu9x50_setI2CMasterClock(uint8_t value);

//I2C_SLV0_ADDR
void mpu9x50_setI2cSlaveRW(uint8_t slave_id, bool value);
void mpu9x50_setI2cSlaveAddress(uint8_t slave_id, uint8_t value);
//I2C_SLV0_REG,
void mpu9x50_setI2cSlaveRegister(uint8_t slave_id, uint8_t value);
//I2C_SLV0_CTRL
void mpu9x50_setI2cSlaveEnable(uint8_t slave_id, bool value);
void mpu9x50_setI2cSlaveByteSwap(uint8_t slave_id, bool value);
void mpu9x50_setI2cSlaveRegDisable(uint8_t slave_id, bool value);
void mpu9x50_setI2cSlaveByteGrouping(uint8_t slave_id, bool value);
void mpu9x50_setI2cSlaveTransactionLength(uint8_t slave_id, uint8_t value);
//I2C_SLV0_DO
void mpu9x50_setI2cSlaveDataOut(uint8_t slave_id, uint8_t value);
//I2C_MST_DELAY_CTRL
void mpu9x50_setI2cSlaveDelay(uint8_t slave_id, uint8_t value);
void mpu9x50_setI2cSlaveShadowDelay(uint8_t value)    ;
//Slave4 is different
void mpu9x50_setI2cSlave4RW( bool value);
void mpu9x50_setI2cSlave4Address( uint8_t value);
void mpu9x50_setI2cSlave4Register(uint8_t value);
void mpu9x50_setI2cSlave4DataOut(uint8_t value);
void mpu9x50_setI2cSlave4Enable(bool value);
void mpu9x50_setI2cSlave4IntEnable(bool value);
void mpu9x50_setI2cSlave4RegDisable(bool value);
void mpu9x50_setI2cMasterDelay(uint8_t value);
uint8_t mpu9x50_getI2cSlave4Di();

//I2C_MST_STATUS
bool mpu9x50_setI2cPassthrough();
bool mpu9x50_setI2cSlave4Done();
bool mpu9x50_setI2cLostArbitration();
bool mpu9x50_setI2cSlave0Nack();
bool mpu9x50_setI2cSlave1Nack();
bool mpu9x50_setI2cSlave2Nack();
bool mpu9x50_setI2cSlave3Nack();
bool mpu9x50_setI2cSlave4Nack();

//INT_PIN_CFG
void mpu9x50_setInterruptActiveLow(bool value);
void mpu9x50_setInterruptOpenDrain(bool value);
void mpu9x50_setInterruptLatch(bool value);
void mpu9x50_setInterruptAnyReadClear(bool value);
void mpu9x50_setFsyncInterruptActiveLow(bool value);
void mpu9x50_setFsyncInterruptEnable(bool value);
void mpu9x50_setI2cAuxBypassEnable(bool value);

//INT_ENABLE
void mpu9x50_setInterruptFifoOverflowEnable(bool value);
void mpu9x50_setInterruptMasterEnable(bool value);
void mpu9x50_setInterruptDataReadyEnable(bool value);

//INT_STATUS
bool mpu9x50_getInterruptFifoOverflow();
bool mpu9x50_getInterruptMaster();
bool mpu9x50_getInterruptDataReady();
bool mpu9x50_getInterruptWOM();
uint8_t mpu9x50_getInterruptStatus();

//SIGNAL_PATH_RESET
void mpu9x50_resetGyroSignalPath();
void mpu9x50_resetAccelSignalPath();
void mpu9x50_resetTempSignalPath();

//USER_CTRL
void mpu9x50_setEnableFifo(bool value);
void mpu9x50_setI2cMasterEnable(bool value);
void mpu9x50_setFifoReset(bool value);
void mpu9x50_setI2cMasterReset(bool value);
void mpu9x50_setFullSensorReset(bool value);

//FIFO_COUNT_H and FIFO_COUNT_L
int16_t mpu9x50_getFifoCount();

//FIFO_R_W
bool mpu9x50_getFifoBuffer(char* buffer, int16_t length);

//UNDOCUMENTED
// XG_OFFS_TC
uint8_t mpu9x50_getOTPBankValid();

//INT_ENABLE
void mpu9x50_setIntPLLReadyEnabled(bool value);
void mpu9x50_setIntDMPEnabled(bool value);

// INT_STATUS
bool mpu9x50_getIntPLLReadyStatus();
bool mpu9x50_getIntDMPStatus();

// USER_CTRL
bool mpu9x50_getDMPEnabled();
void mpu9x50_setDMPEnabled(bool value);
void mpu9x50_resetDMP();

// BANK_SEL
void mpu9x50_setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank);

// MEM_START_ADDR
void mpu9x50_setMemoryStartAddress(uint8_t address);

// MEM_R_W register
uint8_t mpu9x50_readMemoryByte();
void mpu9x50_mpu9x50_writeMemoryByte(uint8_t value);
void mpu9x50_readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address);
bool mpu9x50_writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify);
bool mpu9x50_writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize);

// DMP_CFG_1
uint8_t mpu9x50_getDMPConfig1();
void mpu9x50_setDMPConfig1(uint8_t config);

// DMP_CFG_2
uint8_t mpu9x50_getDMPConfig2();
void mpu9x50_setDMPConfig2(uint8_t config);


#endif /* MPU9x50_H__ */
