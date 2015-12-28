#include "mpu9x50_dmpdata.h"
#include "mpu9x50.h"

#include "mpu9x50_dmpdata.h"

uint8_t device_address;
static uint8_t sample_rate_divider = 124;
static uint8_t i2c_master_delay = 0;

static int16_t acc_x_off = 0;
static int16_t acc_y_off = 0;
static int16_t acc_z_off = 0;

static int16_t gyro_x_off = 0;
static int16_t gyro_y_off = 0;
static int16_t gyro_z_off = 0;

bool read_mult(char reg_addr, char* data, int length){
	//Write
    if(!twi_master_transfer((device_address << 1), (uint8_t *)&reg_addr, 1, TWI_ISSUE_STOP)){
    	APP_ERROR_CHECK_BOOL(false);
        return false;
    }

    //Read
    if(!twi_master_transfer((device_address << 1) | TWI_READ_BIT, (uint8_t *)data, length, TWI_ISSUE_STOP)){
    	APP_ERROR_CHECK_BOOL(false);
        return false;
    }

    //Wait for bus clear
    nrf_delay_us(TWI_MASTER_READ_DELAY);

    return true;
}

bool read(char reg_addr, char* data){
   return read_mult(reg_addr, data, 1);
}

bool readBits(char reg_addr, uint8_t bit_start, uint8_t length, uint8_t *data){
    char ret = 0;

    if(!read(reg_addr, &ret)){
        return false;
    }

    uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);
    ret &= mask;
    ret >>= (bit_start - length + 1);
    *data = ret;

    return true;
}

bool readBit(char reg_addr, uint8_t bit_start, uint8_t *data){
    return readBits(reg_addr, bit_start, 1, data);
}

//Utility Functions
bool getBit(char reg_addr, uint8_t bit){
    uint8_t data = 0;
    readBit(reg_addr, bit, &data);
    return (bool)data;
}

int8_t get8(char reg_addr){
    char data = 0;
    read(reg_addr, &data);
    return data;
}

int16_t get16(char reg_addr){
    char data[2] = {0, 0};
    read_mult(reg_addr, data, 2);
    return (data[0]<<8) + data[1];
}


int16_t get16L(char reg_addr){
    char data[2] = {0, 0};
    read_mult(reg_addr, data, 2);
    return (data[1]<<8) + data[0];
}

bool write_mult(char reg_addr, char* data, int length){
	uint8_t buffer[length + 1];

	buffer[0] = reg_addr;

	for(int i = 0; i < length; i++)
		buffer[i + 1] = data[i];

	if(!twi_master_transfer(device_address << 1, buffer, length + 1, TWI_ISSUE_STOP)){
    	APP_ERROR_CHECK_BOOL(false);
		return false;
	}

    return true;
}

bool write(char reg_addr, char data){
   return write_mult(reg_addr, &data, 1);
}


bool writeBits(char reg_addr, uint8_t bit_start, uint8_t length, uint8_t data){
    char ret = 0;

    if(!read(reg_addr, &ret)){
        return false;
    }

    uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);
    data <<= (bit_start - length + 1);

    data &= mask;
    ret &= ~(mask);
    ret |= data;

    return write(reg_addr, ret);
}

bool writeBit(char reg_addr, uint8_t bit, bool value){
    return writeBits(reg_addr, bit, 1, (uint8_t)value);
}

void set16(uint8_t reg_addr, int16_t value){
	char data[2];

	data[0] = (char)((value & 0xFF00) >> 8);
    data[1] = (char)(value & 0x00FF);

    write_mult(reg_addr, data, 2);
}


uint8_t mpu9x50_getDeviceID(){
    char ret = 0;
    //readBits(MPU6500_RA_WHO_AM_I, MPU6500_WHO_AM_I_BIT, MPU6500_WHO_AM_I_LENGTH, &ret);
    read(MPU6500_RA_WHO_AM_I, &ret);
    return ret;
}

bool mpu9x50_isReady(){
    //return (getDeviceID() == (device_address >> 1));
	return (mpu9x50_getDeviceID() == MPU6500_WHO_AM_I);
}

uint8_t ak89xx_getDeviceID(){
    char ret = 0;
    //readBits(MPU6500_RA_WHO_AM_I, MPU6500_WHO_AM_I_BIT, MPU6500_WHO_AM_I_LENGTH, &ret);
    read(AK8963_RA_WHO_AM_I, &ret);
    return ret;
}

bool ak89xx_isReady(){
    //return (getDeviceID() == (device_address >> 1));
	return (ak89xx_getDeviceID() == AK8963_WHO_AM_I);
}

void mpu9x50_init(uint8_t mpu_address){
	device_address = mpu_address;

    mpu9x50_sleep(false);
	mpu9x50_reset();
    nrf_delay_ms(20);//wait for reset

    mpu9x50_sleep(false);
    mpu9x50_clockSelect(MPU6500_CLOCK_PLL_XGYRO); //use the gyro clock as its more reliable
    mpu9x50_setGyroFullScaleRange(MPU6500_GYRO_FS_500);  //set range to +/- 500dps
    mpu9x50_setAccelFullScaleRange(MPU6500_ACCEL_FS_4);  //set range to +/- 4g

    //Set default offsets. They will be updated later
    mpu9x50_setAccOffsets(acc_x_off, acc_y_off, acc_z_off);
    mpu9x50_setGyroOffsets(gyro_x_off, gyro_y_off, gyro_z_off);

    //setI2CMasterClock(MPU6500_CLOCK_DIV_400);
    mpu9x50_setGyroscopeDigitalLowPassFilter(MPU6500_DLPF_BW_42); //Set gyro to 1Khz and 41Hz bandwidth
    											 //For this to work f_choice must be 0x00 (Gyroscope configuration register)
    											 //it is done on boot (it starts at 0x00)
    mpu9x50_setAccelerometerDigitalLowPassFilter(MPU6500_DLPF_BW_42); //Set acc to 1Khz and 41Hz bandwidth
    											 //For this to work accel_fchoice_b must be 0x00 (Accelerometer configuration register)
    											 //it is done on boot (it starts at 0x00)

    mpu9x50_setSampleRateDivider(sample_rate_divider);			         //Divide rate by (1 + SMPLRT_DIV)
    mpu9x50_initMagnetometer();

    mpu9x50_disableTemperatureSensor(true);
    mpu9x50_enableAccelerometer(false);
    mpu9x50_enableGyroscope(false);

    mpu9x50_setInterruptDataReadyEnable(false);
	mpu9x50_setEnableFifo(false);
	mpu9x50_setFifoReset(true);

    mpu9x50_sleep(true);
}

void mpu9x50_initMagnetometer(){
    //set up slave 0 to read the magnetometor data
	//mpu9x50_setWaitForExternalSensor(true);
    //read data
	mpu9x50_setI2cSlaveRW(0, true);
	mpu9x50_setI2cSlaveAddress(0, AK8963_RA_ADDRESS);
	mpu9x50_setI2cSlaveRegister(0, AK8963_RA_XOUT_L);
	mpu9x50_setI2cSlaveEnable(0, true);
	mpu9x50_setI2cSlaveTransactionLength(0, 6);


    //set up slave 1 to request a new magnetometor reading by writing 0x01 to 0xA
	mpu9x50_setI2cSlaveAddress(1, AK8963_RA_ADDRESS);
	mpu9x50_setI2cSlaveRegister(1, AK8963_RA_CTRL); //write to CNTL register
	mpu9x50_setI2cSlaveTransactionLength(1, 1); //1 byte
	mpu9x50_setI2cSlaveEnable(1, true);
	mpu9x50_setI2cSlaveDataOut(1, 0x11); //write 0x11 to register (enable 16 bit output, single measurement)

    //configure update rates
	mpu9x50_setI2cMasterDelay(i2c_master_delay); //set rate to (Fs / (1 + SMPLRT_DIV)) / (value + 1)
    				      	  	  //Fs is setted in CONFIG reg of the MPU
	mpu9x50_setI2cSlaveDelay(0, true); //enable the delay
	mpu9x50_setI2cSlaveDelay(1, true); //enable the delay
}

void mpu9x50_initDMP(){
	mpu9x50_reset();
    nrf_delay_ms(20);
    mpu9x50_sleep(false);

    mpu9x50_setMemoryBank(0x10, true, true);
    mpu9x50_setMemoryStartAddress(0x06);
//    debug.printf("Hardware Version: %d\r\n", readMemoryByte());

    mpu9x50_setMemoryBank(0, false, false);
    // check OTP bank valid
    //uint8_t otpValid = getOTPBankValid();
//    debug.printf("optValid: %d\r\n", otpValid);

    //Enabling interrupt latch, clear on any read, AUX bypass enabled
    write(MPU6500_RA_INT_PIN_CFG, 0x32);

    if (mpu9x50_writeMemoryBlock(dmpMemory, MPU6500_DMP_CODE_SIZE, 0 ,0, true)) {
 //       debug.printf("Success! DMP code written and verified.\r\n");
        if (mpu9x50_writeDMPConfigurationSet(dmpConfig, MPU6500_DMP_CONFIG_SIZE)) {
//            debug.printf("Success! DMP configuration written and verified.\r\n");
        	mpu9x50_setIntDMPEnabled(true);
        	mpu9x50_setInterruptFifoOverflowEnable(true);
        	mpu9x50_setSampleRateDivider(4);
        	mpu9x50_clockSelect(MPU6500_CLOCK_PLL_XGYRO);
        	mpu9x50_setAccelerometerDigitalLowPassFilter(MPU6500_DLPF_BW_42);
        	mpu9x50_setGyroscopeDigitalLowPassFilter(MPU6500_DLPF_BW_42);

        	mpu9x50_setGyroFullScaleRange(MPU6500_GYRO_FS_2000);

        	mpu9x50_setExternalFrameSync(MPU6500_EXT_SYNC_TEMP_OUT_L);
        	mpu9x50_setDMPConfig1(0x03);
        	mpu9x50_setDMPConfig2(0x00);

            unsigned char *update_ptr = (unsigned char*)dmpUpdates;
            mpu9x50_writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);
            update_ptr += update_ptr[2] + 3;
            mpu9x50_writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);

            mpu9x50_setFifoReset(true);

            update_ptr += update_ptr[2] + 3;
            mpu9x50_writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);
            update_ptr += update_ptr[2] + 3;
            mpu9x50_writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);

            write(MPU6500_RA_PWR_MGMT_2, 0x00);
            mpu9x50_setInterruptAnyReadClear(true);
            mpu9x50_setInterruptLatch(true);

            mpu9x50_setI2cSlaveRW(0, true);
            mpu9x50_setI2cSlaveAddress(0, 0x0C);
            mpu9x50_setI2cSlaveRegister(0, 1);
            mpu9x50_setI2cSlaveEnable(0, true);
            mpu9x50_setI2cSlaveTransactionLength(0, 10);

            //set up slave 1 to request a new magnetometor reading by writing 0x01 to 0xA
            mpu9x50_setI2cSlaveAddress(2, 0x0C);
            mpu9x50_setI2cSlaveRegister(2, 0x0A);
            mpu9x50_setI2cSlaveTransactionLength(2, 1);
            mpu9x50_setI2cSlaveEnable(2, true);
            mpu9x50_setI2cSlaveDataOut(2, 1);

            //configure update rates
            mpu9x50_setI2cMasterDelay(4);
            mpu9x50_setI2cSlaveDelay(0, true);
            mpu9x50_setI2cSlaveDelay(2, true);

            //Enable the aux i2c bus with MPU9250 as master
            mpu9x50_setI2cMasterEnable(true);

            write(MPU6500_RA_INT_PIN_CFG, 0x00);

            // enable I2C master mode and reset DMP/FIFO
            //DEBUG_PRINTLN(F("Enabling I2C master mode..."));
            write( MPU6500_RA_USER_CTRL, 0x20);
            //DEBUG_PRINTLN(F("Resetting FIFO..."));
            write(MPU6500_RA_USER_CTRL, 0x24);
            //DEBUG_PRINTLN(F("Rewriting I2C master mode enabled because...I don't know"));
            write(MPU6500_RA_USER_CTRL, 0x20);
            //DEBUG_PRINTLN(F("Enabling and resetting DMP/FIFO..."));
            write(MPU6500_RA_USER_CTRL, 0xE8);

            update_ptr += update_ptr[2] + 3;
            mpu9x50_writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);
            update_ptr += update_ptr[2] + 3;
            mpu9x50_writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);
            update_ptr += update_ptr[2] + 3;
            mpu9x50_writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);
            update_ptr += update_ptr[2] + 3;
            mpu9x50_writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);
            update_ptr += update_ptr[2] + 3;
            mpu9x50_writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);
            update_ptr += update_ptr[2] + 3;
            mpu9x50_writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);
            update_ptr += update_ptr[2] + 3;
            mpu9x50_writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);

            //read?
            update_ptr += update_ptr[2] + 3;
            //stalls?
            //readMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1]);


            update_ptr += update_ptr[2] + 3;
            mpu9x50_writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);
            update_ptr += update_ptr[2] + 3;
            mpu9x50_writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);
            update_ptr += update_ptr[2] + 3;
            mpu9x50_writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);
            update_ptr += update_ptr[2] + 3;
            mpu9x50_writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);
            update_ptr += update_ptr[2] + 3;
            mpu9x50_writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);

            int fifoCount = 0;
            while ((fifoCount = mpu9x50_getFifoCount()) < 46);
            uint8_t buffer[128];
            mpu9x50_getFifoBuffer((char *)buffer, fifoCount);
            mpu9x50_getInterruptStatus();

            update_ptr += update_ptr[2] + 3;
            mpu9x50_writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);

            fifoCount = 0;
            while ((fifoCount = mpu9x50_getFifoCount()) < 48);
            mpu9x50_getFifoBuffer((char *)buffer, fifoCount);
            mpu9x50_getInterruptStatus();
            fifoCount = 0;
            while ((fifoCount = mpu9x50_getFifoCount()) < 48);
            mpu9x50_getFifoBuffer((char *)buffer, fifoCount);
            mpu9x50_getInterruptStatus();

            update_ptr += update_ptr[2] + 3;
            mpu9x50_writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);

            mpu9x50_setDMPEnabled(false);

 //           debug.printf("finished\r\n");

        }
    }
}

void mpu9x50_setAccOffsets(int16_t x_off, int16_t y_off, int16_t z_off){
	mpu9x50_setXAccelOffset(x_off);
	mpu9x50_setYAccelOffset(y_off);
	mpu9x50_setZAccelOffset(z_off);

	acc_x_off = x_off;
	acc_y_off = y_off;
	acc_z_off = z_off;
}

void mpu9x50_setGyroOffsets(int16_t x_off, int16_t y_off, int16_t z_off){
	mpu9x50_setXGyroOffset(x_off);
	mpu9x50_setYGyroOffset(y_off);
	mpu9x50_setZGyroOffset(z_off);

	gyro_x_off = x_off;
	gyro_y_off = y_off;
	gyro_z_off = z_off;
}


bool mpu9x50_calibrate(mpu_cal_t *m_cal, uint8_t *sample){
	int16_t data[3];

	data[X_AXIS] = ((int16_t)(sample[0] << 8) +  (int16_t) sample[1]);
	data[Y_AXIS] = ((int16_t)(sample[2] << 8) +  (int16_t) sample[3]);
	data[Z_AXIS] = ((int16_t)(sample[4] << 8) +  (int16_t) sample[5]);

	for(int i = X_AXIS; i <= Z_AXIS; i++){
		if(m_cal->cal_type[i] != DONE){
			m_cal->cnt[i]++; //count 1 more sample
			m_cal->sum[i] += data[i] + m_cal->conv_values[i]; //add the current value

			if (m_cal->cnt[i] == CAL_ITER) {
				int32_t avg = m_cal->sum[i] / m_cal->cnt[i];

				if (avg > CAL_THRESHOLD && (m_cal->cal_type[i] == ROUGH)) {
					m_cal->cnt[i] = 0;
					m_cal->sum[i] = 0;

					m_cal->offsets[i] -= CAL_OFF_ROUGH;
				}

				else if (((avg < -CAL_THRESHOLD) &&  (m_cal->cal_type[i] == ROUGH)) || ((avg < -CAL_THRESHOLD) && (m_cal->cal_type[i] == MEDIUM))) {
					m_cal->cal_type[i] = MEDIUM;
					m_cal->cnt[i] = 0;
					m_cal->sum[i] = 0;

					m_cal->offsets[i] += CAL_OFF_MEDIUM;
				}

				else if (((avg > CAL_THRESHOLD) && (m_cal->cal_type[i] == MEDIUM)) || ((avg > CAL_THRESHOLD) && (m_cal->cal_type[i] == FINE))) {
					m_cal->cal_type[i] = FINE;
					m_cal->cnt[i] = 0;
					m_cal->sum[i]= 0;

					m_cal->offsets[i] -= CAL_OFF_FINE;
				}

				if(m_cal->cal_sensor_type == ACCELEROMETER){
					mpu9x50_setAccOffsets(m_cal->offsets[X_AXIS], m_cal->offsets[Y_AXIS], m_cal->offsets[Z_AXIS]);
				}
				else {
					mpu9x50_setGyroOffsets(m_cal->offsets[X_AXIS], m_cal->offsets[Y_AXIS], m_cal->offsets[Z_AXIS]);
				}
			}

		    else if (m_cal->cnt[i] > CAL_ITER) {
		    	m_cal->cal_type[i] = DONE;
				m_cal->cnt[i] = 0;
				m_cal->sum[i] = 0;
		    }
		}
	}

	if((m_cal->cal_type[X_AXIS] == DONE) && (m_cal->cal_type[Y_AXIS] == DONE) && (m_cal->cal_type[Z_AXIS] == DONE)){
	    for(uint8_t i = X_AXIS; i <= Z_AXIS; i++)
	    	m_cal->cal_type[i] = ROUGH; //reset the calibrations

		return true; //Calibration done!
	}

	return false;
}


//Private control variables!
static bool accelerometer_enabled = false;
static bool gyroscope_enabled = false;
static bool magnetometer_enabled = false;
static bool mpu_sleep = false;

void mpu9x50_fifoCtrl(){
	if(!mpu_sleep){
//		 mpu9x50_setInterruptDataReadyEnable(false);
//		 mpu9x50_setEnableFifo(false);
//     	 mpu9x50_setFifoReset(true);
//	}
//	else{
		mpu9x50_setInterruptDataReadyEnable(((accelerometer_enabled | gyroscope_enabled) | magnetometer_enabled));
		mpu9x50_setFifoReset(true);
		mpu9x50_setEnableFifo(((accelerometer_enabled | gyroscope_enabled) | magnetometer_enabled));
	}
}

//Since the power is cut off when the sensor is not needed,
//always init it before use!
static void check_init(){
	if(!accelerometer_enabled && !gyroscope_enabled && !magnetometer_enabled){
		mpu9x50_init(device_address);
	}
}

void mpu9x50_enableAccelerometer(bool enable){
	if(enable)
		check_init(); //Check for init

	mpu9x50_setSampleRateDivider(sample_rate_divider);
	mpu9x50_setI2cMasterDelay(i2c_master_delay);
	mpu9x50_setInterruptDataReadyEnable(false);
	mpu9x50_setEnableFifo(false);
	mpu9x50_setFifoReset(true);

	accelerometer_enabled = enable;

	mpu9x50_setStandbyAccX(!enable);
	mpu9x50_setStandbyAccY(!enable);
	mpu9x50_setStandbyAccZ(!enable);
    mpu9x50_setAccelFifo(enable);

    mpu9x50_fifoCtrl();
}

void mpu9x50_enableGyroscope(bool enable){
	if(enable)
		check_init(); //Check for init

    mpu9x50_setSampleRateDivider(sample_rate_divider);
    mpu9x50_setI2cMasterDelay(i2c_master_delay);
	mpu9x50_setInterruptDataReadyEnable(false);
	mpu9x50_setEnableFifo(false);
	mpu9x50_setFifoReset(true);

	gyroscope_enabled = enable;

	mpu9x50_setStandbyGyroX(!enable);
	mpu9x50_setStandbyGyroY(!enable);
	mpu9x50_setStandbyGyroZ(!enable);
    mpu9x50_setGyroFifo(enable);

    mpu9x50_fifoCtrl();
}

void mpu9x50_enableMagnetometer(bool enable){
	if(enable)
		check_init(); //Check for init

	mpu9x50_setSampleRateDivider(sample_rate_divider);
	mpu9x50_setI2cMasterDelay(i2c_master_delay);
	mpu9x50_setInterruptDataReadyEnable(false);
	mpu9x50_setEnableFifo(false);
	mpu9x50_setFifoReset(true);

	magnetometer_enabled = enable;

    mpu9x50_setSlave0Fifo(enable);
	mpu9x50_setI2cMasterEnable(true);

	mpu9x50_fifoCtrl();
}

void mpu9x50_setRate(uint8_t rate){
	sample_rate_divider = rate;
}

void mpu9x50_setMagRate(uint8_t rate){
	i2c_master_delay = rate;
}

//PWR_MGMT_1 Control Register
//*****************************/
void mpu9x50_reset(){
    writeBit(MPU6500_RA_PWR_MGMT_1, MPU6500_PWR1_DEVICE_RESET_BIT, true);
}


//Can't talk to mpu after putting it to sleep :(
void mpu9x50_sleep(bool state){
//	mpu_sleep = state;
//
//	if(!mpu_sleep){
//		//writeBit(MPU6500_RA_PWR_MGMT_1, MPU6500_PWR1_SLEEP_BIT, state);
//		write(MPU6500_RA_PWR_MGMT_1, 0x00);
//		mpu9x50_fifoCtrl();
//	}
//	else {
//		last_sleep_config = get8(MPU6500_RA_PWR_MGMT_1);
//		mpu9x50_fifoCtrl();
//		writeBit(MPU6500_RA_PWR_MGMT_1, MPU6500_PWR1_SLEEP_BIT, state);
//	}

	//Put IMU to sleep if none of the sensors are needed
	if(state){
		mpu9x50_fifoCtrl();
		writeBit(MPU6500_RA_PWR_MGMT_1, MPU6500_PWR1_SLEEP_BIT, state);
	}
}

/*
cycle between sleep mode and waking up to take a single sample of data from
active sensors at a rate determined by LP_WAKE_CTRL (register 108).
*/
void mpu9x50_cycleMode(bool state){
    writeBit(MPU6500_RA_PWR_MGMT_1, MPU6500_PWR1_CYCLE_BIT, state);
}
void mpu9x50_disableTemperatureSensor(bool state){
    writeBit(MPU6500_RA_PWR_MGMT_1, MPU6500_PWR1_TEMP_DIS_BIT, state);
}
void mpu9x50_clockSelect(uint8_t clk){
    writeBits(MPU6500_RA_PWR_MGMT_1, MPU6500_PWR1_CLKSEL_BIT, MPU6500_PWR1_CLKSEL_LENGTH, clk);
}

//PWR_MGMT_2 Control Register
//*****************************/
void mpu9x50_setCycleWakeFrequency(uint8_t freq){
    writeBits(MPU6500_RA_LP_ACCEL_ODR, MPU6500_PWR2_LP_WAKE_CTRL_BIT, MPU6500_PWR2_LP_WAKE_CTRL_LENGTH, freq);
}
void mpu9x50_setStandbyAccX(bool value){
    writeBit(MPU6500_RA_PWR_MGMT_2, MPU6500_PWR2_STBY_XA_BIT, value);
}
void mpu9x50_setStandbyAccY(bool value){
    writeBit(MPU6500_RA_PWR_MGMT_2, MPU6500_PWR2_STBY_YA_BIT, value);
}
void mpu9x50_setStandbyAccZ(bool value){
    writeBit(MPU6500_RA_PWR_MGMT_2, MPU6500_PWR2_STBY_ZA_BIT, value);
}
void mpu9x50_setStandbyGyroX( bool value){
    writeBit(MPU6500_RA_PWR_MGMT_2, MPU6500_PWR2_STBY_XG_BIT, value);
}
void mpu9x50_setStandbyGyroY( bool value){
    writeBit(MPU6500_RA_PWR_MGMT_2, MPU6500_PWR2_STBY_YG_BIT, value);
}
void mpu9x50_setStandbyGyroZ( bool value){
    writeBit(MPU6500_RA_PWR_MGMT_2, MPU6500_PWR2_STBY_ZG_BIT, value);
}

//SMPRT_DIV  Sample Rate Divider
//*****************************/
void mpu9x50_setSampleRateDivider(uint8_t value){
    write(MPU6500_RA_SMPLRT_DIV, value);
}

void mpu9x50_getSampleRateDivider(uint8_t *buff){
	read(MPU6500_RA_SMPLRT_DIV, (char *)buff);
}

//CONFIG
void mpu9x50_setExternalFrameSync(uint8_t value){
    writeBits(MPU6500_RA_CONFIG, MPU6500_CFG_EXT_SYNC_SET_BIT, MPU6500_CFG_EXT_SYNC_SET_LENGTH, value);
}
void mpu9x50_setGyroscopeDigitalLowPassFilter(uint8_t value){
    writeBits(MPU6500_RA_CONFIG, MPU6500_CFG_DLPF_CFG_BIT, MPU6500_CFG_DLPF_CFG_LENGTH, value);
}

void mpu9x50_setAccelerometerDigitalLowPassFilter(uint8_t value){
    writeBits(MPU6500_RA_ACCEL_CONFIG_2, MPU6500_CFG_DLPF_CFG_BIT, MPU6500_CFG_DLPF_CFG_LENGTH, value);
}

//GYRO_CONFIG
void mpu9x50_setGyroSelfTest(bool value){
    writeBit(MPU6500_RA_GYRO_CONFIG, 7, value); //X
    writeBit(MPU6500_RA_GYRO_CONFIG, 6, value); //Y
    writeBit(MPU6500_RA_GYRO_CONFIG, 5, value); //Z
}

void mpu9x50_setGyroFullScaleRange(uint8_t value){
    writeBits(MPU6500_RA_GYRO_CONFIG, MPU6500_GCONFIG_FS_SEL_BIT, MPU6500_GCONFIG_FS_SEL_LENGTH, value);
}

//ACCEL_CONFIG
void mpu9x50_setAccelSelfTest(bool value){
    writeBit(MPU6500_RA_ACCEL_CONFIG, MPU6500_ACONFIG_XA_ST_BIT, value);
    writeBit(MPU6500_RA_ACCEL_CONFIG, MPU6500_ACONFIG_YA_ST_BIT, value);
    writeBit(MPU6500_RA_ACCEL_CONFIG, MPU6500_ACONFIG_ZA_ST_BIT, value);
}
void mpu9x50_setAccelFullScaleRange(uint8_t value){
    writeBits(MPU6500_RA_ACCEL_CONFIG, MPU6500_ACONFIG_AFS_SEL_BIT , MPU6500_ACONFIG_AFS_SEL_LENGTH, value);
}

//GYRO_CAL
int16_t mpu9x50_getXGyroOffset(){
    return get16(MPU6500_RA_XG_OFFS_H);
}

int16_t mpu9x50_getYGyroOffset(){
	return get16(MPU6500_RA_YG_OFFS_H);
}

int16_t mpu9x50_getZGyroOffset(){
	return get16(MPU6500_RA_ZG_OFFS_H);
}

void mpu9x50_setXGyroOffset(int16_t value){
	set16(MPU6500_RA_XG_OFFS_H, value);
}

void mpu9x50_setYGyroOffset(int16_t value){
	set16(MPU6500_RA_YG_OFFS_H, value);
}

void mpu9x50_setZGyroOffset(int16_t value){
	set16(MPU6500_RA_ZG_OFFS_H, value);
}

//ACCEL_CAL
int16_t mpu9x50_getXAccelOffset(){
    return (get16(MPU6500_RA_XA_OFFS_H) >> 1); //last bit of the low byte is reserved
}

int16_t mpu9x50_getYAccelOffset(){
    return (get16(MPU6500_RA_YA_OFFS_H) >> 1); //last bit of the low byte is reserved
}

int16_t mpu9x50_getZAccelOffset(){
    return (get16(MPU6500_RA_ZA_OFFS_H) >> 1);
}

void mpu9x50_setXAccelOffset(int16_t value){
	//last bit of the low byte is reserved
	bool reserved_bit = (get16(MPU6500_RA_XA_OFFS_H) & 0x0001);

	if((value & 0x8000) > 0) //if value is negative
		value = (((value << 1) | 0x8000) | reserved_bit);
	else
		value = ((value << 1) | reserved_bit);

	set16(MPU6500_RA_XA_OFFS_H, value);
}

void mpu9x50_setYAccelOffset(int16_t value){
	//last bit of the low byte is reserved
	bool reserved_bit = (get16(MPU6500_RA_YA_OFFS_H) & 0x0001);

	if((value & 0x8000) > 0) //if value is negative
		value = (((value << 1) | 0x8000) | reserved_bit);
	else
		value = ((value << 1) | reserved_bit);

	set16(MPU6500_RA_YA_OFFS_H, value);
}

void mpu9x50_setZAccelOffset(int16_t value){
	//last bit of the low byte is reserved
	bool reserved_bit = (get16(MPU6500_RA_ZA_OFFS_H) & 0x0001);

	if((value & 0x8000) > 0) //if value is negative
		value = (((value << 1) | 0x8000) | reserved_bit);
	else
		value = ((value << 1) | reserved_bit);

	set16(MPU6500_RA_ZA_OFFS_H, value);
}

//FIFO_EN
void mpu9x50_setTemperatureFifo(bool value){
    writeBit(MPU6500_RA_FIFO_EN, MPU6500_TEMP_FIFO_EN_BIT, value);
}
void mpu9x50_setGyroFifo(bool value){
    writeBit(MPU6500_RA_FIFO_EN, MPU6500_XG_FIFO_EN_BIT, value);
    writeBit(MPU6500_RA_FIFO_EN, MPU6500_YG_FIFO_EN_BIT, value);
    writeBit(MPU6500_RA_FIFO_EN, MPU6500_ZG_FIFO_EN_BIT, value);
}
void mpu9x50_setAccelFifo(bool value){
    writeBit(MPU6500_RA_FIFO_EN, MPU6500_ACCEL_FIFO_EN_BIT, value);
}
void mpu9x50_setSlave2Fifo(bool value){
    writeBit(MPU6500_RA_FIFO_EN, MPU6500_SLV2_FIFO_EN_BIT, value);
}
void mpu9x50_setSlave1Fifo(bool value){
    writeBit(MPU6500_RA_FIFO_EN, MPU6500_SLV1_FIFO_EN_BIT, value);
}
void mpu9x50_setSlave0Fifo(bool value){
    writeBit(MPU6500_RA_FIFO_EN, MPU6500_SLV0_FIFO_EN_BIT, value);
}

//I2C_MST_CTRL
void mpu9x50_setMultiMaster(bool value){
    writeBit(MPU6500_RA_I2C_MST_CTRL, MPU6500_MULT_MST_EN_BIT, value);
}
void mpu9x50_setWaitForExternalSensor(bool value){
    writeBit(MPU6500_RA_I2C_MST_CTRL, MPU6500_WAIT_FOR_ES_BIT, value);
}
void mpu9x50_setSlave3Fifo(bool value){
    writeBit(MPU6500_RA_I2C_MST_CTRL, MPU6500_SLV_3_FIFO_EN_BIT, value);
}
void mpu9x50_setMasterStartStop(bool value){
    writeBit(MPU6500_RA_I2C_MST_CTRL, MPU6500_I2C_MST_P_NSR_BIT, value);
}
void mpu9x50_setI2CMasterClock(uint8_t value){
    writeBits(MPU6500_RA_I2C_MST_CTRL, MPU6500_I2C_MST_CLK_BIT, MPU6500_I2C_MST_CLK_LENGTH, value);
}

//I2C slaves 0 to 3
//I2C_SLV0_ADDR
void mpu9x50_setI2cSlaveRW(uint8_t slave_id, bool value){
    if(slave_id > 3)return;
    writeBit(MPU6500_RA_I2C_SLV0_ADDR + (slave_id * 3), MPU6500_I2C_SLV_RW_BIT, value);
}
void mpu9x50_setI2cSlaveAddress(uint8_t slave_id, uint8_t value){
    if(slave_id > 3)return;
    writeBits(MPU6500_RA_I2C_SLV0_ADDR + (slave_id * 3), MPU6500_I2C_SLV_ADDR_BIT, MPU6500_I2C_SLV_ADDR_LENGTH, value);
}
//I2C_SLV0_REG,
void mpu9x50_setI2cSlaveRegister(uint8_t slave_id, uint8_t value){
    if(slave_id > 3)return;
    write(MPU6500_RA_I2C_SLV0_REG + (slave_id * 3), value);
}
//I2C_SLV0_CTRL
void mpu9x50_setI2cSlaveEnable(uint8_t slave_id, bool value){
    if(slave_id > 3)return;
    writeBit(MPU6500_RA_I2C_SLV0_CTRL + (slave_id * 3), MPU6500_I2C_SLV_EN_BIT, value);
}
void mpu9x50_setI2cSlaveByteSwap(uint8_t slave_id, bool value){
    if(slave_id > 3)return;
    writeBit(MPU6500_RA_I2C_SLV0_CTRL + (slave_id * 3), MPU6500_I2C_SLV_BYTE_SW_BIT, value);
}
void mpu9x50_setI2cSlaveRegDisable(uint8_t slave_id, bool value){
    if(slave_id > 3)return;
    writeBit(MPU6500_RA_I2C_SLV0_CTRL + (slave_id * 3), MPU6500_I2C_SLV_REG_DIS_BIT, value);
}
void mpu9x50_setI2cSlaveByteGrouping(uint8_t slave_id, bool value){
    if(slave_id > 3)return;
    writeBit(MPU6500_RA_I2C_SLV0_CTRL + (slave_id * 3), MPU6500_I2C_SLV_GRP_BIT, value);
}
void mpu9x50_setI2cSlaveTransactionLength(uint8_t slave_id, uint8_t value){
    if(slave_id > 3)return;
    writeBits(MPU6500_RA_I2C_SLV0_CTRL + (slave_id * 3), MPU6500_I2C_SLV_LEN_BIT, MPU6500_I2C_SLV_LEN_LENGTH, value);
}
//I2C_SLV0_DO
void mpu9x50_setI2cSlaveDataOut(uint8_t slave_id, uint8_t value){
    if(slave_id > 3)return;
    write(MPU6500_RA_I2C_SLV0_DO + slave_id, value);
}
//I2C_MST_DELAY_CTRL
void mpu9x50_setI2cSlaveDelay(uint8_t slave_id, uint8_t value){
    writeBit(MPU6500_RA_I2C_MST_DELAY_CTRL, slave_id, value);
}
void mpu9x50_setI2cSlaveShadowDelay(uint8_t value){
    writeBit(MPU6500_RA_I2C_MST_DELAY_CTRL, 7, value);
}

//I2C slave4
//I2C_SLV4_ADDR
void mpu9x50_setI2cSlave4RW( bool value){
    writeBit(MPU6500_RA_I2C_SLV4_ADDR, MPU6500_I2C_SLV4_RW_BIT, value);
}
void mpu9x50_setI2cSlave4Address( uint8_t value){
    writeBits(MPU6500_RA_I2C_SLV4_ADDR, MPU6500_I2C_SLV4_ADDR_BIT, MPU6500_I2C_SLV4_ADDR_LENGTH, value);
}
//I2C_SLV4_REG,
void mpu9x50_setI2cSlave4Register(uint8_t value){
    write(MPU6500_RA_I2C_SLV4_REG, value);
}
//I2C_SLV4_DO
void mpu9x50_setI2cSlave4DataOut(uint8_t value){
    write(MPU6500_RA_I2C_SLV4_DO, value);
}

//I2C_SLV4_CTRL
void mpu9x50_setI2cSlave4Enable(bool value){
    writeBit(MPU6500_RA_I2C_SLV4_CTRL, MPU6500_I2C_SLV4_EN_BIT, value);
}

void mpu9x50_setI2cSlave4IntEnable(bool value){
    writeBit(MPU6500_RA_I2C_SLV4_CTRL, MPU6500_I2C_SLV4_INT_EN_BIT, value);
}

void mpu9x50_setI2cSlave4RegDisable(bool value){
    writeBit(MPU6500_RA_I2C_SLV4_CTRL, MPU6500_I2C_SLV4_REG_DIS_BIT, value);
}

void mpu9x50_setI2cMasterDelay(uint8_t value){
    writeBits(MPU6500_RA_I2C_SLV4_CTRL, MPU6500_I2C_SLV4_MST_DLY_BIT, MPU6500_I2C_SLV4_MST_DLY_LENGTH, value);
}

uint8_t mpu9x50_getI2cSlave4Di(){
    return get8(MPU6500_RA_I2C_SLV4_DI);
}

//I2C_MST_STATUS
bool mpu9x50_setI2cPassthrough(){
    return getBit(MPU6500_RA_I2C_MST_STATUS, MPU6500_MST_PASS_THROUGH_BIT);
}
bool mpu9x50_setI2cSlave4Done(){
    return getBit(MPU6500_RA_I2C_MST_STATUS, MPU6500_MST_I2C_SLV4_DONE_BIT);
}
bool mpu9x50_setI2cLostArbitration(){
    return getBit(MPU6500_RA_I2C_MST_STATUS, MPU6500_MST_I2C_LOST_ARB_BIT);
}
bool mpu9x50_setI2cSlave0Nack(){
    return getBit(MPU6500_RA_I2C_MST_STATUS, MPU6500_MST_I2C_SLV0_NACK_BIT);
}
bool mpu9x50_setI2cSlave1Nack(){
    return getBit(MPU6500_RA_I2C_MST_STATUS, MPU6500_MST_I2C_SLV1_NACK_BIT);
}
bool mpu9x50_setI2cSlave2Nack(){
    return getBit(MPU6500_RA_I2C_MST_STATUS, MPU6500_MST_I2C_SLV2_NACK_BIT);
}
bool mpu9x50_setI2cSlave3Nack(){
    return getBit(MPU6500_RA_I2C_MST_STATUS, MPU6500_MST_I2C_SLV3_NACK_BIT);
}
bool mpu9x50_setI2cSlave4Nack(){
   return getBit(MPU6500_RA_I2C_MST_STATUS, MPU6500_MST_I2C_SLV4_NACK_BIT);
}

//INT_PIN_CFG
void mpu9x50_setInterruptActiveLow(bool value){
    writeBit(MPU6500_RA_INT_PIN_CFG, MPU6500_INTCFG_INT_LEVEL_BIT, value);
}
void mpu9x50_setInterruptOpenDrain(bool value){
    writeBit(MPU6500_RA_INT_PIN_CFG, MPU6500_INTCFG_INT_OPEN_BIT, value);
}
void mpu9x50_setInterruptLatch(bool value){
    writeBit(MPU6500_RA_INT_PIN_CFG, MPU6500_INTCFG_LATCH_INT_EN_BIT, value);
}
void mpu9x50_setInterruptAnyReadClear(bool value){
    writeBit(MPU6500_RA_INT_PIN_CFG, MPU6500_INTCFG_INT_RD_CLEAR_BIT, value);
}
void mpu9x50_setFsyncInterruptActiveLow(bool value){
    writeBit(MPU6500_RA_INT_PIN_CFG, MPU6500_INTCFG_FSYNC_INT_LEVEL_BIT, value);
}
void mpu9x50_setFsyncInterruptEnable(bool value){
    writeBit(MPU6500_RA_INT_PIN_CFG, MPU6500_INTCFG_FSYNC_INT_EN_BIT, value);
}
void mpu9x50_setI2cAuxBypassEnable(bool value){
    writeBit(MPU6500_RA_INT_PIN_CFG, MPU6500_INTCFG_I2C_BYPASS_EN_BIT, value);
}

//INT_ENABLE
void mpu9x50_setInterruptFifoOverflowEnable(bool value){
    writeBit(MPU6500_RA_INT_ENABLE, MPU6500_INTERRUPT_FIFO_OFLOW_BIT, value);
}
void mpu9x50_setInterruptMasterEnable(bool value){
    writeBit(MPU6500_RA_INT_ENABLE, MPU6500_INTERRUPT_I2C_MST_INT_BIT, value);
}
void mpu9x50_setInterruptDataReadyEnable(bool value){
    writeBit(MPU6500_RA_INT_ENABLE, MPU6500_INTERRUPT_DATA_RDY_BIT, value);
}

//INT_STATUS
bool mpu9x50_getInterruptFifoOverflow(){
    return getBit(MPU6500_RA_INT_STATUS, MPU6500_INTERRUPT_FIFO_OFLOW_BIT);
}
bool mpu9x50_getInterruptMaster(){
    return getBit(MPU6500_RA_INT_STATUS, MPU6500_INTERRUPT_I2C_MST_INT_BIT);
}
bool mpu9x50_getInterruptDataReady(){
    return getBit(MPU6500_RA_INT_STATUS, MPU6500_INTERRUPT_DATA_RDY_BIT);
}
bool mpu9x50_getInterruptWOM(){
    return getBit(MPU6500_RA_INT_STATUS, MPU6500_INTERRUPT_MOT_BIT);
}


uint8_t mpu9x50_getInterruptStatus(){
    return get8(MPU6500_RA_INT_STATUS);
}

//SIGNAL_PATH_RESET
void mpu9x50_resetGyroSignalPath(){
    writeBit(MPU6500_RA_SIGNAL_PATH_RESET, MPU6500_PATHRESET_GYRO_RESET_BIT, true);
}
void mpu9x50_resetAccelSignalPath(){
    writeBit(MPU6500_RA_SIGNAL_PATH_RESET, MPU6500_PATHRESET_ACCEL_RESET_BIT, true);
}
void mpu9x50_resetTempSignalPath(){
    writeBit(MPU6500_RA_SIGNAL_PATH_RESET, MPU6500_PATHRESET_TEMP_RESET_BIT, true);
}

//USER_CTRL
void mpu9x50_setEnableFifo(bool value){
    writeBit(MPU6500_RA_USER_CTRL, MPU6500_USERCTRL_FIFO_EN_BIT, value);
}
void mpu9x50_setI2cMasterEnable(bool value){
    writeBit(MPU6500_RA_USER_CTRL, MPU6500_USERCTRL_I2C_MST_EN_BIT, value);
}
void mpu9x50_setFifoReset(bool value){
    writeBit(MPU6500_RA_USER_CTRL, MPU6500_USERCTRL_FIFO_RESET_BIT, value);
}
void mpu9x50_setI2cMasterReset(bool value){
    writeBit(MPU6500_RA_USER_CTRL, MPU6500_USERCTRL_I2C_MST_RESET_BIT, value);
}
void mpu9x50_setFullSensorReset(bool value){
    writeBit(MPU6500_RA_USER_CTRL, MPU6500_USERCTRL_SIG_COND_RESET_BIT, value);
}

//FIFO_COUNT_H and FIFO_COUNT_L
int16_t mpu9x50_getFifoCount(){
    return get16(MPU6500_RA_FIFO_COUNTH);
}

//FIFO_R_W
bool mpu9x50_getFifoBuffer(char* buffer, int16_t length){
    return read_mult(MPU6500_RA_FIFO_R_W, buffer, length);
}

//UNDOCUMENTED (again reimplemention from sparkfun github) can't find any origional documentation
// XG_OFFS_TC
uint8_t mpu9x50_getOTPBankValid() {
    return getBit(MPU6500_RA_XG_OFFS_TC, MPU6500_TC_OTP_BNK_VLD_BIT);
}

//INT_ENABLE
void mpu9x50_setIntPLLReadyEnabled(bool value) {
    writeBit( MPU6500_RA_INT_ENABLE, MPU6500_INTERRUPT_PLL_RDY_INT_BIT, value);
}
void mpu9x50_setIntDMPEnabled(bool value) {
    writeBit( MPU6500_RA_INT_ENABLE, MPU6500_INTERRUPT_DMP_INT_BIT, value);
}

// INT_STATUS
bool mpu9x50_getIntPLLReadyStatus() {
    return getBit( MPU6500_RA_INT_STATUS, MPU6500_INTERRUPT_PLL_RDY_INT_BIT);
}
bool mpu9x50_getIntDMPStatus() {
    return getBit( MPU6500_RA_INT_STATUS, MPU6500_INTERRUPT_DMP_INT_BIT);
}

// USER_CTRL
bool mpu9x50_getDMPEnabled() {
    return getBit(MPU6500_RA_USER_CTRL, MPU6500_USERCTRL_DMP_EN_BIT);
}
void mpu9x50_setDMPEnabled(bool value) {
    writeBit(MPU6500_RA_USER_CTRL, MPU6500_USERCTRL_DMP_EN_BIT, value);
}
void mpu9x50_resetDMP() {
    writeBit(MPU6500_RA_USER_CTRL, MPU6500_USERCTRL_DMP_RESET_BIT, true);
}

// BANK_SEL
void mpu9x50_setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank) {
    bank &= 0x1F;
    if (userBank){
        bank |= 0x20;
    }
    if (prefetchEnabled){
        bank |= 0x40;
    }
    write( MPU6500_RA_BANK_SEL, bank);
}

// MEM_START_ADDR
void mpu9x50_setMemoryStartAddress(uint8_t address) {
    write(MPU6500_RA_MEM_START_ADDR, address);
}

// MEM_R_W
uint8_t mpu9x50_readMemoryByte() {
    return get8(MPU6500_RA_MEM_R_W);
}
void mpu9x50_writeMemoryByte(uint8_t value) {
    write(MPU6500_RA_MEM_R_W, value);
}
void mpu9x50_readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address) {
	mpu9x50_setMemoryBank(bank, false, false);
	mpu9x50_setMemoryStartAddress(address);

    uint8_t chunkSize;
    for (uint16_t i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6500_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;
        //debug.printf("reading %d", chunkSize);
        // read the chunk of data as specified
        read_mult(MPU6500_RA_MEM_R_W, (char*)(data+i), chunkSize);
        //debug.printf("read");
        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) bank++;
            mpu9x50_setMemoryBank(bank, false, false);
            mpu9x50_setMemoryStartAddress(address);
        }
    }
}
bool mpu9x50_writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify) {
	mpu9x50_setMemoryBank(bank, false, false);
	mpu9x50_setMemoryStartAddress(address);
    uint8_t chunkSize;
    uint8_t *verifyBuffer = 0;
    uint8_t *progBuffer = 0;
    uint16_t i;

    if (verify) verifyBuffer = (uint8_t *)malloc(MPU6500_DMP_MEMORY_CHUNK_SIZE);
    for (i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6500_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;

        progBuffer = (uint8_t *)data + i;

        write_mult(MPU6500_RA_MEM_R_W, (char*)progBuffer, chunkSize);


        // verify data if needed
        if (verify && verifyBuffer) {
        	mpu9x50_setMemoryBank(bank, false, false);
        	mpu9x50_setMemoryStartAddress(address);
            read_mult(MPU6500_RA_MEM_R_W, (char*)verifyBuffer, chunkSize);
            if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) {
                free(verifyBuffer);
                //debug.printf("invalid(%d, %d)\r\n", bank, read_errors, write_errors);
                return false; // uh oh.
            }
        }

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) bank++;
            mpu9x50_setMemoryBank(bank, false, false);
            mpu9x50_setMemoryStartAddress(address);
        }
    }
    if (verify) free(verifyBuffer);
    return true;
}
bool mpu9x50_writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize) {
    uint8_t *progBuffer;
    uint8_t success, special;
    uint16_t i;

    // config set data is a long string of blocks with the following structure:
    // [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
    uint8_t bank, offset, length;
    for (i = 0; i < dataSize;) {
        bank = data[i++];
        offset = data[i++];
        length = data[i++];

        // write data or perform special action
        if (length > 0) {
            progBuffer = (uint8_t *)data + i;
            success = mpu9x50_writeMemoryBlock(progBuffer, length, bank, offset, true);
            i += length;
        } else {
            // special instruction
            // NOTE: this kind of behavior (what and when to do certain things)
            // is totally undocumented. This code is in here based on observed
            // behavior only, and exactly why (or even whether) it has to be here
            // is anybody's guess for now.
            special = data[i++];

            if (special == 0x01) {
                // enable DMP-related interrupts
                //setIntZeroMotionEnabled(true);
                //setIntFIFOBufferOverflowEnabled(true);
                //setIntDMPEnabled(true);
                write(MPU6500_RA_INT_ENABLE, 0x32);  // single operation
                success = true;
            } else {
                // unknown special command
                success = false;
            }
        }

        if (!success) {
            return false;
        }
    }
    return true;
}
// DMP_CFG_1
uint8_t mpu9x50_getDMPConfig1() {
   return get8(MPU6500_RA_DMP_CFG_1);

}
void mpu9x50_setDMPConfig1(uint8_t config) {
    write(MPU6500_RA_DMP_CFG_1, config);
}

// DMP_CFG_2
uint8_t mpu9x50_getDMPConfig2() {
    return get8(MPU6500_RA_DMP_CFG_2);

}
void mpu9x50_setDMPConfig2(uint8_t config) {
    write(MPU6500_RA_DMP_CFG_2, config);
}


