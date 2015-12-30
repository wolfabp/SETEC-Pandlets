/*
 * boron_config.h
 *
 *  Created on: Apr 21, 2015
 *      Author: João Oliveira
 */

#ifndef BORON_CONFIG_H_
#define BORON_CONFIG_H_

#define LED_RED                           28                              		  // Red LED pin
#define LED_GREEN                         29   									  // Green LED pin

	/*DC-DC converters */
#define EN_PIN_3V3 						  30                                      // Output pin to enable 3V3 rail

	/* QI Charger */
#define IND_CH_CTRL                       9                                       // NTC pin. If not used, pull-down.
#define IND_CH_EN   					  10									  // Enable or disable wired charging. Put it to 0 to enable both chargings.
#define IND_CH_STAT 				      11           							  // Active low when wireless charging is active.
#define IND_CH_STAT_BITMASK               (1 << IND_CH_STAT)                      // Bitmask for the GPIOTE interrupt. Each bit is a pin, IND_CH_STAT interrupt.

	/* USB Charger */
#define USB_CH_STAT   					  21									  // LOW if USB charging is on.
#define USB_CH_STAT_BITMASK               (1 << USB_CH_STAT)                      // Bitmask for the GPIOTE interrupt. Each bit is a pin, USB_CH_STAT interrupt.

	/* Fuel Gauge */
#define GAUGE_ADDRESS                     0x36                                    // Gauge I2C address
#define GAUGE_INT                         6                                      // Used for Gauge interrupts
#define GAUGE_INT_BITMASK                 (1 << GAUGE_INT)                        // Bitmask for the GPIOTE interrupt. Each bit is a pin, INT_GAUGE interrupt.
#define GAUGE_TIMER                       60000								      // ms between reads

	/* ADC Mux */
#define ADC_MUX_ADDRESS                   0x34                                    //I2C ADC Mux address

 	/* GPIO Extender */
#define GPIO_EXT_ADDRESS          	      0x73                               	  //I2C PCA9538 address

	/* BME */
#define BME_ADDRESS                       0x76                                    //I2C BMX280 address

	/* MPU */
#define MPU_ADDRESS                       0x68                                    //I2C MPU address
#define MPU_INT_PIN                       14
#define MPU_INT_PIN_BITMASK               (1 << MPU_INT_PIN)                      //GPIOTE bitmask to signalize MOTION_INT_PIN. Each bit represents one pin.

	/* TWI */
#define TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER 1                                      //Defines I2C Clock pin
#define TWI_MASTER_CONFIG_DATA_PIN_NUMBER  0                                      //Defines I2C Data pin

	/* GPIOS */
#define GPIO_1								2
#define GPIO_2								3
#define GPIO_3								4
#define GPIO_4								7

#define BOARD_FIRMWARE_VERSION              4001									  //FW v. 4.X.X.BORON

/****************************************************************************/
/************************ BLE Services and Sensors **************************/
/****************************************************************************/

/*============================= Ambient Service =============================*/
#define AMBIENT_SERVICE_ENABLED            1    				                  // Enables Ambient service

#if AMBIENT_SERVICE_ENABLED

#define TEMP_ENABLED                        1      							      //Enables External Temperature sensor
#define PR_ENABLED                          1      							      //Enables Pressure sensor
#define HUM_ENABLED							1      							      //Enables Humidity sensor 

#endif /* AMBIENT_SERVICE_ENABLED */

#endif /* BORON_CONFIG_H_ */
