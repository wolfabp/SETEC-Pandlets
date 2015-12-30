/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef SPI_MASTER_CONFIG_H
#define SPI_MASTER_CONFIG_H

#define SPI_OPERATING_FREQUENCY   SPI_FREQUENCY_FREQUENCY_M4  /*!< Slave clock frequency. */

/*  SPI0 */
#define SPI_PSELSCK0              25   /*!< GPIO pin number for SPI clock (note that setting this to 31 will only work for loopback purposes as it not connected to a pin) */
#define SPI_PSELMOSI0             22   /*!< GPIO pin number for Master Out Slave In    */
#define SPI_PSELMISO0             23   /*!< GPIO pin number for Master In Slave Out    */
#define SPI_PSELSS0               24   /*!< GPIO pin number for Slave Select           */

/*  SPI1 */
#define SPI_PSELSCK1              30   /*!< GPIO pin number for SPI clock              */
#define SPI_PSELMOSI1             30   /*!< GPIO pin number for Master Out Slave In    */
#define SPI_PSELMISO1             30   /*!< GPIO pin number for Master In Slave Out    */
#define SPI_PSELSS1               30   /*!< GPIO pin number for Slave Select           */

#define TIMEOUT_COUNTER           0x3000UL  /*!< timeout for getting rx bytes from slave */

#define MMCSD_PIN_SELECT          24 /* CS PIM */

#endif /* SPI_MASTER_CONFIG_H */
