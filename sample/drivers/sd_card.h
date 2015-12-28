#ifndef SD_CARD_H__
#define SD_CARD_H__

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "nrf.h"
#include "nrf_delay.h"
#include "app_gpiote.h"
#include "nrf_gpio.h"
#include "spi_fhp_master.h"

/* MMC/SD commands (SPI mode) */
#define CMD0	(0)			/* GO_IDLE_STATE */
#define CMD1	(1)			/* SEND_OP_COND */
#define	ACMD41	(0x80+41)	/* SEND_OP_COND (SDC) */
#define CMD8	(8)			/* SEND_IF_COND */
#define CMD9	(9)			/* SEND_CSD */
#define CMD10	(10)		/* SEND_CID */
#define CMD12	(12)		/* STOP_TRANSMISSION */
#define CMD13	(13)		/* SEND_STATUS */
#define ACMD13	(0x80+13)	/* SD_STATUS (SDC) */
#define CMD16	(16)		/* SET_BLOCKLEN */
#define CMD17	(17)		/* READ_SINGLE_BLOCK */
#define CMD18	(18)		/* READ_MULTIPLE_BLOCK */
#define CMD23	(23)		/* SET_BLOCK_COUNT */
#define	ACMD23	(0x80+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24	(24)		/* WRITE_BLOCK */
#define CMD25	(25)		/* WRITE_MULTIPLE_BLOCK */
#define CMD32	(32)		/* ERASE_ER_BLK_START */
#define CMD33	(33)		/* ERASE_ER_BLK_END */
#define CMD38	(38)		/* ERASE */
#define CMD55	(55)		/* APP_CMD */
#define CMD58	(58)		/* READ_OCR */


/**@brief Init the SPI and the CS pin.
 */
void spi_init();


/**@brief Send bytes to the card
 *
 * @param[in]   data   Pointer to the data to be sent.
 * @param[in]   bytes  Number of bytes to be sent.
 */
void send_mmc(const uint8_t *data, uint32_t bytes);


/**@brief Receive bytes from the card
 *
 * @param[in]   data   Pointer to where to save the data received.
 * @param[in]   bytes  Number of bytes to receive.
 */
void receive_mmc(uint8_t *data, uint32_t bytes);


/**@brief Wait for card ready
 *
 * @return 1 for OK, 0 for timeout
 */
uint8_t wait_ready (void);


/**@brief Select the card and wait for ready.
 *
 * @return 1 for OK, 0 for timeout
 */
uint8_t select(void);


/**@brief Deselect the card and release SPI bus.
 */
void deselect(void);


/**@brief Receive a data packet from the card.
 *
 * @param[in]   buff    Pointer to where to save the packet received.
 * @param[in]   btr     Number of bytes to receive.
 *
 * @return 1 for OK, 0 for failed
 */
uint8_t receive_datablock(uint8_t *buff, uint32_t btr);


/**@brief Send a data packet to the card
 *
 * @param[in]   buff    Pointer to the data to be sent.
 * @param[in]   token   Data/Stop token
 *
 * @return 1 for OK, 0 for failed
 */
uint8_t send_datablock(const uint8_t *buff, uint32_t token);


/**@brief Send a command packet to the card
 *
 * @param[in]   cmd     Command byte.
 * @param[in]   token   Command argument.
 *
 * @return Command response (bit7 == 1: Send failed)
 */
uint8_t send_cmd(uint8_t cmd, uint32_t arg);

#endif /* SD_CARD_H__ */



