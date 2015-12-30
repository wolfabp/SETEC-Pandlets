#include "sd_card.h"

static uint32_t *sdcard_spi_addr;

/**@brief Init the SPI and the CS pin.
 */
void spi_init(){
	sdcard_spi_addr = spi_master_init(SPI0, SPI_MODE0, 0);
	nrf_gpio_cfg_output(MMCSD_PIN_SELECT);
	nrf_gpio_pin_set(MMCSD_PIN_SELECT);
}

/**@brief Send bytes to the card
 *
 * @param[in]   data   Pointer to the data to be sent.
 * @param[in]   bytes  Number of bytes to be sent.
 */
void send_mmc(const uint8_t *data, uint32_t bytes){
	uint8_t rx[bytes];

	spi_master_tx_rx(sdcard_spi_addr, bytes, data, rx);
}


/**@brief Receive bytes from the card
 *
 * @param[in]   data   Pointer to where to save the data received.
 * @param[in]   bytes  Number of bytes to receive.
 */
void receive_mmc(uint8_t *data, uint32_t bytes){
	uint8_t tx[bytes];

	for(uint32_t i = 0; i < bytes; i++)
		tx[i] = 0xFF;

	spi_master_tx_rx(sdcard_spi_addr, bytes, tx, data);
}


/**@brief Wait for card ready
 *
 * @return 1 for OK, 0 for timeout
 */
uint8_t wait_ready (void)	/* 1:OK, 0:Timeout */
{
	uint8_t d;
	uint32_t tmr;

	for (tmr = 500000; tmr; tmr--) {	/* Wait for ready in timeout of 500ms */
		receive_mmc(&d, 1);
		if (d == 0xFF){
			break;
		}
		nrf_delay_us(1);
	}

	return tmr ? 1 : 0;
}


/**@brief Select the card and wait for ready.
 *
 * @return 1 for OK, 0 for timeout
 */
uint8_t select(void){
	uint8_t d;

	nrf_gpio_pin_clear(MMCSD_PIN_SELECT);
	receive_mmc(&d, 1);	/* Dummy clock (force DO enabled) */

	if (wait_ready())
		return 1;	/* OK */

	deselect();
	return 0;			/* Failed */
}


/**@brief Deselect the card and release SPI bus.
 */
void deselect(void){
	uint8_t d;

	nrf_gpio_pin_set(MMCSD_PIN_SELECT);
	receive_mmc(&d, 1);	/* Dummy clock (force DO hi-z for multiple slave SPI) */
}


/**@brief Receive a data packet from the card.
 *
 * @param[in]   buff    Pointer to where to save the packet received.
 * @param[in]   btr     Number of bytes to receive.
 *
 * @return 1 for OK, 0 for failed
 */
uint8_t receive_datablock(uint8_t *buff, uint32_t btr){
	uint8_t d[2];
	uint32_t tmr;

	for (tmr = 100000; tmr; tmr--) {	/* Wait for data packet in timeout of 100ms */
		receive_mmc(d, 1);
		if (d[0] != 0xFF) break;
		nrf_delay_us(1);
	}
	if (d[0] != 0xFE) return 0;		/* If not valid data token, return with error */

	receive_mmc(buff, btr);			/* Receive the data block into buffer */
	receive_mmc(d, 2);				/* Discard CRC */

	return 1;						/* Return with success */
}


/**@brief Send a data packet to the card
 *
 * @param[in]   buff    Pointer to the data to be sent.
 * @param[in]   token   Data/Stop token
 *
 * @return 1 for OK, 0 for failed
 */
uint8_t send_datablock(const uint8_t *buff, uint32_t token){
	uint8_t d[2];

	if (!wait_ready()) return 0;

	d[0] = token;
	send_mmc(d, 1);					/* Send a token */
	if (token != 0xFD) {			/* Is it data token? */
		send_mmc(buff, 512);		/* Send the 512 byte data block to MMC */
		receive_mmc(d, 2);			/* Send dummy CRC (0xFF,0xFF) */
		receive_mmc(d, 1);	     	/* Receive data response */
		if ((d[0] & 0x1F) != 0x05)	/* If not accepted, return with error */
			return 0;
	}

	return 1;
}


/**@brief Send a command packet to the card
 *
 * @param[in]   cmd     Command byte.
 * @param[in]   token   Command argument.
 *
 * @return Command response (bit7 == 1: Send failed)
 */
uint8_t send_cmd(uint8_t cmd, uint32_t arg){
	uint8_t n, d, buf[6];

	if (cmd & 0x80) {				/* ACMD<n> is the command sequense of CMD55-CMD<n> */
		cmd &= 0x7F;
		n = send_cmd(CMD55, 0);
		if (n > 1) return n;
	}

	/* Select the card and wait for ready except to stop multiple block read */
	if ((cmd != CMD12) && ((cmd != CMD0))) {
		deselect();
		if (!select()) return 0xFF;
	}

	/* Send a command packet */
	buf[0] = 0x40 | cmd;			/* Start + Command index */
	buf[1] = (uint8_t)(arg >> 24);	/* Argument[31..24] */
	buf[2] = (uint8_t)(arg >> 16);	/* Argument[23..16] */
	buf[3] = (uint8_t)(arg >> 8);	/* Argument[15..8] */
	buf[4] = (uint8_t)arg;			/* Argument[7..0] */
	n = 0x01;						/* Dummy CRC + Stop */
	if (cmd == CMD0) n = 0x95;		/* (valid CRC for CMD0(0)) */
	if (cmd == CMD8) n = 0x87;		/* (valid CRC for CMD8(0x1AA)) */
	buf[5] = n;
	send_mmc(buf, 6);

	/* Receive command response */
	if (cmd == CMD12) receive_mmc(&d, 1);	/* Skip a stuff byte when stop reading */
	n = 10;								    /* Wait for a valid response in timeout of 10 attempts */
	do
		receive_mmc(&d, 1);
	while ((d & 0x80) && --n);

	return d;						/* Return with the response value */
}



