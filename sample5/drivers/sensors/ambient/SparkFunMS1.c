#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "max1161x.h"
#include "SparkFunMS1.h"

bool SparkFunMS1_read(uint16_t *molhadinho){
	
	uint8_t moist[2];
	moist[0]=moist[1]=0;
	max1161x_read_channel(3, moist);
	(*molhadinho) = (moist[1] << 4 )| (moist[0] & 0b00001111);		
	printf(" Humidade := %d\n", (*molhadinho));
	nrf_delay_ms(5);
return NRF_SUCCESS;

}

