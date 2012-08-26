#include <18F458.h>
#device ADC=10
#include <stdlib.h>
#include <math.h>

int8 flash_mfg_id[4];


void flash_read_mfg_id() {
		output_low(PIN_C2);
		delay_cycles(20);
		spi_write(0x9F);
		flash_mfg_id[0] = spi_read(0);
		flash_mfg_id[1] = spi_read(0);
		flash_mfg_id[2] = spi_read(0);
		flash_mfg_id[3] = spi_read(0);
		delay_cycles(20);
		output_high(PIN_C2);
}