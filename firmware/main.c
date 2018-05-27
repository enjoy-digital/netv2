#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <irq.h>
#include <uart.h>
#include <time.h>
#include <generated/csr.h>
#include <generated/mem.h>
#include "flags.h"
#include <console.h>
#include <system.h>

#include "config.h"
#include "ci.h"
#include "ethernet.h"
#include "etherbone.h"
#include "processor.h"
#include "pattern.h"


static const unsigned char mac_addr[6] = {0x10, 0xe2, 0xd5, 0x00, 0x00, 0x00};
static const unsigned char ip_addr[4] = {192, 168, 1, 50};

int main(void)
{
	irq_setmask(0);
	irq_setie(1);
	uart_init();
#ifdef CSR_HDMI_OUT0_I2C_W_ADDR
	hdmi_out0_i2c_init();
#endif

	puts("\nNeTV2 CPU testing software built "__DATE__" "__TIME__);

	config_init();
	time_init();

#ifdef ETHMAC_BASE
	ethernet_init(mac_addr, ip_addr);
	etherbone_init();
#endif

	processor_init();
	processor_update();
	processor_start(config_get(CONFIG_KEY_RESOLUTION));

#ifdef CSR_DMA_WRITER_BASE
	// do it here to be sure values are stabilized when we'll use them
	dma_writer_slot0_base_write(MAIN_RAM_BASE + 0x1000000);
	dma_writer_slot1_base_write(MAIN_RAM_BASE + 0x2000000);
	dma_writer_length_write(2200*1125*4);
#endif

#ifdef CSR_DMA_READER_BASE
	// do it here to be sure values are stabilized when we'll use them
	dma_reader_slot0_base_write(MAIN_RAM_BASE + 0x1000000);
	dma_reader_slot1_base_write(MAIN_RAM_BASE + 0x2000000);
	dma_reader_length_write(2200*1125*4);
#endif

	ci_prompt();
	while(1) {
		processor_service();
		ci_service();
		pattern_service();
#ifdef ETHMAC_BASE
		ethernet_service();
#endif
	}

	return 0;
}
