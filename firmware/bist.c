#include <generated/csr.h>
#ifdef CSR_GENERATOR_BASE
#include "bist.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <uart.h>
#include <time.h>
#include <console.h>

#define TEST_BASE 0x20000
#define TEST_SIZE 64*1024*1024

unsigned int write_test(unsigned int base, unsigned int length) {
	unsigned int ticks;
	unsigned int speed;
	generator_reset_write(1);
	generator_reset_write(0);
	generator_base_write(base);
	generator_length_write(length);
	generator_start_write(1);
	while(generator_done_read() == 0);
	ticks = generator_ticks_read();
	speed = SYSTEM_CLOCK_FREQUENCY/ticks;
	speed = length*speed/1000000;
	speed = 8*speed;
	return speed;
}

unsigned int read_test(unsigned int base, unsigned int length) {
	unsigned int ticks;
	unsigned int speed;
	checker_reset_write(1);
	checker_reset_write(0);
	checker_base_write(base);
	checker_length_write(length);
	checker_start_write(1);
	while(checker_done_read() == 0);
	ticks = checker_ticks_read();
	speed = SYSTEM_CLOCK_FREQUENCY/ticks;
	speed = length*speed/1000000;
	speed = 8*speed;
	return speed;
}

void bist_test(void)
{
	unsigned int write_speed;
	unsigned int read_speed;
	unsigned int tested_length;
	unsigned int tested_errors;

	unsigned int i;

	/* init */
	i = 0;
	tested_length = 0;
	tested_errors = 0;
	for(;;) {
		/* exit on key pressed */
		if (readchar_nonblock())
			break;

		/* write */
		write_speed = write_test(TEST_BASE, TEST_SIZE);

		/* read */
		read_speed = read_test(TEST_BASE, TEST_SIZE);

		/* results */
		if (i%10 == 0)
			printf("WR_SPEED(Mbps) RD_SPEED(Mbps)  TESTED(MB)       ERRORS\n");
		i++;
		tested_length += TEST_SIZE/(1024*1024);
		tested_errors += checker_errors_read();
		printf("%14d %14d %11d %12d\n",
			write_speed,
			read_speed,
			tested_length,
			tested_errors);
	}
}

#endif