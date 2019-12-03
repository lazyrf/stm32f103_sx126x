#include "delay.h"

static __IO uint32_t delay_times;


void mdelay(__IO uint32_t n_time)
{
	delay_times = n_time;
	while (delay_times != 0);
}

void delay_update(void)
{
	if (delay_times != 0) {
		delay_times--;
	}
}
