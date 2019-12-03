#ifndef _BSP_LED_H
#define _BSP_LED_H

#include "config.h"

#define LED_ON(port, pin)	do { \
		port->BRR = pin; \
	} while (0);

#define LED_OFF(port, pin)	do { \
		port->BSRR = pin; \
	} while (0);

#define LED_TOGGLE(port, pin)	do { \
		port->ODR ^= pin; \
	} while (0);


void bsp_led_init(void);

#endif /* _BSP_LED_H */
