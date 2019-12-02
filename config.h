#ifndef _CONFIG_H
#define _CONFIG_H

#include "hardware/stm32f10x_conf.h"

/* Clock source */
#define CONFIG_RCC_HSI		0
#define CONFIG_RCC_HSE		1
#define CONFIG_RCC_PLL		0
#if (CONFIG_RCC_HSI) && (CONFIG_RCC_HSE) && (CONFIG_RCC_PLL)
#error "Please choose one SYSCLK source"
#endif

#endif /* _CONFIG_H */
