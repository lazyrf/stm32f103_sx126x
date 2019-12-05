#ifndef _BSP_DI_H
#define _BSP_DI_H

#include "config.h"

#if CONFIG_MODULE_DI

#define DI1_DATA()         GPIO_ReadInputDataBit(CONFIG_DI1_GPIO_PORT, CONFIG_DI1_GPIO_PIN)

void bsp_di_init(void);

#endif /* COFNIG_MODULE_DI */

#endif /* _BSP_DI_H */
