#ifndef _BSP_CRC_H
#define _BSP_CRC_H

#include "config.h"

void bsp_crc_init(void);
uint32_t bsp_crc_calc(uint32_t *data, uint32_t len);

#endif /* _BSP_CRC_H */
