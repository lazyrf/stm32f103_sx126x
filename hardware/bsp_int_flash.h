#ifndef _BSP_INT_FLASH_H
#define _BSP_INT_FLASH_H

#include "config.h"

#define FLASH_PAGE_SIZE	((uint16_t) 0x400)

void bsp_int_flash_lock(void);
void bsp_int_flash_unlock(void);
int bsp_int_flash_erase(uint32_t start_addr, uint32_t nb_page);
int bsp_int_flash_write(__IO uint32_t *address, uint32_t *data, uint16_t data_length);
void bsp_int_flash_read(__IO uint32_t *address, uint32_t *buf, uint16_t len);

#endif /* _BSP_INT_FLASH_H */
