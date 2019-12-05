#ifndef _BSP_OLED_H
#define _BSP_OLED_H

#include "config.h"

#if CONFIG_MODULE_OLED

// 0: SPI 4-wired mode, 1: parallel 8080 mode
#define OLED_MODE	0

#define OLED_SIZE		16
#define OLED_X_LEVEL_L		0x00
#define OLED_X_LEVEL_H		0x10
#define OLED_MAX_COL		128
#define OLED_MAX_ROW		64
#define OLED_MAX_BRIGHTNESS	0xFF
#define OLED_X_WIDTH		128
#define OLED_Y_WIDTH		64

#define OLED_SCLK_CLR()	GPIO_ResetBits(CONFIG_OLED_SCLK_PORT, CONFIG_OLED_SCLK_PIN)
#define OLED_SCLK_SET()	GPIO_SetBits(CONFIG_OLED_SCLK_PORT, CONFIG_OLED_SCLK_PIN)

#define OLED_SDIN_CLR()	GPIO_ResetBits(CONFIG_OLED_SDIN_PORT, CONFIG_OLED_SDIN_PIN)
#define OLED_SDIN_SET()	GPIO_SetBits(CONFIG_OLED_SDIN_PORT, CONFIG_OLED_SDIN_PIN)

#define OLED_RST_CLR()	GPIO_ResetBits(CONFIG_OLED_RST_PORT, CONFIG_OLED_RST_PIN)
#define OLED_RST_SET()	GPIO_SetBits(CONFIG_OLED_RST_PORT, CONFIG_OLED_RST_PIN)

#define OLED_DC_CLR()	GPIO_ResetBits(CONFIG_OLED_DC_PORT, CONFIG_OLED_DC_PIN)
#define OLED_DC_SET()	GPIO_SetBits(CONFIG_OLED_DC_PORT, CONFIG_OLED_DC_PIN)

#define OLED_CMD	0
#define OLED_DATA	1

void bsp_oled_init(void);
void bsp_oled_show_string(uint8_t x, uint8_t y, char *chr);

#endif /* CONFIG_MODULE_OLED */

#endif /* _BSP_OLED_H */
