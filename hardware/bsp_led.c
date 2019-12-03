#include "bsp_led.h"

void bsp_led_init(void)
{
	GPIO_InitTypeDef gpio_init;

	CONFIG_LED_1_GPIO_CLK_FUN(CONFIG_LED_1_GPIO_CLK, ENABLE);
	gpio_init.GPIO_Pin = CONFIG_LED_1_GPIO_PIN;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CONFIG_LED_1_GPIO_PORT, &gpio_init);
}
