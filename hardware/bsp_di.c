#include "bsp_di.h"

#if CONFIG_MODULE_DI

void bsp_di_init(void)
{
        GPIO_InitTypeDef gpio_init;

        CONFIG_DI1_GPIO_CLK_FUN(CONFIG_DI1_GPIO_CLK, ENABLE);

        gpio_init.GPIO_Mode = GPIO_Mode_IPD;
        gpio_init.GPIO_Pin = CONFIG_DI1_GPIO_PIN;
        GPIO_Init(CONFIG_DI1_GPIO_PORT, &gpio_init);

}

#endif /* CONFIG_MDOULE_DI */
