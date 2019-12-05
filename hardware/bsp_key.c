#include "bsp_key.h"
#include "clock.h"

void EXTI9_5_IRQHandler(void)
{
        uint32_t delay = 0xFFFF;

        if (EXTI_GetITStatus(CONFIG_KEY1_EXTI_LINE) != RESET) {
                while (--delay);
                if (GPIO_ReadInputDataBit(CONFIG_KEY1_GPIO_PORT, CONFIG_KEY1_GPIO_PIN) == 0) {
                        key1_handler();
                }
                EXTI_ClearITPendingBit(CONFIG_KEY1_EXTI_LINE);
        } else if (EXTI_GetITStatus(CONFIG_KEY2_EXTI_LINE) != RESET) {
                while (--delay);
                if (GPIO_ReadInputDataBit(CONFIG_KEY2_GPIO_PORT, CONFIG_KEY2_GPIO_PIN) == 0) {
                        key2_handler();
                }
                EXTI_ClearITPendingBit(CONFIG_KEY2_EXTI_LINE);
        }
}

void bsp_key_init(void)
{
        GPIO_InitTypeDef gpio_init;
        EXTI_InitTypeDef exti_init;

        CONFIG_KEY1_GPIO_CLK_FUNC(CONFIG_KEY1_GPIO_CLK, ENABLE);
        CONFIG_KEY2_GPIO_CLK_FUNC(CONFIG_KEY2_GPIO_CLK, ENABLE);

        gpio_init.GPIO_Mode = GPIO_Mode_IPU;
        gpio_init.GPIO_Pin = CONFIG_KEY1_GPIO_PIN;
        GPIO_Init(CONFIG_KEY1_GPIO_PORT, &gpio_init);

        gpio_init.GPIO_Pin = CONFIG_KEY2_GPIO_PIN;
        GPIO_Init(CONFIG_KEY2_GPIO_PORT, &gpio_init);

        GPIO_EXTILineConfig(CONFIG_KEY1_EXTI_PORTSOURCE, CONFIG_KEY1_EXTI_PINSOURCE);
        GPIO_EXTILineConfig(CONFIG_KEY2_EXTI_PORTSOURCE, CONFIG_KEY2_EXTI_PINSOURCE);

        NVIC_SetPriority(CONFIG_KEY1_IRQ, 3);
        NVIC_EnableIRQ(CONFIG_KEY1_IRQ);

        exti_init.EXTI_Line = CONFIG_KEY1_EXTI_LINE;
        exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
        exti_init.EXTI_Trigger = EXTI_Trigger_Falling;
        exti_init.EXTI_LineCmd = ENABLE;
        EXTI_Init(&exti_init);

        exti_init.EXTI_Line = CONFIG_KEY2_EXTI_LINE;
        EXTI_Init(&exti_init);
}
