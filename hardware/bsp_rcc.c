#include "bsp_rcc.h"

int bsp_rcc_init(void)
{
#if CONFIG_RCC_HSI
	__IO uint32_t hsi_startup_status = 0;

	RCC_DeInit();

	/* Enable HSI */
	RCC_HSICmd(ENABLE);

	/* Wait HSI ready */
	hsi_startup_status = RCC->CR & RCC_CR_HSIRDY;
	if (hsi_startup_status == RCC_CR_HSIRDY) {
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		/* Flash 2 wait state */
		FLASH_SetLatency(FLASH_Latency_2);

		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);

		/* PCLK2 = HCLK */
		RCC_PCLK2Config(RCC_HCLK_Div1);

		/* PCLK1 = HCLK / 2 */
		RCC_PCLK1Config(RCC_HCLK_Div2);

		/* Select HSI as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);

		/* Wait till PLL is used as system clock source */
		while (RCC_GetSYSCLKSource() != 0x00) {
		}

		SystemCoreClockUpdate();
	} else {
		while (1);
	}
#elif CONFIG_RCC_HSE
	__IO uint32_t hse_startup_status = 0;

	RCC_DeInit();

	/* Eanble HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	hse_startup_status = RCC_WaitForHSEStartUp();
	if (hse_startup_status == SUCCESS) {
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		/* Flash 2 wait state */
		FLASH_SetLatency(FLASH_Latency_2);

		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);

		/* PCLK2 = HCLK */
		RCC_PCLK2Config(RCC_HCLK_Div1);

		/* PCLK1 = HCLK */
		RCC_PCLK1Config(RCC_HCLK_Div1);

		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE);

		/* Wait till PLL is used as system clock source */
		while (RCC_GetSYSCLKSource() != 0x04) {
		}

		SystemCoreClockUpdate();
	} else {
		while (1);
	}
#elif CONFIG_RCC_PLL
#endif

#if 0 /* Use contiki, move to clock.c */
	if (SysTick_Config(SystemCoreClock / 1000)) {
		while (1);
	}
#endif

	return 0;
}

void bsp_rcc_mco_config(void)
{
	GPIO_InitTypeDef gpio_init;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	gpio_init.GPIO_Pin = GPIO_Pin_8;
	gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio_init);

	RCC_MCOConfig(RCC_MCO_SYSCLK);
}

