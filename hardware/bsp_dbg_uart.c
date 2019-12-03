#include "bsp_dbg_uart.h"
#include <stdio.h>

#if CONFIG_MODULE_DBG_UART

#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	  /* e.g. write a character to the USART1 and Loop until the end of transmission */
	USART_SendData(USART1, ch);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);

	return ch;
}

void bsp_dbg_uart_init(void)
{
	GPIO_InitTypeDef gpio_init;
	USART_InitTypeDef uart_init;

	CONFIG_DBG_UART_CLK_FUN(CONFIG_DBG_UART_CLK, ENABLE);
	CONFIG_DBG_UART_TX_GPIO_CLK_FUN(CONFIG_DBG_UART_TX_GPIO_CLK, ENABLE);
	CONFIG_DBG_UART_RX_GPIO_CLK_FUN(CONFIG_DBG_UART_RX_GPIO_CLK, ENABLE);

	gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_init.GPIO_Pin = CONFIG_DBG_UART_TX_GPIO_PIN;
	GPIO_Init(CONFIG_DBG_UART_TX_GPIO_PORT, &gpio_init);
	
	gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio_init.GPIO_Pin = CONFIG_DBG_UART_RX_GPIO_PIN;
	GPIO_Init(CONFIG_DBG_UART_RX_GPIO_PORT, &gpio_init);
	
	uart_init.USART_BaudRate = CONFIG_DBG_UART_BAUDRATE;
	uart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	uart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	uart_init.USART_Parity = USART_Parity_No;
	uart_init.USART_StopBits = USART_StopBits_1;
	uart_init.USART_WordLength = USART_WordLength_8b;
	USART_Init(CONFIG_DBG_UART, &uart_init);

	USART_Cmd(CONFIG_DBG_UART, ENABLE);
}

#endif /* CONFIG_MODULE_DBG_UART */

