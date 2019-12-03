#include "bsp_sx126x.h"
#include <stdio.h>
#include "m_err.h"

static __IO uint32_t timeout = SX126X_SPI_LONG_TIMEOUT;

dio_irq_handler_fn dio_rq_cb;

void CONFIG_SX126X_DIO1_EXTI_ISR(void)
{
        //printf("[%s, %d]\r\n", __func__, __LINE__);
        if (EXTI_GetITStatus(CONFIG_SX126X_DIO1_EXTI_LINE) != RESET) {
                EXTI_ClearITPendingBit(CONFIG_SX126X_DIO1_EXTI_LINE);

                dio_rq_cb(NULL);
        }
}

void CONFIG_SX126X_DIO2_EXTI_ISR(void)
{
        //printf("[%s, %d]\r\n", __func__, __LINE__);
        if (EXTI_GetITStatus(CONFIG_SX126X_DIO2_EXTI_LINE) != RESET) {
                EXTI_ClearITPendingBit(CONFIG_SX126X_DIO2_EXTI_LINE);

                dio_rq_cb(NULL);
        }
}

static void _sx126x_err_cb(int code)
{
        printf("[%s, %d] code = %d\r\n", __func__, __LINE__, code);
}

uint8_t bsp_sx126x_read_byte(void)
{
        return bsp_sx126x_send_byte(DUMMY_BYTE);
}

uint8_t bsp_sx126x_send_byte(uint8_t byte)
{
        timeout = SX126X_SPI_SHORT_TIMEOUT;
        while (SPI_I2S_GetFlagStatus(CONFIG_SX126X_SPI, SPI_I2S_FLAG_TXE) == RESET) {
                if ((timeout--) == 0) _sx126x_err_cb(M_ERR_TIMEOUT);
        }

        SPI_I2S_SendData(CONFIG_SX126X_SPI, byte);

        timeout = SX126X_SPI_SHORT_TIMEOUT;
        while (SPI_I2S_GetFlagStatus(CONFIG_SX126X_SPI, SPI_I2S_FLAG_RXNE) == RESET) {
                if ((timeout--) == 0) _sx126x_err_cb(M_ERR_TIMEOUT);
        }

        return SPI_I2S_ReceiveData(CONFIG_SX126X_SPI);
}

#if 0
static void _nvic_config(void)
{
	NVIC_InitTypeDef nvic_init;

	nvic_init.NVIC_IRQChannel = CONFIG_SX126X_DIO1_EXTI_IRQ;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 5;
	nvic_init.NVIC_IRQChannelSubPriority = 0;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);

	nvic_init.NVIC_IRQChannel = CONFIG_SX126X_DIO2_EXTI_IRQ;
	NVIC_Init(&nvic_init);
}
#endif

void bsp_sx126x_dio1_interrupt_enable(void)
{
        NVIC_EnableIRQ(CONFIG_SX126X_DIO1_EXTI_IRQ);
}

void bsp_sx126x_dio1_interrupt_disable(void)
{
        NVIC_DisableIRQ(CONFIG_SX126X_DIO1_EXTI_IRQ);
}

void bsp_sx126x_dio2_interrupt_enable(void)
{
        NVIC_EnableIRQ(CONFIG_SX126X_DIO2_EXTI_IRQ);
}

void bsp_sx126x_dio2_interrupt_disable(void)
{
        NVIC_DisableIRQ(CONFIG_SX126X_DIO2_EXTI_IRQ);
}

void bsp_sx126x_init(dio_irq_handler_fn cb)
{
        GPIO_InitTypeDef gpio_init;
        SPI_InitTypeDef spi_init;
        EXTI_InitTypeDef exti_init;

        dio_rq_cb = cb;

        // 1. Enable SPI1 clock ----------------------------------------
        CONFIG_SX126X_CLK_FUN(CONFIG_SX126X_CLK, ENABLE);

        // 2. Enable GPIO clock -----------------------------------------
        //NSS	SCK	MISO	MOSI	NRST	TXEN	RXEN	BUSY	DIO1	DIO2
        //PA4	PA5	PA6	PA7	PB5	PB4	PB3	PA15	PB0	PB1 
        CONFIG_SX126X_CS_CLK_FUN(CONFIG_SX126X_CS_CLK, ENABLE);
        CONFIG_SX126X_SCK_CLK_FUN(CONFIG_SX126X_SCK_CLK, ENABLE);
        CONFIG_SX126X_MISO_CLK_FUN(CONFIG_SX126X_MISO_CLK, ENABLE);
        CONFIG_SX126X_MOSI_CLK_FUN(CONFIG_SX126X_MOSI_CLK, ENABLE);
        CONFIG_SX126X_NRST_CLK_FUN(CONFIG_SX126X_NRST_CLK, ENABLE);
        CONFIG_SX126X_TXEN_CLK_FUN(CONFIG_SX126X_TXEN_CLK, ENABLE);
        CONFIG_SX126X_RXEN_CLK_FUN(CONFIG_SX126X_RXEN_CLK, ENABLE);
        CONFIG_SX126X_BUSY_CLK_FUN(CONFIG_SX126X_BUSY_CLK, ENABLE);
        CONFIG_SX126X_DIO1_CLK_FUN(CONFIG_SX126X_DIO1_CLK, ENABLE);
        CONFIG_SX126X_DIO2_CLK_FUN(CONFIG_SX126X_DIO2_CLK, ENABLE);

        // 3. Disable JTAG feature to release PB3, PB4, PA15 -------------------
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
        GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

        // 4. Init GPIO --------------------------------------------------
        // CS
        gpio_init.GPIO_Pin = CONFIG_SX126X_CS_PIN;
        gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
        gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(CONFIG_SX126X_CS_PORT, &gpio_init);

        // SCK
        gpio_init.GPIO_Pin = CONFIG_SX126X_SCK_PIN;
        gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
        gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(CONFIG_SX126X_SCK_PORT, &gpio_init);

        // MISO
        gpio_init.GPIO_Pin = CONFIG_SX126X_MISO_PIN;
        GPIO_Init(CONFIG_SX126X_MISO_PORT, &gpio_init);

        // MOSI
        gpio_init.GPIO_Pin = CONFIG_SX126X_MOSI_PIN;
        GPIO_Init(CONFIG_SX126X_MOSI_PORT, &gpio_init);

        // NRST
        gpio_init.GPIO_Pin = CONFIG_SX126X_NRST_PIN;
        gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
        gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(CONFIG_SX126X_NRST_PORT, &gpio_init);

        // TXEN
        gpio_init.GPIO_Pin = CONFIG_SX126X_TXEN_PIN;
        GPIO_Init(CONFIG_SX126X_TXEN_PORT, &gpio_init);

        // RXEN
        gpio_init.GPIO_Pin = CONFIG_SX126X_RXEN_PIN;
        GPIO_Init(CONFIG_SX126X_RXEN_PORT, &gpio_init);

        // BUSY
        gpio_init.GPIO_Pin = CONFIG_SX126X_BUSY_PIN;
        gpio_init.GPIO_Mode= GPIO_Mode_IN_FLOATING;
        GPIO_Init(CONFIG_SX126X_BUSY_PORT, &gpio_init);

        // DIO1
        gpio_init.GPIO_Pin = CONFIG_SX126X_DIO1_PIN;
        gpio_init.GPIO_Mode= GPIO_Mode_IPD;
        GPIO_Init(CONFIG_SX126X_DIO1_PORT, &gpio_init);

        // DIO2
        gpio_init.GPIO_Pin = CONFIG_SX126X_DIO2_PIN;
        gpio_init.GPIO_Mode= GPIO_Mode_IPD;
        GPIO_Init(CONFIG_SX126X_DIO2_PORT, &gpio_init);

        // 5. Config interrupt ----------------------------------------
        GPIO_EXTILineConfig(CONFIG_SX126X_DIO1_EXTI_PORTSRC, CONFIG_SX126X_DIO1_EXTI_PINSRC);
        GPIO_EXTILineConfig(CONFIG_SX126X_DIO2_EXTI_PORTSRC, CONFIG_SX126X_DIO2_EXTI_PINSRC);

        exti_init.EXTI_Line = CONFIG_SX126X_DIO1_EXTI_LINE;
        exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
        exti_init.EXTI_Trigger = EXTI_Trigger_Rising;
        exti_init.EXTI_LineCmd =ENABLE;
        EXTI_Init(&exti_init);

        exti_init.EXTI_Line = CONFIG_SX126X_DIO2_EXTI_LINE;
        EXTI_Init(&exti_init);

        //_nvic_config();
        // DIO1
        NVIC_SetPriority(CONFIG_SX126X_DIO1_EXTI_IRQ, 3);
        NVIC_EnableIRQ(CONFIG_SX126X_DIO1_EXTI_IRQ);
        // DIO2
        NVIC_SetPriority(CONFIG_SX126X_DIO2_EXTI_IRQ, 3);
        NVIC_EnableIRQ(CONFIG_SX126X_DIO2_EXTI_IRQ);

        // 6. Configure SPI mode -------------------------------------
        spi_init.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
        spi_init.SPI_Mode = SPI_Mode_Master;
        spi_init.SPI_DataSize = SPI_DataSize_8b;
        spi_init.SPI_CPOL = SPI_CPOL_Low;
        spi_init.SPI_CPHA = SPI_CPHA_1Edge;
        spi_init.SPI_NSS = SPI_NSS_Soft;
        spi_init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
        spi_init.SPI_FirstBit = SPI_FirstBit_MSB;
        spi_init.SPI_CRCPolynomial = 7;
        SPI_Init(CONFIG_SX126X_SPI, &spi_init);

        // 7. Enable SPI  ----------------------------------------
        SPI_Cmd(CONFIG_SX126X_SPI, ENABLE);
}

