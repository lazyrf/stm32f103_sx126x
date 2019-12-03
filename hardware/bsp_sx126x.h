#ifndef __BSP_SX126X_H
#define __BSP_SX126X_H

#include "config.h"

/*!
 * Hardware IO IRQ callback function definition
 */
typedef void (*dio_irq_handler_fn )( void* context );

#define DUMMY_BYTE		0x00

#define SX126X_SPI_CS_LOW()	GPIO_ResetBits(CONFIG_SX126X_CS_PORT, CONFIG_SX126X_CS_PIN)
#define SX126X_SPI_CS_HIGH()	GPIO_SetBits(CONFIG_SX126X_CS_PORT, CONFIG_SX126X_CS_PIN)

#define SX126X_SPI_SHORT_TIMEOUT	((uint32_t) 0x1000)
#define SX126X_SPI_LONG_TIMEOUT	((uint32_t) (10 * SX126X_SPI_SHORT_TIMEOUT))

#define SX126X_RESET_HIGH()	GPIO_SetBits(CONFIG_SX126X_NRST_PORT, CONFIG_SX126X_NRST_PIN)
#define SX126X_RESET_LOW() 	GPIO_ResetBits(CONFIG_SX126X_NRST_PORT, CONFIG_SX126X_NRST_PIN)

#define SX126X_TXEN_HIGH() 	GPIO_SetBits(CONFIG_SX126X_TXEN_PORT, CONFIG_SX126X_TXEN_PIN)
#define SX126X_TXEN_LOW()		GPIO_ResetBits(CONFIG_SX126X_TXEN_PORT, CONFIG_SX126X_TXEN_PIN)

#define SX126X_RXEN_HIGH()	GPIO_SetBits(CONFIG_SX126X_RXEN_PORT, CONFIG_SX126X_RXEN_PIN)
#define SX126X_RXEN_LOW() 	GPIO_ResetBits(CONFIG_SX126X_RXEN_PORT, CONFIG_SX126X_RXEN_PIN)

#define SX126X_DIO1_STATE()     GPIO_ReadInputDataBit(CONFIG_SX126X_DIO1_PORT, CONFIG_SX126X_DIO1_PIN)
#define SX126X_DIO2_STATE()     GPIO_ReadInputDataBit(CONFIG_SX126X_DIO2_PORT, CONFIG_SX126X_DIO2_PIN)

#define SX126X_BUSY()		GPIO_ReadInputDataBit(CONFIG_SX126X_BUSY_PORT, CONFIG_SX126X_BUSY_PIN)

#define BOARD_TCXO_WAKEUP_TIME			    5


/* BACKUP_PRIMASK MUST be implemented at the begining of the funtion
   that implement a critical section
   PRIMASK is saved on STACK and recovered at the end of the funtion
   That way RESTORE_PRIMASK ensures critical sections are maintained even in nested calls...*/
#define BACKUP_PRIMASK()  uint32_t primask_bit= __get_PRIMASK()
#define DISABLE_IRQ() __disable_irq()
#define ENABLE_IRQ() __enable_irq()
#define RESTORE_PRIMASK() __set_PRIMASK(primask_bit)


#define CRITICAL_SECTION_BEGIN( )     uint32_t primask_bit= __get_PRIMASK();\
                                    __disable_irq()
#define CRITICAL_SECTION_END( )   __set_PRIMASK(primask_bit)


void bsp_sx126x_init(dio_irq_handler_fn cb);
uint8_t bsp_sx126x_send_byte(uint8_t byte);
uint8_t bsp_sx126x_read_byte(void);
void bsp_sx126x_dio1_interrupt_enable(void);
void bsp_sx126x_dio1_interrupt_disable(void);
void bsp_sx126x_dio2_interrupt_enable(void);
void bsp_sx126x_dio2_interrupt_disable(void);

#endif /* __BSP_SX126X_H */

