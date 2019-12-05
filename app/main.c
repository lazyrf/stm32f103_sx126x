#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

#include "config.h"
#include "bsp_rcc.h"
#include "bsp_led.h"
#include "bsp_dbg_uart.h"
#include "delay.h"
#include "bsp_oled.h"

#include <sys/process.h>
#include <sys/procinit.h>
#include <etimer.h>
#include <sys/autostart.h>
#include <clock.h>

#include "lora_mac.h"

#define osDelay(ms) \
	static struct etimer et; \
	etimer_set(&et, ms / (1000 / CLOCK_SECOND)); \
	PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et))

unsigned int idle_count = 0;

PROCESS(led_process, "LED");
PROCESS(oled_display_process, "OLED_Display");
//PROCESS(lora_test_process, "LoRa test");
AUTOSTART_PROCESSES(&led_process);

// Default LoRa config
radio_config_t lora_config = {
        433691000,      // Frequency
        2,    // Bandwidth [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
        LORA_SF11,       // Spreading factor
        LORA_CR_4_5,    // Code Rate
        22,     // Power config
        false,   // Fixed Packet length
        true,    // CRC on
        false, // Invert IQ on
        0,      // Symbol timeout
        8,      // Preamble length
        64,     // Payload length
};

int freq_sel = 0;
int bw_sel = 0;
int sf_sel = 0;
int cr_sel = 0;

uint32_t tx_cnt = 0;
uint32_t rx_cnt = 0;

static void lora_app_tx_done_cb(void)
{
        printf("[LoRa_APP][INFO] Packet tx success.\r\n");
	tx_cnt++;
}

static void lora_app_rx_done_cb(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
        printf("[LoRa_APP][INFO] Packet rx success.\r\n");
	rx_cnt++;
}

void lora_app_tx_timeout_cb(void)
{
        printf("[LoRa_APP][INFO] Packet tx timeout.\r\n");
}

void lora_app_rx_timeout_cb(void)
{
        printf("[LoRa_APP][INFO] Packet rx timeout.\r\n");
}

void lora_app_rx_error_cb(void)
{
        printf("[LoRa_APP][INFO] Packet rx error.\r\n");
}

void show_fw_info(void)
{
	printf("\r\n=========================================\r\n");
	printf("Project name: sx126x demo kit\r\n");
	printf("Author: Aron Li\r\n");
	printf("Version: 0.1\r\n");
	printf("SystemCoreClock: %lu\r\n", SystemCoreClock);
	printf("=========================================\r\n");
}

void oled_show_blank(int x, int y, int len)
{
	char *str = (char *) malloc(len + 1);
	memset(str, 0x0, len + 1);
	memset(str, ' ', len);
	bsp_oled_show_string(x, y, str);
}

void oled_show_string(int x, int y, int len, const char *fmt, ...)
{
	va_list ap;
	char *str = (char *) malloc(len + 1);

	memset(str, 0x0, len + 1);

	va_start(ap, fmt);
	vsprintf(str, fmt, ap);
	va_end(ap);

	str[len] = '\0';

	printf("### str = %s (origin: %s, len = %d) ###\r\n", str, fmt, len);
	bsp_oled_show_string(x, y, str);

	free(str);
}

PROCESS_THREAD(led_process, ev, data)
{
	PROCESS_BEGIN();
	while (1) {
		LED_TOGGLE(CONFIG_LED_1_GPIO_PORT, CONFIG_LED_1_GPIO_PIN);
		osDelay(1000);
	}
	PROCESS_END();
}

PROCESS_THREAD(oled_display_process, ev, data)
{
	PROCESS_BEGIN();
	static int n = 0;
	static int blink = 0;
	while (1) {
		int bw;
		char cr_str[5];

		switch (lora_config.bandwidth) {
			case 0:
				bw = 125;
				break;
			case 1:
				bw = 250;
				break;
			case 2:
				bw = 500;
				break;
			default:
				bw = 0;
				break;
		}

		switch (lora_config.coding_rate) {
			case LORA_CR_4_5:
				memcpy(cr_str, "C:4/5", 5);
				break;
			case LORA_CR_4_6:
				memcpy(cr_str, "C:4/6", 5);
				break;
			case LORA_CR_4_7:
				memcpy(cr_str, "C:4/7", 5);
				break;
			case LORA_CR_4_8:
				memcpy(cr_str, "C:4/8", 5);
				break;
		}

		if (freq_sel || bw_sel || sf_sel || cr_sel) {
			if (++n == 5) {
				if (blink) {
					oled_show_string(0, 0, 9, "F:%.2fM", lora_config.freq / 1000000.0f);
					oled_show_string(10 * 8, 0, 6, "B:%03dM", bw);
					oled_show_string(0, 2, 5, "SF:%02d", lora_config.spreading_factor);
					oled_show_string(6 * 8, 2, 5, cr_str);
				} else {
					if (freq_sel) {
						oled_show_blank(0, 0, 9);
					}
					if (bw_sel) {
						oled_show_blank(10 * 8, 0, 6);
					}
					if (sf_sel) {
						oled_show_blank(0, 2, 5);
					}
					if (cr_sel) {
						oled_show_blank(6 * 8, 2, 5);
					}
				}
				n = 0;
				blink = !blink;
			}
		} else {
			oled_show_string(0, 0, 9, "F:%.2fM", lora_config.freq / 1000000.0f);
			oled_show_string(10 * 8, 0, 6, "B:%03dM", bw);
			oled_show_string(0, 2, 5, "SF:%02d", lora_config.spreading_factor);
			oled_show_string(6 * 8, 2, 5, cr_str);
			oled_show_string(12 * 8, 2, 4, "P:22");
			oled_show_string(0, 4, 7, "TX:%4d", tx_cnt);
			oled_show_string(9 * 8, 4, 7, "RX:%4d", rx_cnt);
		}

		osDelay(100);
	}
	PROCESS_END();
}

#if 0

PROCESS_THREAD(lora_test_process, ev, data)
{
        PROCESS_BEGIN();
        while(1) {
                lora_mac_test_send();
                osDelay(2000);
        }
        PROCESS_END();
}
#endif

int main(void)
{
	lora_callback_t *lora_cbs;

	bsp_rcc_init();

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	// Eanble PWR clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	clock_init();
	bsp_led_init();
	bsp_oled_init();
	bsp_dbg_uart_init();

	show_fw_info();

	process_init();
	process_start(&etimer_process, NULL);
	autostart_start(autostart_processes);

	lora_cbs = (lora_callback_t *) malloc(sizeof(lora_callback_t));
	memset(lora_cbs, 0, sizeof(lora_callback_t));
	lora_cbs->tx_done_cb = lora_app_tx_done_cb;
	lora_cbs->rx_done_cb = lora_app_rx_done_cb;
	lora_cbs->tx_timeout_cb = lora_app_tx_timeout_cb;
	lora_cbs->rx_timeout_cb = lora_app_rx_timeout_cb;
	lora_cbs->rx_error_cb = lora_app_rx_error_cb;

	//lora_mac_init(1, &lora_config, lora_cbs);
	//process_start(&lora_test_process, NULL);

	process_start(&oled_display_process, NULL);

	printf("Porcesses running\r\n");

	while (1) {
		do {
		} while (process_run() > 0);
		idle_count++;
	}

	return 0;
}
