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
#include "bsp_di.h"
#include "bsp_key.h"
#include "bsp_crc.h"
#include "fw_env.h"

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
PROCESS(lora_test_process, "LoRa test");
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

uint32_t ch_list[] = {
        440816000,      // 0
        438441000,      // 1
        436066000,      // 2
        433691000,      // 3
        428941000,      // 4
        426566000      // 5
};
uint32_t bw_list[] = {125, 250, 500};  // 3
radio_lora_spreading_factor_t sf_list[] = {LORA_SF5, LORA_SF6, LORA_SF7, LORA_SF8, LORA_SF9, LORA_SF10, LORA_SF11, LORA_SF12}; // 8
radio_lora_coding_rate_t cr_list[] = {LORA_CR_4_5, LORA_CR_4_6, LORA_CR_4_7, LORA_CR_4_8}; // 4

int sel_item = 0;

uint32_t tx_cnt = 0;
uint32_t rx_cnt = 0;

int lora_ch_index = 3;
int lora_bw_index = 0;
int lora_sf_index = 6;
int lora_cr_index = 0;

uint8_t tx_buffer[32] = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF,
        0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF};
uint8_t rx_buffer[32] = {0};

void key1_handler(void)
{
	printf("Key1 handler\r\n");

	sel_item++;
	sel_item %= 4;
}

void key2_handler(void)
{
	printf("Key2 handler\r\n");
	switch (sel_item) {
                case 0:
                        lora_ch_index++;
                        lora_ch_index %= 6;
                        lora_config.freq = ch_list[lora_ch_index];
                        break;
                case 1:
                        lora_bw_index++;
                        lora_bw_index %= 3;
                        lora_config.bandwidth = lora_bw_index;
                        break;
                case 2:
                        lora_sf_index++;
                        lora_sf_index %= 8;
                        lora_config.spreading_factor = sf_list[lora_sf_index];
                        break;
                case 3:
                        lora_cr_index++;
                        lora_cr_index %= 4;
                        lora_config.coding_rate = cr_list[lora_cr_index];
                        break;
	}
}

void dump_packet(uint8_t *packet, int length, int16_t rssi, int8_t snr)
{
	printf("======= Receive packet (rssi =%d, snr = %d) =======\r\n", rssi, snr);

	for (int cnt = 0; cnt < length; cnt++) {
		printf("%02x%s", packet[cnt], (cnt % 16 == 15) ? "\r\n" : " ");
	}
	if (length % 16 == 0) {
		printf("==================================================\r\n");
	} else {
		printf("\r\n==================================================\r\n");
	}
}

static void lora_app_tx_done_cb(void)
{
	printf("[LoRa_APP][INFO] Packet tx success.\r\n");
	tx_cnt++;
}

static void lora_app_rx_done_cb(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
	printf("[LoRa_APP][INFO] Packet rx success.\r\n");

	// dump_packet(payload, size, rssi, snr);

	if (memcmp(payload, tx_buffer, 32) == 0) {
		if (!DI2_DATA()) {
			mdelay(200);
			memcpy(rx_buffer, tx_buffer, 32);
			rx_buffer[0] = 0x88;
			lora_mac_send(tx_buffer, 32);
		}
		rx_cnt++;
	}
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
	static int old_config_mode = 0;
	while (1) {
		int bw;
		char cr_str[5];

		int config_mode = DI1_DATA();
		if (old_config_mode == 1 && config_mode == 0) {
			char buf[5];
			// Save config to flash
			// Channel
			sprintf(buf, "%d", lora_ch_index);
			fw_env_write("ch", buf);

			// Bandwidth
			sprintf(buf, "%d", lora_bw_index);
			fw_env_write("bw", buf);

			// Spreading factor
			sprintf(buf, "%d", lora_sf_index);
			fw_env_write("sf", buf);

			// Coding rate
			sprintf(buf, "%d", lora_cr_index);
			fw_env_write("cr", buf);

			fw_env_save();

			NVIC_SystemReset();
		}

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

		if (config_mode) {
			if (++n == 5) {
				if (blink) {
					oled_show_string(0, 0, 9, "F:%.2fM", lora_config.freq / 1000000.0f);
					oled_show_string(10 * 8, 0, 6, "B:%03dM", bw);
					oled_show_string(0, 2, 5, "SF:%02d", lora_config.spreading_factor);
					oled_show_string(6 * 8, 2, 5, cr_str);
				} else {
					switch (sel_item) {
					case 0:
						oled_show_blank(0, 0, 9);
						break;
					case 1:
						oled_show_blank(10 * 8, 0, 6);
						break;
					case 2:
						oled_show_blank(0, 2, 5);
						break;
					case 3:
						oled_show_blank(6 * 8, 2, 5);
						break;
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

		old_config_mode = config_mode;
		osDelay(100);
	}
	PROCESS_END();
}


PROCESS_THREAD(lora_test_process, ev, data)
{
	PROCESS_BEGIN();
	while(1) {
		if (DI2_DATA()) {
			lora_mac_send(tx_buffer, 32);
		}
		osDelay(5000);
	}
	PROCESS_END();
}

int main(void)
{
	lora_callback_t *lora_cbs;
	char *ch_str, *bw_str, *sf_str, *cr_str;

	bsp_rcc_init();

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	// Eanble PWR clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	clock_init();
	bsp_led_init();
	bsp_oled_init();
	bsp_dbg_uart_init();
	show_fw_info();
	bsp_di_init();
	bsp_key_init();
	bsp_crc_init();
	fw_env_open();
	fw_printenv();

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

	ch_str = fw_getenv("ch");
	if (ch_str != NULL) {
		sscanf(ch_str, "%d", &lora_ch_index);
	}

	bw_str = fw_getenv("bw");
	if (bw_str != NULL) {
		sscanf(bw_str, "%d", &lora_bw_index);
	}

	sf_str = fw_getenv("sf");
	if (sf_str != NULL) {
		sscanf(sf_str, "%d", &lora_sf_index);
	}

	cr_str = fw_getenv("cr");
	if (cr_str != NULL) {
		sscanf(cr_str, "%d", &lora_cr_index);
	}

	lora_config.freq = ch_list[lora_ch_index];
	lora_config.bandwidth = lora_bw_index;
	lora_config.spreading_factor = sf_list[lora_sf_index];
	lora_config.coding_rate = cr_list[lora_cr_index];

	lora_mac_init(1, &lora_config, lora_cbs);
	process_start(&lora_test_process, NULL);

	process_start(&oled_display_process, NULL);

	printf("Porcesses running\r\n");

	while (1) {
		do {
		} while (process_run() > 0);
		idle_count++;
	}

	return 0;
}
