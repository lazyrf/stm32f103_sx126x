#include "lora_mac.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "bsp_led.h"

#include <sys/process.h>
#include <sys/procinit.h>
#include <etimer.h>
#include <sys/autostart.h>
#include <clock.h>

lora_t *lora_ctx;

extern void RadioOnTxTimeoutIrq( void* context);
extern void RadioOnRxTimeoutIrq( void* context);

PROCESS(lora_timer_process, "LoRa_Timer");
PROCESS(lora_irq_process, "LoRa_IRQ");

extern struct timer tx_timeout_timer;
extern struct timer rx_timeout_timer;

#define RX_TIMEOUT_VALUE			0

void lora_mac_send(uint8_t *data, uint8_t len)
{
        Radio.send(data, len);
}

static void dump_raw_packet(uint8_t is_tx, uint8_t *packet, int length, int16_t rssi, int8_t snr)
{
        if (is_tx) {
                printf("================= Send packet ===================\r\n");
        } else {
                printf("======= Receive packet (rssi =%d, snr = %d) =======\r\n", rssi, snr);
        }
        for (int cnt = 0; cnt < length; cnt++) {
                printf("%02x%s", packet[cnt], (cnt % 16 == 15) ? "\r\n" : " ");
        }
        if (length % 16 == 0) {
                printf("==================================================\r\n");
        } else {
                printf("\r\n==================================================\r\n");
        }
}

void radio_irq_cb(void)
{
        process_poll(&lora_irq_process);
}

static void on_radio_tx_done( void )
{
        printf("[LoRa_MAC][INFO] Radio tx done.\r\n");

        lora_ctx->stats.tx_cnt++;
        lora_ctx->last_tx_time = clock_time();

        Radio.start_rx(RX_TIMEOUT_VALUE);
	
	if ((lora_ctx->callbacks != NULL) && (lora_ctx->callbacks->tx_done_cb != NULL)) {
                lora_ctx->callbacks->tx_done_cb();
        }
}

static void on_radio_rx_done( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
        printf("[LoRa_MAC][INFO] Radio rx done.\r\n");

        lora_ctx->stats.rx_cnt++;
        lora_ctx->last_rx_time = clock_time();

	if ((lora_ctx->callbacks != NULL) && (lora_ctx->callbacks->rx_done_cb != NULL)) {
                lora_ctx->callbacks->rx_done_cb(payload, size, rssi, snr);
        }

}

static void on_radio_tx_timeout( void )
{
        printf("[LoRa_MAC][INFO] Radio tx timeout.\r\n");

        Radio.set_channel(lora_ctx->config->freq);
        Radio.start_rx(RX_TIMEOUT_VALUE);

        if ((lora_ctx->callbacks != NULL) && (lora_ctx->callbacks->tx_timeout_cb!= NULL)) {
                lora_ctx->callbacks->tx_timeout_cb();
        }
}

static void on_radio_rx_timeout( void )
{
        printf("[LoRa_MAC][INFO] Radio rx timeout.\r\n");

        Radio.start_rx(RX_TIMEOUT_VALUE);

        if ((lora_ctx->callbacks != NULL) && (lora_ctx->callbacks->rx_timeout_cb!= NULL)) {
                lora_ctx->callbacks->rx_timeout_cb();
        }
}

static void on_radio_rx_error( void )
{
        printf("[LoRa_MAC][INFO] Radio rx error.\r\n");

        Radio.start_rx(RX_TIMEOUT_VALUE);

        if ((lora_ctx->callbacks != NULL) && (lora_ctx->callbacks->rx_error_cb!= NULL)) {
                lora_ctx->callbacks->rx_error_cb();
        }
}

PROCESS_THREAD(lora_timer_process, ev, data)
{
        PROCESS_BEGIN();
        static struct etimer lora_timer;
        etimer_set(&lora_timer, CLOCK_CONF_SECOND / 100);
        while(1) {
                PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);

                if (timer_expired(&tx_timeout_timer)) {
                        RadioOnTxTimeoutIrq(NULL);
                        timer_stop(&tx_timeout_timer);
                }

                if (timer_expired(&rx_timeout_timer)) {
                        RadioOnRxTimeoutIrq(NULL);
                        timer_stop(&rx_timeout_timer);
                }
                etimer_reset(&lora_timer);
        }
        PROCESS_END();
}

PROCESS_THREAD(lora_irq_process, ev, data)
{
        PROCESS_BEGIN();
        while (1) {
                PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
                Radio.irq_process();
        }
        PROCESS_END();
}

lora_stats_t lora_mac_stats_get(void)
{
        return lora_ctx->stats;
}

void lora_mac_init(int is_master, radio_config_t *config, lora_callback_t *callbacks)
{
	printf("Init LoRa %s...\r\n", (is_master) ? "Master" : "Slave");
        lora_ctx = (lora_t *) calloc(1, sizeof(lora_t));
	lora_ctx->is_master = is_master;
        lora_ctx->callbacks = callbacks;
	lora_ctx->config = config;

        //Radio initialization
        lora_ctx->radio_events.tx_done = on_radio_tx_done;
        lora_ctx->radio_events.rx_done = on_radio_rx_done;
        lora_ctx->radio_events.tx_timeout = on_radio_tx_timeout;
        lora_ctx->radio_events.rx_timeout = on_radio_rx_timeout;
        lora_ctx->radio_events.rx_error = on_radio_rx_error;
        Radio.init( &lora_ctx->radio_events, lora_ctx->config);

        // ------------------ Frequncy -----------------------
        Radio.set_channel(lora_ctx->config->freq);

        // --------- TX config (power, bandwidth, SF, coding rate, etc.) ----------
        Radio.set_tx_config(MODEM_LORA, lora_ctx->config->power_config, 0,
                lora_ctx->config->bandwidth, lora_ctx->config->spreading_factor,
                lora_ctx->config->coding_rate, lora_ctx->config->preamble_len,
                lora_ctx->config->fix_len, lora_ctx->config->crc_on, 0,
                0, lora_ctx->config->invert_iq, 3000 );

        // --------- RX config (bandwidth, SF, coding rate, etc.) -------------
        Radio.set_rx_config(MODEM_LORA, lora_ctx->config->bandwidth,
                lora_ctx->config->spreading_factor, lora_ctx->config->coding_rate,
                0, lora_ctx->config->preamble_len,
                lora_ctx->config->symbol_timeout, lora_ctx->config->fix_len,
                0,
                lora_ctx->config->crc_on, 0, 0,
                lora_ctx->config->invert_iq, true );

        Radio.start_rx( RX_TIMEOUT_VALUE );

        process_start(&lora_timer_process, NULL);
        process_start(&lora_irq_process, NULL);
}

