#ifndef __LORA_MAC_H
#define __LORA_MAC_H

#include "config.h"
#include "radio.h"

#define LORA_CHANNLE_NUM                12
#define LORA_WAIT_REQ_TIMEOUT		60000

typedef struct lora_callback_s {
        void (*tx_done_cb) (void);
        void (*rx_done_cb)(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
        void (*tx_timeout_cb)(void);
        void (*rx_timeout_cb)(void);
        void (*rx_error_cb)(void);
} lora_callback_t;

typedef struct {
        uint32_t tx_cnt;
        uint32_t rx_cnt;
        uint32_t tx_timeout_cnt;
        uint32_t rx_err_cnt;
} lora_stats_t;

typedef struct lora_s   lora_t;
struct lora_s{
	int is_master;
	radio_config_t *config;	/* LoRa config (Freq, Bandwidth, SF, CR..) */
        lora_callback_t *callbacks;     /* LoRa mac upper layer event functions */
        radio_events_t radio_events;    /* Radio evnets function pointer */
        lora_stats_t stats;     /* Statistics */
        uint32_t last_tx_time;  /* Last packet transmit time */
        uint32_t last_rx_time;  /* Last packet receive time */
} ;

void lora_mac_set_channel(uint8_t ch);
void lora_mac_init(int is_master, radio_config_t *config, lora_callback_t *callbacks);
void lora_mac_test_send(void);

#endif /* __LORA_MAC_H */

