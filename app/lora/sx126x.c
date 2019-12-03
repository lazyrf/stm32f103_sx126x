#include <stdio.h>
#include  <math.h>
#include <string.h>
#include "bsp_rcc.h"

#include "sx126x.h"


/**
 * brief	Holds the internal opearting mode of the radio
 */
static radio_operating_mode_t operating_mode;

/**
 * brief Stores the current packet tyep set in the radio
 */
static radio_packet_type_t packet_type;

#if 0
/*!
 * \brief Hold the status of the Image calibration
 */
static bool image_calibrated = false;
#endif

/**
 * brief Stores the last frequency error measured on LoRa received packet
 */
volatile uint32_t freq_err = 0;

void sx126x_wait_on_busy(void)
{
        while (SX126X_BUSY() == 1);
}

void sx126x_wakeup(void)
{
        SX126X_SPI_CS_LOW();
        bsp_sx126x_send_byte(RADIO_GET_STATUS);
        bsp_sx126x_send_byte(DUMMY_BYTE);
        SX126X_SPI_CS_HIGH();
        // Wait for chip to be ready
        sx126x_wait_on_busy();
}

radio_operating_mode_t sx126x_get_operating_mode(void)
{
        return operating_mode;
}

void sx126x_set_operating_mode(radio_operating_mode_t mode)
{
        operating_mode = mode;
}


void sx126x_check_device_ready(void)
{
        if ((sx126x_get_operating_mode() == MODE_SLEEP) || (sx126x_get_operating_mode() == MODE_RX_DC)) {
                sx126x_wakeup();
                // Aron, Switch is tuned off when device is in sleep mode and tuned on is all other modes
                //sx126x_ant_sw_on();
        }
        sx126x_wait_on_busy();
}


void sx126x_write_registers(uint16_t addr, uint8_t *buffer, uint16_t size)
{
        sx126x_check_device_ready();

        SX126X_SPI_CS_LOW();
        bsp_sx126x_send_byte(RADIO_WRITE_REGISTER);
        bsp_sx126x_send_byte((addr & 0xFF00) >> 8);
        bsp_sx126x_send_byte(addr & 0x00FF);
        for (uint16_t i = 0; i < size; i++) {
                bsp_sx126x_send_byte(buffer[i]);
        }
        SX126X_SPI_CS_HIGH();

        sx126x_wait_on_busy();
}

void sx126x_write_register(uint16_t addr, uint8_t value)
{
        sx126x_write_registers(addr, &value, 1);
}

void sx126x_read_registers(uint16_t addr, uint8_t *buffer, uint16_t size)
{
        sx126x_check_device_ready();

        SX126X_SPI_CS_LOW();
        bsp_sx126x_send_byte(RADIO_READ_REGISTER);
        bsp_sx126x_send_byte((addr & 0xFF00) >> 8);
        bsp_sx126x_send_byte(addr & 0x00FF);
        bsp_sx126x_send_byte(DUMMY_BYTE);
        for (uint16_t i = 0; i < size; i++) {
                buffer[i] = bsp_sx126x_send_byte(DUMMY_BYTE);
        }
        SX126X_SPI_CS_HIGH();

        sx126x_wait_on_busy();
}

uint8_t sx126x_read_register(uint16_t addr)
{
        uint8_t data;
        sx126x_read_registers(addr, &data, 1);
        return data;
}

void sx126x_write_cmd(radio_cmd_t cmd, uint8_t *buffer, uint16_t size)
{
        sx126x_check_device_ready();

        SX126X_SPI_CS_LOW();
        bsp_sx126x_send_byte((uint8_t) cmd);
        for (uint16_t i = 0; i < size; i++) {
                bsp_sx126x_send_byte(buffer[i]);
        }
        SX126X_SPI_CS_HIGH();

        if (cmd != RADIO_SET_SLEEP) {
                sx126x_wait_on_busy();
        }
}

void sx126x_read_cmd(radio_cmd_t cmd, uint8_t *buffer, uint16_t size)
{
        sx126x_check_device_ready();

        SX126X_SPI_CS_LOW();
        bsp_sx126x_send_byte((uint8_t) cmd);
        bsp_sx126x_send_byte(DUMMY_BYTE);
        for (uint16_t i = 0; i < size; i++) {
                buffer[i] = bsp_sx126x_send_byte(DUMMY_BYTE);
        }
        SX126X_SPI_CS_HIGH();

        sx126x_wait_on_busy();
}

void sx126x_write_buffer(uint8_t offset, uint8_t *buffer, uint16_t size)
{
        sx126x_check_device_ready();

        SX126X_SPI_CS_LOW();
        bsp_sx126x_send_byte(RADIO_WRITE_BUFFER);
        bsp_sx126x_send_byte(offset);
        for (uint16_t i = 0; i < size; i++) {
                bsp_sx126x_send_byte(buffer[i]);
        }
        SX126X_SPI_CS_HIGH();

        sx126x_wait_on_busy();
}

void sx126x_read_buffer(uint8_t offset, uint8_t *buffer, uint16_t size)
{
        sx126x_check_device_ready();

        SX126X_SPI_CS_LOW();
        bsp_sx126x_send_byte(RADIO_READ_BUFFER);
        bsp_sx126x_send_byte(offset);
        bsp_sx126x_send_byte(DUMMY_BYTE);
        for (uint16_t i = 0; i < size; i++) {
                buffer[i] = bsp_sx126x_send_byte(DUMMY_BYTE);
        }
        SX126X_SPI_CS_HIGH();

        sx126x_wait_on_busy();
}


/*******************************************************************
 *
 * sx126x module reset, tx power, tx frequency configuration
 * Operating mode, packet read/write
 *
 *******************************************************************/

void sx126x_reset(void)
{
        mdelay(10);
        SX126X_RESET_LOW();
        mdelay(20);
        SX126X_RESET_HIGH();
        mdelay(10);
}

void sx126x_tx_en(void)
{
        SX126X_TXEN_HIGH();
        SX126X_RXEN_LOW();
}

void sx126x_rx_en(void)
{
        SX126X_TXEN_LOW();
        SX126X_RXEN_HIGH();
}

void sx126x_cad_en(void)
{
        SX126X_TXEN_HIGH();
        SX126X_RXEN_HIGH();
}

void sx126x_switch_off(void)
{
        SX126X_TXEN_LOW();
        SX126X_RXEN_LOW();
}

void sx126x_dio1_interrupt_enable(void)
{
        bsp_sx126x_dio1_interrupt_enable();
}

void sx126x_dio1_interrupt_disable(void)
{
        bsp_sx126x_dio1_interrupt_disable();
}

uint8_t sx126x_dio1_state_get(void)
{
        return SX126X_DIO1_STATE();
}

void sx126x_dio2_interrupt_enable(void)
{
        bsp_sx126x_dio2_interrupt_enable();
}

void sx126x_dio2_interrupt_disable(void)
{
        bsp_sx126x_dio2_interrupt_disable();
}

uint8_t sx126x_dio2_state_get(void)
{
        return SX126X_DIO2_STATE();
}

bool sx126x_check_rf_freq(uint32_t freq)
{
        // Implement check. Currently all frequencies are support
        return true;
}

radio_packet_type_t sx126x_get_packet_type(void)
{
        return packet_type;
}

void sx126x_set_packet_type(radio_packet_type_t new_packet_type)
{
        if (new_packet_type == PACKET_TYPE_GFSK) {
                sx126x_write_register(REG_BIT_SYNC, 0x00);
        }

        packet_type = new_packet_type;
        sx126x_write_cmd(RADIO_SET_PACKET_TYPE, (uint8_t *) &new_packet_type, 1);
}

void sx126x_set_crc_seed(uint16_t seed)
{
        uint8_t buf[2];

        buf[0] = (uint8_t) ((seed >> 8) & 0xFF);
        buf[1] = (uint8_t) (seed & 0xFF);

        switch (sx126x_get_packet_type()) {
        case PACKET_TYPE_GFSK:
                sx126x_write_registers(REG_LR_CRC_SEED_BASE_ADDR, buf, 2);
                break;
        default:
                break;
        }
}

void sx126x_set_crc_polynomial(uint16_t polynomial)
{
        uint8_t buf[2];

        buf[0] = (uint8_t) ((polynomial >> 8) & 0xFF);
        buf[1] = (uint8_t) (polynomial & 0xFF);

        switch (sx126x_get_packet_type()) {
        case PACKET_TYPE_GFSK:
                sx126x_write_registers(REG_LR_CRC_POLY_BASE_ADDR, buf, 2);
                break;
        default:
                break;
        }
}

void sx126x_set_whitening_seed(uint16_t seed)
{
        uint8_t reg_val = 0;

        switch (sx126x_get_packet_type()) {
        case PACKET_TYPE_GFSK:
                reg_val = sx126x_read_register(REG_LR_WHIT_SEED_BASE_ADDR_MSB) & 0xFE;
                reg_val = ((seed >> 8) & 0x01) | reg_val;
                sx126x_write_register(REG_LR_WHIT_SEED_BASE_ADDR_MSB, reg_val); // Only 1 bit
                sx126x_write_register(REG_LR_WHIT_SEED_BASE_ADDR_LSB, (uint8_t) seed);
                break;
        default:
                break;
        }
}

void sx126x_set_modulation_params(modulation_params_t *mod_params)
{
        uint8_t n;
        uint32_t tmp_val = 0;
        uint8_t buf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

        // Check if required configuration corresponds to the stored packe type
        // If not, silently update radio packe type
        if (packet_type != mod_params->packet_type) {
                sx126x_set_packet_type(mod_params->packet_type);
        }

        switch (mod_params->packet_type) {
        case PACKET_TYPE_GFSK:
                n = 8;
                tmp_val = (uint32_t) (32 *((double) XTAL_FREQ / (double) mod_params->params.gfsk.bit_rate));
                buf[0] = (tmp_val >> 16) & 0xFF;
                buf[1] = (tmp_val >> 8) & 0xFF;
                buf[2] = tmp_val & 0xFF;
                buf[3] = mod_params->params.gfsk.modulation_shaping;
                buf[4] = mod_params->params.gfsk.bandwidth;
                tmp_val = (uint32_t) ((double) mod_params->params.gfsk.fdev / (double) FREQ_STEP);
                buf[5] = (tmp_val >> 16) & 0xFF;
                buf[6] = (tmp_val >> 8) & 0xFF;
                buf[7] = (tmp_val & 0xFF);
                sx126x_write_cmd(RADIO_SET_MODULATION_PARAMS, buf, n);
                break;
        case PACKET_TYPE_LORA:
                n = 4;
                buf[0] = mod_params->params.lora.spreading_factor;
                buf[1] = mod_params->params.lora.bandwith;
                buf[2] = mod_params->params.lora.coding_rate;
                buf[3] = mod_params->params.lora.low_data_rate_optimize;
                sx126x_write_cmd(RADIO_SET_MODULATION_PARAMS, buf, n);
                break;
        default:
        case PACKET_TYPE_NONE:
                return ;
        }
}


void sx126x_set_packet_params(packet_params_t *packet_params)
{
        uint8_t n;
        uint8_t crc_val = 0;
        uint8_t buf[9] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

        // Check if required configuration corresponds to the stored packet type
        // If not, silenty update radio packet type
        if (packet_type != packet_params->packet_type) {
                sx126x_set_packet_type(packet_params->packet_type);
        }

        switch (packet_params->packet_type) {
        case PACKET_TYPE_GFSK:
                if (packet_params->params.gfsk.crc_len == CRC_2_BYTES_IBM) {
                        sx126x_set_crc_seed(CRC_IBM_SEED);
                        sx126x_set_crc_polynomial(CRC_POLYNOMIAL_IBM);
                        crc_val = CRC_2_BYTES;
                } else if (packet_params->params.gfsk.crc_len == CRC_2_BYTES_CCIT) {
                        sx126x_set_crc_seed(CRC_CCITT_SEED);
                        sx126x_set_crc_polynomial(CRC_POLYNOMIAL_CCITT);
                        crc_val = CRC_2_BYTES_INV;
                } else {
                        crc_val = packet_params->params.gfsk.crc_len;
                }
                n = 9;
                buf[0] = (packet_params->params.gfsk.preamble_length >> 8) & 0xFF;
                buf[1] = packet_params->params.gfsk.preamble_length;
                buf[2] = packet_params->params.gfsk.preamble_min_detect;
                buf[3] = (packet_params->params.gfsk.sync_word_length /* << 3 */); // Convert from byte to bit
                buf[4] = packet_params->params.gfsk.addr_comp;
                buf[5] = packet_params->params.gfsk.header_type;
                buf[6] = packet_params->params.gfsk.payload_len;
                buf[7] = crc_val;
                buf[8] = packet_params->params.gfsk.dc_free;
                break;
        case PACKET_TYPE_LORA:
                n = 6;
                buf[0] = (packet_params->params.lora.preamble_length >> 8) & 0xFF;
                buf[1] = packet_params->params.lora.preamble_length;
                buf[2] = packet_params->params.lora.header_type;
                buf[3] = packet_params->params.lora.payload_len;
                buf[4] = packet_params->params.lora.crc_mode;
                buf[5] = packet_params->params.lora.invert_iq;
                break;
        default:
        case PACKET_TYPE_NONE:
                return ;
        }

        sx126x_write_cmd(RADIO_SET_PACKET_PARAMS, buf, n);
}


void sx126x_set_cad_params(radio_lora_cad_symbols_t cad_symbol_num, uint8_t cad_det_peak, uint8_t cad_det_min, radio_cad_exit_modes_t cad_exit_mode, uint32_t cad_timeout)
{
        uint8_t buf[7];

        buf[0] = (uint8_t) cad_symbol_num;
        buf[1] = cad_det_peak;
        buf[2] = cad_det_min;
        buf[3] = (uint8_t) cad_exit_mode;
        buf[4] = (uint8_t) ((cad_timeout >> 16) & 0xFF);
        buf[5] = (uint8_t) ((cad_timeout >> 8) & 0xFF);
        buf[6] = (uint8_t) (cad_timeout & 0xFF);

        sx126x_write_cmd(RADIO_SET_CAD_PARAMS, buf,  5);	// Aron, why not 7?
        sx126x_set_operating_mode(MODE_CAD);
}

void sx126x_set_buffer_base_addr(uint8_t tx_base_addr, uint8_t rx_base_addr)
{
        uint8_t buf[2];

        buf[0] = tx_base_addr;
        buf[1] = rx_base_addr;
        sx126x_write_cmd(RADIO_SET_BUFF_BASE_ADDR, buf, 2);
}

sx126x_status_t sx126x_get_status(void)
{
        uint8_t stat = 0;
        sx126x_status_t status;

        sx126x_read_cmd(RADIO_GET_STATUS, (uint8_t *) &stat, 1);
        status.value = stat;
        return status;
}

int8_t sx126x_get_rssi_inst(void)
{
        uint8_t buf[1];
        int8_t rssi = 0;

        sx126x_read_cmd(RADIO_GET_RSSI_INST, buf, 1);
        rssi = -buf[0] >> 1;
        return rssi;
}

void sx126x_get_rx_buffer_status(uint8_t *payload_len, uint8_t *rx_start_buffer_ptr)
{
        uint8_t status[2];

        sx126x_read_cmd(RADIO_GET_RX_BUFF_STATUS, status, 2);

        // In case of LoRa fixed header, the paylod length is obtained by reading
        // the register REG_LR_PAYLOADLENGTH
        if ((sx126x_get_packet_type() == PACKET_TYPE_LORA) && (sx126x_read_register(REG_LR_PACKET_PARAMS) >> 7 == 1)) {
                *payload_len = sx126x_read_register(REG_LR_PAYLOAD_LEN);
        } else {
                *payload_len = status[0];
        }
        *rx_start_buffer_ptr = status[1];
}


void sx126x_get_packet_status(packet_status_t *pkt_status)
{
        uint8_t status[3];

        sx126x_read_cmd(RADIO_GET_PACKET_STATUS, status, 3);

        pkt_status->packet_type = sx126x_get_packet_type();
        switch (pkt_status->packet_type) {
        case PACKET_TYPE_GFSK:
                pkt_status->params.gfsk.rx_status = status[0];
                pkt_status->params.gfsk.rssi_sync = -status[1] >> 1;
                pkt_status->params.gfsk.rssi_avg = -status[2] >> 1;
                pkt_status->params.gfsk.freq_err = 0;
                break;
        case PACKET_TYPE_LORA:
                pkt_status->params.lora.rssi_pkt = -status[0] >> 1;
                // Return SNR vlaue [dB] rounded to the nearest integer value
                ( status[1] < 128 ) ? ( pkt_status->params.lora.snr_pkt = status[1] >> 2 ) : ( pkt_status->params.lora.snr_pkt = ( ( status[1] - 256 ) >> 2 ) );
                //pkt_status->params.lora.snr_pkt = (((int8_t) status[1]) + 2) >> 2;
                pkt_status->params.lora.signal_rssi_pkt = -status[2] >> 1;
                pkt_status->params.lora.freq_err = freq_err;
                break;
        default:
        case PACKET_TYPE_NONE:
                // In that specific case, we set everything in the packet_status to zeros
                // and reset the packet type accordingly
                memset(pkt_status, 0, sizeof(packet_status_t));
                pkt_status->packet_type = PACKET_TYPE_NONE;
                break;
        }
}


radio_err_t sx126x_get_device_errors(void)
{
        radio_err_t err;
        sx126x_read_cmd(RADIO_GET_ERROR, (uint8_t *) &err, 2);
        return err;
}


void sx126x_clear_device_erros(void)
{
        uint8_t buf[2] = {0x00, 0x00};
        sx126x_write_cmd(RADIO_CLR_ERROR, buf, 2);
}

void sx126x_clear_irq_status(uint16_t irq)
{
        uint8_t buf[2];

        buf[0] = (uint8_t) (((uint16_t) irq >> 8) & 0x00FF);
        buf[1] = (uint8_t) ((uint16_t) irq & 0x00FF);
        sx126x_write_cmd(RADIO_CLR_IRQ_STATUS, buf, 2);
}

void sx126x_set_tx(uint32_t timeout)
{
        uint8_t buf[3];

        sx126x_set_operating_mode(MODE_TX);

        // Aron
        sx126x_clear_irq_status(IRQ_RADIO_ALL);
        buf[0] = (uint8_t) ((timeout >> 16) & 0xFF);
        buf[1] = (uint8_t) ((timeout >> 8) & 0xFF);
        buf[2] = (uint8_t) (timeout & 0xFF);
        sx126x_write_cmd(RADIO_SET_TX, buf, 3);
}


void sx126x_set_rx(uint32_t timeout)
{
        uint8_t buf[3];

        sx126x_set_operating_mode(MODE_RX);

        sx126x_clear_irq_status(IRQ_RADIO_ALL);
        buf[0] = (uint8_t) ((timeout >> 16) & 0xFF);
        buf[1] = (uint8_t) ((timeout >> 8) & 0xFF);
        buf[2] = (uint8_t) (timeout & 0xFF);
        sx126x_write_cmd(RADIO_SET_RX, buf, 3);
}

void sx126x_set_fs(void)
{
        sx126x_write_cmd(RADIO_SET_FS, 0, 0);
        sx126x_set_operating_mode(MODE_FS);
}

void sx126x_set_standby(radio_standy_mode_t standby_config)
{
        sx126x_write_cmd(RADIO_SET_STANDBY, (uint8_t *) &standby_config, 1);
        if (standby_config == STDBY_RC) {
                sx126x_set_operating_mode(MODE_STDBY_RC);
        } else {
                sx126x_set_operating_mode(MODE_STDBY_XOSC);
        }
}

void sx126x_set_sleep(sleep_params_t sleep_config)
{
        sx126x_switch_off();

        sx126x_write_cmd(RADIO_SET_SLEEP, &sleep_config.value, 1);
        sx126x_set_operating_mode(MODE_SLEEP);
}


void sx126x_set_rx_boosted(uint32_t timeout)
{
        uint8_t buf[3];

        sx126x_set_operating_mode(MODE_RX);

        sx126x_write_register(REG_RX_GAIN, 0x96);	// max LNA gain, increase current by ~2mA for around ~ 3dB in sensivity

        buf[0] = (uint8_t) ((timeout >> 16) & 0xFF);
        buf[1] = (uint8_t) ((timeout >> 8) & 0xFF);
        buf[2] = (uint8_t) (timeout  & 0xFF);
        sx126x_write_cmd(RADIO_SET_RX, buf, 3);
}

void sx126x_set_rx_duty_cycle(uint32_t rx_time, uint32_t sleep_time)
{
        uint8_t buf[6];

        buf[0] = (uint8_t) ((rx_time >> 16) & 0xFF);
        buf[1] = (uint8_t) ((rx_time >> 8) & 0xFF);
        buf[2] = (uint8_t) (rx_time & 0xFF);
        buf[3] = (uint8_t) ((sleep_time >> 16) & 0xFF);
        buf[4] = (uint8_t) ((sleep_time >> 8) & 0xFF);
        buf[5] = (uint8_t) (sleep_time & 0xFF);
        sx126x_write_cmd(RADIO_SET_RX_DUTYCYCLE, buf, 6);
        sx126x_set_operating_mode(MODE_RX_DC);
}

void sx126x_set_cad(void)
{
        // Aron
        // sx126x_cad_en();

        sx126x_write_cmd(RADIO_SET_CAD, 0, 0);
        sx126x_set_operating_mode(MODE_CAD);
}

void sx126x_set_tx_continuous_wave(void)
{
        sx126x_write_cmd(RADIO_SET_TX_CONT_WAVE, 0, 0);
}

void sx126x_set_tx_infinite_preamble(void)
{
        sx126x_write_cmd(RADIO_SET_TX_CONT_PREAMBLE, 0, 0);
}

void sx126x_set_stop_rx_timer_on_preamble_detect(bool enable)
{
        sx126x_write_cmd(RADIO_SET_STOP_RX_TIMER_ON_PREAMBLE, (uint8_t *) &enable, 1);
}

void sx126x_set_lora_symbol_num_timeout(uint8_t symbol_num)
{
        sx126x_write_cmd(RADIO_SET_LORA_SYMB_TIMEOUT, &symbol_num, 1);
}

void sx126x_set_regulator_mode(sx126x_regulator_mode_t mode)
{
        sx126x_write_cmd(RADIO_SET_REGULATOR_MODE, (uint8_t *) &mode, 1);
}

void sx126x_set_payload(uint8_t *payload, uint8_t size)
{
        sx126x_write_buffer(0x00, payload, size);
}

uint8_t sx126x_get_payload(uint8_t *buffer, uint8_t *size, uint8_t max_size)
{
        uint8_t offset = 0;

        sx126x_get_rx_buffer_status(size, &offset);
        if (*size > max_size) {
                return 1;
        }
        sx126x_read_buffer(offset, buffer, *size);
        return 0;
}

void sx126x_send_payload(uint8_t *payload, uint8_t size, uint32_t timeout)
{
        sx126x_set_payload(payload, size);
        sx126x_set_tx(timeout);
}

void sx126x_calibrate(calibration_params_t calib_param)
{
        sx126x_write_cmd(RADIO_CALIBRATE, (uint8_t *) &calib_param, 1);
}

void sx126x_calibrate_image(uint32_t freq)
{
        uint8_t cal_freq[2];

        if( freq > 900000000 ) {
                cal_freq[0] = 0xE1;
                cal_freq[1] = 0xE9;
        } else if( freq > 850000000 ) {
                cal_freq[0] = 0xD7;
                cal_freq[1] = 0xDB;
        } else if( freq > 770000000 ) {
                cal_freq[0] = 0xC1;
                cal_freq[1] = 0xC5;
        } else if( freq > 460000000 ) {
                cal_freq[0] = 0x75;
                cal_freq[1] = 0x81;
        } else if( freq > 425000000 ) {
                cal_freq[0] = 0x6B;
                cal_freq[1] = 0x6F;
        }
        sx126x_write_cmd(RADIO_CALIBRATE_IMAGE, cal_freq, 2);
}


void sx126x_set_pa_config(uint8_t pa_duty_cycle, uint8_t hp_max, uint8_t device_sel, uint8_t pa_lut)
{
        uint8_t buf[4];

        buf[0] = pa_duty_cycle;
        buf[1] = hp_max;
        buf[2] = device_sel;
        buf[3] = pa_lut;
        sx126x_write_cmd(RADIO_SET_PACONFIG, buf, 4);
}

void sx126x_set_rx_tx_fallback_mode(uint8_t fallback_mode)
{
        sx126x_write_cmd(RADIO_SET_TX_FALLBACK_MODE, &fallback_mode, 1);
}

void sx126x_set_dio_irq_params( uint16_t irq_mask, uint16_t dio1_mask, uint16_t dio2_mask, uint16_t dio3_mask )
{
        uint8_t buf[8];

        buf[0] = ( uint8_t )( ( irq_mask >> 8 ) & 0x00FF );
        buf[1] = ( uint8_t )( irq_mask & 0x00FF );
        buf[2] = ( uint8_t )( ( dio1_mask >> 8 ) & 0x00FF );
        buf[3] = ( uint8_t )( dio1_mask & 0x00FF );
        buf[4] = ( uint8_t )( ( dio2_mask >> 8 ) & 0x00FF );
        buf[5] = ( uint8_t )( dio2_mask & 0x00FF );
        buf[6] = ( uint8_t )( ( dio3_mask >> 8 ) & 0x00FF );
        buf[7] = ( uint8_t )( dio3_mask & 0x00FF );

        sx126x_write_cmd( RADIO_CFG_DIO_IRQ, buf, 8 );
}

uint16_t sx126x_get_irq_status( void )
{
        uint8_t irqStatus[2];

        sx126x_read_cmd( RADIO_GET_IRQ_STATUS, irqStatus, 2 );
        return ( irqStatus[0] << 8 ) | irqStatus[1];
}

void sx126x_set_dio2_as_rf_switch_ctrl( uint8_t enable )
{
        sx126x_write_cmd( RADIO_SET_RF_SWITCH_MODE, &enable, 1 );
}

void sx126x_set_dio3_as_tcxo_ctrl( radio_tcxo_ctrl_voltage_t tcxo_voltage, uint32_t timeout )
{
        uint8_t buf[4];

        buf[0] = tcxo_voltage & 0x07;
        buf[1] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
        buf[2] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
        buf[3] = ( uint8_t )( timeout & 0xFF );

        sx126x_write_cmd( RADIO_SET_TCXO_MODE, buf, 4 );
}

void sx126x_set_rf_frequency( uint32_t frequency )
{
        uint8_t buf[4];
        uint32_t freq = 0;

#if 0
        if( image_calibrated == false ) {
                sx126x_calibrate_image( frequency );
                image_calibrated = true;
        }
#endif

        sx126x_set_standby(STDBY_RC);
        sx126x_calibrate_image(frequency);

        freq = ( uint32_t )( ( double ) frequency / ( double )FREQ_STEP );
        buf[0] = ( uint8_t )( ( freq >> 24 ) & 0xFF );
        buf[1] = ( uint8_t )( ( freq >> 16 ) & 0xFF );
        buf[2] = ( uint8_t )( ( freq >> 8 ) & 0xFF );
        buf[3] = ( uint8_t )( freq & 0xFF );
        sx126x_write_cmd( RADIO_SET_RF_FREQ, buf, 4 );
}

void sx126x_set_tx_params( int8_t power, radio_ramp_times_t rampTime )
{
        uint8_t buf[2];

        sx126x_set_pa_config( 0x04, 0x07, 0x00, 0x01 );
        if( power > 22 ) {
                power = 22;
        } else if( power < -3 ) {
                power = -3;
        }
        sx126x_write_register( REG_OCP, 0x38 ); // current max 140mA for the whole device
        buf[0] = power;
        buf[1] = ( uint8_t ) rampTime;
        sx126x_write_cmd( RADIO_SET_TX_PARAMS, buf, 2 );
}

uint8_t sx126x_set_sync_word(uint8_t *sync_word)
{
        sx126x_write_registers(REG_LR_SYNC_WORD_BASE_ADDR, sync_word, 8);
        return 0;
}

uint32_t sx126x_get_random(void)
{
        uint8_t buf[] = {0, 0, 0, 0};

        // Set radio in continuous reception
        sx126x_set_rx(0);

        mdelay(1);

        sx126x_read_registers(RANDOM_NUMBER_GEN_BASE_ADDR, buf, 4);

        sx126x_set_standby(STDBY_RC);

        return (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
}

void sx126x_set_rf_tx_power(int8_t power)
{
        sx126x_set_tx_params(power, RADIO_RAMP_800_US);
}

uint32_t sx126x_get_board_txco_wakeup_time(void)
{
        //return BOARD_TCXO_WAKEUP_TIME;
        return 2000;
}

void sx126x_init(dio_irq_handler_fn dio_irq_cb)
{
        bsp_sx126x_init(dio_irq_cb);

        // sx126x module init
        sx126x_reset();
        sx126x_wait_on_busy();
        sx126x_wakeup();
        sx126x_set_standby(STDBY_RC);

#ifdef USE_TXCO
        calibration_params_t calib_param;

        sx126x_set_dio3_as_tcxo_ctrl(TCXO_CTRL_2_7V, 6400);	// Convert from ms to sx126x time base
        calib_param.value = 0x7F;
        sx126x_calibrate(calib_param);
#endif /* USE_TXCO */

        //sx126x_set_dio2_as_rf_switch_ctrl(false);
        sx126x_set_operating_mode(MODE_STDBY_RC);
}

