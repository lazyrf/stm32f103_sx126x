#ifndef __SX126X_H
#define __SX126X_H

#include "config.h"
#include "bsp_sx126x.h"
#include <stdio.h>
#include <stdbool.h>

#define XTAL_FREQ				   	 ( double ) 32000000
#define FREQ_DIV				    	( double ) pow( 2.0,  25.0 )
#define FREQ_STEP				    	( double ) ( XTAL_FREQ / FREQ_DIV )

/*!
 * Radio complete Wake-up Time with margin for temperature compensation
 */
#define RADIO_WAKEUP_TIME			    3 // [ms]


/**
 * brief The address of the register holding the packet configuration
 */
#define REG_LR_PACKET_PARAMS			0x0704

/**
 * brief The address of the register holding the payload size
 */
#define REG_LR_PAYLOAD_LEN			0x0702

/**
 * brief LFSR initial value to compute IBM type CRC
 */
#define CRC_IBM_SEED				0xFFFF

/**
 * brief LFSR initial value to compute CCIT type CRC
 */
#define CRC_CCITT_SEED				0x1D0F

/**
 * brief Polynomial used to compute IBM CRC
 */
#define CRC_POLYNOMIAL_IBM			0x8005

/**
 * brief Polynomial used to compute CCIT CRC
 */
#define CRC_POLYNOMIAL_CCITT			0x1021

/**
 * brief The address of the register holding the first byte defining the CRC seed
 */
#define REG_LR_CRC_SEED_BASE_ADDR		0x06BC


/**
 * brief The address of the register holding the first byte defining the CRC polynomial
 */
#define REG_LR_CRC_POLY_BASE_ADDR		0x06BE


/*!
 * \brief The addresses of the registers holding SyncWords values
 */
#define REG_LR_SYNC_WORD_BASE_ADDR		    0x06C0


/*!
 * \brief The address of the register holding the first byte defining the whitening seed
 */
#define REG_LR_WHIT_SEED_BASE_ADDR_MSB		    0x06B8
#define REG_LR_WHIT_SEED_BASE_ADDR_LSB		    0x06B9

/*!
 * The address of the register giving a 4 bytes random number
 */
#define RANDOM_NUMBER_GEN_BASE_ADDR 	    0x0819

/*!
 * The address of the register holding RX Gain value (0x94: power saving, 0x96: rx boosted)
 */
#define REG_RX_GAIN				    0x08AC

/*!
 * Change the value on the device internal trimming capacitor
 */
#define REG_XTA_TRIM				    0x0911

/*!
 * Set the current max value in the over current protection
 */
#define REG_OCP 				    0x08E7


/*!
 * \brief The addresses of the register holding LoRa Modem SyncWord value
 */
#define REG_LR_SYNCWORD 			    0x0740

/*!
 * Syncword for Private LoRa networks
 */
#define LORA_MAC_PRIVATE_SYNCWORD		    0x1424

/*!
 * Syncword for Public LoRa networks
 */
#define LORA_MAC_PUBLIC_SYNCWORD		    0x3444

/*!
 * The address of the register holding Bit Sync configuration
 */
#define REG_BIT_SYNC                                0x06AC


#define USE_TXCO

/**
 * brief	Represents all possible opcode
 */
typedef enum radio_cmd_e {
        RADIO_GET_STATUS                = 0xC0,
        RADIO_WRITE_REGISTER            = 0x0D,
        RADIO_READ_REGISTER             = 0x1D,
        RADIO_WRITE_BUFFER              = 0x0E,
        RADIO_READ_BUFFER               = 0x1E,
        RADIO_SET_SLEEP         = 0x84,
        RADIO_SET_STANDBY               = 0x80,
        RADIO_SET_FS            = 0xC1,
        RADIO_SET_TX            = 0x83,
        RADIO_SET_RX			= 0x82,
        RADIO_SET_RX_DUTYCYCLE          = 0x94,
        RADIO_SET_CAD			= 0xC5,
        RADIO_SET_TX_CONT_WAVE          = 0xD1,
        RADIO_SET_TX_CONT_PREAMBLE              = 0xD2,
        RADIO_SET_PACKET_TYPE           = 0x8A,
	SX126X_GET_PACKET_TYPE		= 0x11,
        RADIO_SET_RF_FREQ               = 0x86,
        RADIO_SET_TX_PARAMS             = 0x8E,
        RADIO_SET_PACONFIG              = 0x95,
        RADIO_SET_CAD_PARAMS            = 0x88,
        RADIO_SET_BUFF_BASE_ADDR                = 0x8F,
        RADIO_SET_MODULATION_PARAMS	= 0x8B,
        RADIO_SET_PACKET_PARAMS	= 0x8C,
        RADIO_GET_RX_BUFF_STATUS	= 0x13,
        RADIO_GET_PACKET_STATUS	= 0x14,
        RADIO_GET_RSSI_INST             = 0x15,
	SX126X_GET_STATS			= 0x10,
	SX126X_RESET_STATS		= 0x00,
        RADIO_CFG_DIO_IRQ               = 0x08,
        RADIO_GET_IRQ_STATUS            = 0x12,
        RADIO_CLR_IRQ_STATUS            = 0x02,
        RADIO_CALIBRATE         = 0x89,
        RADIO_CALIBRATE_IMAGE           = 0x98,
        RADIO_SET_REGULATOR_MODE                = 0x96,
        RADIO_GET_ERROR         = 0x17,
        RADIO_CLR_ERROR         = 0x07,
        RADIO_SET_TCXO_MODE		= 0x97,
        RADIO_SET_TX_FALLBACK_MODE              = 0x93,
        RADIO_SET_RF_SWITCH_MODE                = 0x9D,
        RADIO_SET_STOP_RX_TIMER_ON_PREAMBLE             = 0x9F,
        RADIO_SET_LORA_SYMB_TIMEOUT	= 0xA0,
} radio_cmd_t;

/**
 * brief 	Represents the operating mode the radio is actually running
 */
typedef enum {
        MODE_SLEEP 		= 0x00,	// Sleep mode
        MODE_STDBY_RC,			// Standby mode with RC oscillator
        MODE_STDBY_XOSC,		// Standby mode with XSOC oscillator
        MODE_FS,			// Frequency synthesis mode
        MODE_TX,			// Transmit mode
        MODE_RX,			// Receivie mode
        MODE_RX_DC,			// Receive duty cycle mode
        MODE_CAD,			// Channel activity detection mode
} radio_operating_mode_t;


/*!
 * \brief Declares the oscillator in use while in standby mode
 *
 * Using the STDBY_RC standby mode allow to reduce the energy consumption
 * STDBY_XOSC should be used for time critical applications
 */
typedef enum
{
        STDBY_RC		= 0x00,
        STDBY_XOSC		= 0x01,
} radio_standy_mode_t;


/**
 * brief	Represents the number of symbols to be used for channel activity detection operation
 */
typedef enum {
        LORA_CAD_01_SYMBOL	= 0x00,
        LORA_CAD_02_SYMBOL	= 0x01,
        LORA_CAD_04_SYMBOL 	= 0x02,
        LORA_CAD_08_SYMBOL	= 0x03,
        LORA_CAD_16_SYMBOL	= 0x04,
} radio_lora_cad_symbols_t;

/**
 * brief	Represents the Channel Activity Detection actions after the CAD operation is finished
 */
typedef enum {
        LORA_CAD_ONLY 	= 0x00,
        LORA_CAD_RX	= 0x01,
        LORA_CAD_LBT	= 0x10,
} radio_cad_exit_modes_t;

/**
 * brief	Represents the preamble length used to detect the packet on RX side
 */
typedef enum {
	PREAMBLE_DETECTOR_OFF		= 0x00,	//Preamblel detection lenghth off
	PREAMBLE_DETECTOR_08_BITS	= 0x04,	// Preamble detection length 8 bits
	PREAMBLE_DETECTOR_16_BITS	= 0x05,	// Preamble detection length 16 bits
	PREAMBLE_DETECTOR_24_BITS	= 0x06,	// Preamble detection length 24 bits
	PREAMBLE_DETECTOR_32_BITS	= 0x07,	// Preamble detection length 32 bits
}  sx126x_preamble_detection_t;

/**
 * brief	Represents the possible combinations of sync word correlators activated
 */
typedef enum {
	ADDR_COMP_FILT_OFF		= 0x00,	// No corrlator turned on, ie. do not search for sync word
	ADDR_COMP_FILT_NODE		= 0x01,
	ADDR_COMP_FILT_NODE_BOARD	= 0x02,
} sx126x_addr_cmp_t;


/**
 * brief	GFSK packet length mode
 */
typedef enum {
	PACKET_FIXED_LENGTH		= 0x00,	// The packet is known on both sides, no header included in the packet
	PACKET_VARIABLE_LENGTH		= 0x01,	// The packet is on variable size, header included
} sx126x_packet_length_modes_t;


/**
 * brief	Represents the CRC length
 */
typedef enum {
	CRC_OFF			= 0x01,	// No CRC in use
	CRC_1_BYTES		= 0x00,
	CRC_2_BYTES		= 0x02,
	CRC_1_BYTES_INV		= 0x04,
	CRC_2_BYTES_INV		= 0x06,
	CRC_2_BYTES_IBM		= 0xF1,
	CRC_2_BYTES_CCIT		= 0xF2,
} sx126x_crc_type_t;

/**
 * brief	Radio whitening mode activated or deactivated
 */
typedef enum {
	DC_FREE_OFF		= 0x00,
	DC_FREE_WHITENING	= 0x01,
} sx126x_dc_free_t;

/**
 * brief	Holds the radio lengths mode fro the LoRa packet type
 */
typedef enum {
        LORA_PACKET_VARIABLE_LENGTH		= 0x00,	// The packet is on variable size, header included
        LORA_PACKET_FIXED_LENGTH		= 0x01,	// The packet is known on both sides, no header included in the packet
        LORA_PACKET_EXPLICIT			= LORA_PACKET_VARIABLE_LENGTH,
        LORA_PACKET_IMPLICIT			= LORA_PACKET_FIXED_LENGTH,
} radio_lora_packet_length_mode_t;

/**
 * brief	Represents the CRC mode for LoRa packet tyep
 */
typedef enum {
        LORA_CRC_ON		= 0x01,	// CRC activated
        LORA_CRC_OFF		= 0x00,	// CRC not used
} radio_lora_crc_mode_t;


/**
 * brief	Represents the IQ mode for LoRa packet type
 */
typedef enum {
        LORA_IQ_NORMAL		= 0x00,
        LORA_IQ_INVERTED		= 0x01,
} radio_lora_iq_mode_t;


/**
 * brief	Represents the modulation shaping parameter
 */
typedef enum {
	MOD_SHAPING_OFF		= 0x00,
	MOD_SHAPING_G_BT_03		= 0x08,
	MOD_SHAPING_G_BT_05		= 0x09,
	MOD_SHAPING_G_BT_07		= 0x0A,
	MOD_SHAPING_G_BT_1		= 0x0B,
} sx126x_mod_shaping_t;


/*!
 * \brief Declares the power regulation used to power the device
 *
 * This command allows the user to specify if DC-DC or LDO is used for power regulation.
 * Using only LDO implies that the Rx or Tx current is doubled
 */
typedef enum
{
	USE_LDO				    = 0x00, // default
	USE_DCDC				    = 0x01,
} sx126x_regulator_mode_t;


/*!
 * \brief Represents the voltage used to control the TCXO on/off from DIO3
 */
typedef enum
{
        TCXO_CTRL_1_6V			    = 0x00,
        TCXO_CTRL_1_7V			    = 0x01,
        TCXO_CTRL_1_8V			    = 0x02,
        TCXO_CTRL_2_2V			    = 0x03,
        TCXO_CTRL_2_4V			    = 0x04,
        TCXO_CTRL_2_7V			    = 0x05,
        TCXO_CTRL_3_0V			    = 0x06,
        TCXO_CTRL_3_3V			    = 0x07,
} radio_tcxo_ctrl_voltage_t;


/*!
 * \brief Represents the ramping time for power amplifier
 */
typedef enum
{
        RADIO_RAMP_10_US			    = 0x00,
        RADIO_RAMP_20_US			    = 0x01,
        RADIO_RAMP_40_US			    = 0x02,
        RADIO_RAMP_80_US			    = 0x03,
        RADIO_RAMP_200_US			    = 0x04,
        RADIO_RAMP_800_US			    = 0x05,
        RADIO_RAMP_1700_US			    = 0x06,
        RADIO_RAMP_3400_US			    = 0x07,
} radio_ramp_times_t;


/*!
 * \brief Represents a calibration configuration
 */
typedef union {
	struct {
		uint8_t rc64k_enable	: 1;			//!< Calibrate RC64K clock
		uint8_t rc13m_enable    	: 1;			//!< Calibrate RC13M clock
		uint8_t pll_enable      	: 1;			//!< Calibrate PLL
		uint8_t adc_pulse_enable 	: 1;			//!< Calibrate ADC Pulse
		uint8_t adc_bulkn_enable 	: 1;			//!< Calibrate ADC bulkN
		uint8_t adc_bulkp_enable 	: 1;			//!< Calibrate ADC bulkP
		uint8_t img_enable      	: 1;
		uint8_t 	       		: 1;
	} fields;
	uint8_t value;
} calibration_params_t;


/**
 * brief	Represents the possible radio system error status
 */
typedef union {
        struct {
                uint8_t rc64k_calib: 1;	// RC 64KHz oscillator calibration failed
                uint8_t rc13m_calib: 1;	// RC 13MHz clibration failed
                uint8_t pll_calib: 1;		// PLL calibration failed
                uint8_t adc_calib: 1;		// ADC calibration failed
                uint8_t img_calib: 1;	// IMG calibration failed
                uint8_t xosc_start: 1;	// XOSC_ oscillator failed to start
                uint8_t pll_lock: 1;		// PLL lock failed
                uint8_t buk_start: 1;		// Buck converter fiailed to start
                uint8_t pa_ramp: 1;		// PA ramp failed
                uint8_t reserved: 7;		// Reserved
        } fields;
        uint16_t value;
} radio_err_t;

/**
 * brief	Represents the possible packet type (i.e. modem) used
 */
typedef enum {
        PACKET_TYPE_GFSK	= 0x00,
        PACKET_TYPE_LORA	= 0x01,
        PACKET_TYPE_NONE 	= 0x0F,
} radio_packet_type_t;


/**
 * brief	Represents the possible spreading factor values in LoRa packet types
 */
typedef enum {
        LORA_SF5	= 0x05,
        LORA_SF6 	= 0x06,
        LORA_SF7	= 0x07,
        LORA_SF8	= 0x08,
        LORA_SF9	= 0x09,
        LORA_SF10	= 0x0A,
        LORA_SF11	= 0x0B,
        LORA_SF12	= 0x0C,
} radio_lora_spreading_factor_t;


/**
 * brief	Represents the bandwidth values for LoRa packe type
 */
typedef enum {
        LORA_BW_500	= 6,
        LORA_BW_250	= 5,
        LORA_BW_125	= 4,
        LORA_BW_062	= 3,
        LORA_BW_041	= 10,
        LORA_BW_031	= 2,
        LORA_BW_020	= 9,
        LORA_BW_015	= 1,
        LORA_BW_010	= 8,
        LORA_BW_007	= 0,
} radio_lora_bandwidth_t;


/**
 * brief	Represents the coding rate value for LoRa packet type
 */
typedef enum {
	LORA_CR_4_5	= 0x01,
	LORA_CR_4_6	= 0x02,
	LORA_CR_4_7	= 0x03,
	LORA_CR_4_8	= 0x04,
} radio_lora_coding_rate_t;

/*!
 * \brief Represents the interruption masks available for the radio
 *
 * \remark Note that not all these interruptions are available for all packet types
 */
typedef enum
{
	IRQ_RADIO_NONE			    = 0x0000,
	IRQ_TX_DONE 			    = 0x0001,
	IRQ_RX_DONE 			    = 0x0002,
	IRQ_PREAMBLE_DETECTED		    = 0x0004,
	IRQ_SYNCWORD_VALID			    = 0x0008,
	IRQ_HEADER_VALID			    = 0x0010,
	IRQ_HEADER_ERROR			    = 0x0020,
	IRQ_CRC_ERROR			    = 0x0040,
	IRQ_CAD_DONE			    = 0x0080,
	IRQ_CAD_ACTIVITY_DETECTED		    = 0x0100,
	IRQ_RX_TX_TIMEOUT			    = 0x0200,
	IRQ_RADIO_ALL			    = 0xFFFF,
} RadioIrqMasks_t;



/**
 * brief	Represents the packet status for every packet type
 */
typedef struct {
        radio_packet_type_t	packet_type;	// Packet to which the packet status are referring to.
        struct {
                struct {
                        uint8_t rx_status;
                        int8_t rssi_avg;	// The averaged RSSI
                        int8_t rssi_sync;	// The RSSI measure on last packet
                        uint32_t freq_err;
                } gfsk;
                struct {
                        int8_t rssi_pkt;	// The RSSI of the last packet
                        int8_t snr_pkt;	// The SNT of the last packet
                        int8_t signal_rssi_pkt;
                        uint32_t freq_err;
                } lora;
        } params;
} packet_status_t;

/**
 * brief	Structure describing the radio status
 */
typedef union sx126x_status_u {
	uint8_t value;
	struct {
		// bit order is lsb -> msb
		uint8_t reserved: 1;		// Reserved
		uint8_t cmd_status: 3;	// Command status
		uint8_t chip_mdoe: 3;	// Chip mode
		uint8_t cpu_busy: 1;		// Flag for CPU radio busy
	}  fields;
} sx126x_status_t;

/**
 * brief	The type describing the packet parameters for every packet types
 */
typedef struct {
        radio_packet_type_t	packet_type;	// Pacekt to which the packet parameters are referring to
        struct {
                //Holds the GFSK packet parameters
                struct {
                        uint16_t preamble_length;		// The preamble TX length for GFSK packet type in bit
                        sx126x_preamble_detection_t preamble_min_detect;	// The preamble RX length minimal for GFSK packet type
                        uint8_t sync_word_length;		// The synchronization word length for GFSK packet type
                        sx126x_addr_cmp_t addr_comp;	// Activated sync word correlators
                        sx126x_packet_length_modes_t header_type;	// If the header is explicit, it will be transmitted in the GFSK packet. If the header is implicit, it will not be transmitted
                        uint8_t payload_len;	// Size of the payload in GFSK packet
                        sx126x_crc_type_t crc_len;	// Size of the CRC block in the GFSK packet
                        sx126x_dc_free_t dc_free;
                } gfsk;
                //Holds the LoRa packet parameters
                struct {
                        uint16_t preamble_length;		// The preamble length is the number of LoRa symbols in the preamble
                        radio_lora_packet_length_mode_t header_type;	// If the headeer is explicit, it will be transmitted in the LoRa packet. If the header is implicit, it will not be transmitted
                        uint8_t payload_len;	// Size of the payload in LoRa packet
                        radio_lora_crc_mode_t crc_mode;	// Size of CRC block in LoRa packet
                        radio_lora_iq_mode_t invert_iq;	// Allows to swap IQ for LoRa packet
                } lora;
        } params;		// Holds the packet parameters structure
} packet_params_t;


/**
 * brief The type describing the modulation parameters for every packet types
 */
typedef struct {
        radio_packet_type_t packet_type;	// Packet to which the modulation parameters are referring to
        struct {
                struct {
                        uint32_t bit_rate;
                        uint32_t fdev;
                        sx126x_mod_shaping_t modulation_shaping;
                        uint8_t bandwidth;
                } gfsk;
                struct {
                        radio_lora_spreading_factor_t spreading_factor;	// Spreading factor for the LoRa modulation
                        radio_lora_bandwidth_t bandwith;		// Bandwith for the LoRa modulation
                        radio_lora_coding_rate_t coding_rate;	// Coding rate for the LoRa modulation
                        uint8_t low_data_rate_optimize;		// Indicates if the modem uses the low data rate optimation
                } lora;
        } params;
} modulation_params_t;


/**
 * brief	Represents a sleep mode configuration
 */
typedef union {
        struct {
                uint8_t wake_up_rtc: 1;	// Get out of sleep mode if wakeup signal receive from RTC
                uint8_t reset: 1;
                uint8_t warm_start: 1;
                uint8_t reserved: 5;
        } fields;
        uint8_t value;
} sleep_params_t;

/*!
 * Radio hardware and global parameters
 */
typedef struct sx126x_s{
	packet_params_t PacketParams;
	packet_status_t PacketStatus;
	modulation_params_t ModulationParams;
} sx126x_t;

void sx126x_reset(void);
void sx126x_wakeup(void);
void sx126x_write_registers(uint16_t addr, uint8_t *buffer, uint16_t size);
void sx126x_write_register(uint16_t addr, uint8_t value);
void sx126x_read_registers(uint16_t addr, uint8_t *buffer, uint16_t size);
uint8_t sx126x_read_register(uint16_t addr);
void sx126x_write_cmd(radio_cmd_t cmd, uint8_t *buffer, uint16_t size);
void sx126x_read_cmd(radio_cmd_t cmd, uint8_t *buffer, uint16_t size);
void sx126x_write_buffer(uint8_t offset, uint8_t *buffer, uint16_t size);
void sx126x_read_buffer(uint8_t offset, uint8_t *buffer, uint16_t size);
radio_operating_mode_t sx126x_get_operating_mode(void);
void sx126x_set_operating_mode(radio_operating_mode_t mode);
bool sx126x_check_rf_freq(uint32_t freq);
radio_packet_type_t sx126x_get_packet_type(void);
void sx126x_set_packet_type(radio_packet_type_t new_packet_type);
void sx126x_set_crc_seed(uint16_t seed);
void sx126x_set_crc_polynomial(uint16_t polynomial);
void sx126x_set_modulation_params(modulation_params_t *mod_params);
void sx126x_set_packet_params(packet_params_t *packet_params);
void sx126x_set_cad_params(radio_lora_cad_symbols_t cad_symbol_num, uint8_t cad_det_peak, uint8_t cad_det_min, radio_cad_exit_modes_t cad_exit_mode, uint32_t cad_timeout);
void sx126x_set_buffer_base_addr(uint8_t tx_base_addr, uint8_t rx_base_addr);
sx126x_status_t sx126x_get_status(void);
int8_t sx126x_get_rssi_inst(void);
void sx126x_get_rx_buffer_status(uint8_t *payload_len, uint8_t *rx_start_buffer_ptr);
void sx126x_get_packet_status(packet_status_t *pkt_status);
radio_err_t sx126x_get_device_errors(void);
void sx126x_clear_irq_status(uint16_t irq);
void sx126x_set_tx(uint32_t timeout);
void sx126x_set_rx(uint32_t timeout);
void sx126x_set_fs(void);
void sx126x_set_standby(radio_standy_mode_t standby_config);
void sx126x_set_sleep(sleep_params_t sleep_config);
void sx126x_set_rx_boosted(uint32_t timeout);
void sx126x_set_rx_duty_cycle(uint32_t rx_time, uint32_t sleep_time);
void sx126x_set_cad(void);
void sx126x_set_payload(uint8_t *payload, uint8_t size);
void sx126x_set_tx_continuous_wave(void);
void sx126x_set_tx_infinite_preamble(void);
void sx126x_set_stop_rx_timer_on_preamble_detect(bool enable);
void sx126x_set_lora_symbol_num_timeout(uint8_t symbol_num);
void sx126x_set_regulator_mode(sx126x_regulator_mode_t mode);
void sx126x_calibrate(calibration_params_t calib_param);
void sx126x_calibrate_image(uint32_t freq);
void sx126x_set_pa_config(uint8_t pa_duty_cycle, uint8_t hp_max, uint8_t device_sel, uint8_t pa_lut);
void sx126x_set_rx_tx_fallback_mode(uint8_t fallback_mode);
void sx126x_set_dio_irq_params( uint16_t irq_mask, uint16_t dio1_mask, uint16_t dio2_mask, uint16_t dio3_mask );
uint16_t sx126x_get_irq_status( void );
void sx126x_set_dio2_as_rf_switch_ctrl( uint8_t enable );
void sx126x_set_dio3_as_tcxo_ctrl( radio_tcxo_ctrl_voltage_t tcxo_voltage, uint32_t timeout );
void sx126x_set_rf_frequency( uint32_t frequency );
void sx126x_set_tx_params( int8_t power, radio_ramp_times_t rampTime );
uint8_t sx126x_get_payload(uint8_t *buffer, uint8_t *size, uint8_t max_size);
void sx126x_send_payload(uint8_t *payload, uint8_t size, uint32_t timeout);
uint8_t sx126x_set_sync_word(uint8_t *sync_word);
void sx126x_set_whitening_seed(uint16_t seed);
uint32_t sx126x_get_random(void);
void sx126x_set_rf_tx_power(int8_t power);
uint32_t sx126x_get_board_txco_wakeup_time(void);
void sx126x_init(dio_irq_handler_fn dio_irq_cb);
void sx126x_switch_off(void);
void sx126x_dio1_interrupt_enable(void);
void sx126x_dio1_interrupt_disable(void);
uint8_t sx126x_dio1_state_get(void);
void sx126x_dio2_interrupt_enable(void);
void sx126x_dio2_interrupt_disable(void);
uint8_t sx126x_dio2_state_get(void);
void sx126x_tx_en(void);
void sx126x_rx_en(void);
#endif /* __SX126X_H */

