/*!
 * \file      radio.c
 *
 * \brief     Radio driver API definition
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *		  ______			      _
 *		 / _____)	      _ 	     | |
 *		( (____  _____ ____ _| |_ _____  ____| |__
 *		 \____ \| ___ |    (_	_) ___ |/ ___)	_ \
 *		 _____) ) ____| | | || |_| ____( (___| | | |
 *		(______/|_____)_|_|_| \__)_____)\____)_| |_|
 *		(C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include <math.h>
#include <string.h>
#include "radio.h"
#include "timer.h"
#include "delay.h"

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio = {
        radio_init,
        radio_get_status,       // get_status
        radio_set_modem,        // set_modem
        radio_set_channel,      // set_channel
        radio_is_channel_free,  // is_channel_free
        radio_random,   // random
        radio_set_rx_config,    // set_rx_config
        radio_set_tx_config,    // set_tx_config
        radio_check_rf_frequency,       // check_rf_frequency
        radio_time_on_air,         // time_on_air
        radio_send,     // send
        radio_sleep,    // sleep
        radio_stanby,   // standby
        radio_start_rx, // start_rx
        radio_start_cad,  // start_cad
        radio_set_tx_continuous_wave,   // set_tx_continuous_wave
        radio_rssi,     // rssi
        radio_write,    // write
        radio_read,     // read
        radio_write_buffer,     // write_buffer
        radio_read_buffer,        // read_buffer
        radio_set_max_payload_length,   // set_max_payload_length
        radio_set_public_network,  // set_public_network
        radio_get_wakeup_time,     // get_wakeup_time
        radio_irq_process,      // irq_process
        // Available on SX126x only
        radio_rx_boosted, // rx_boosted
        radio_set_rx_duty_cycle // set_rx_duty_cycle
};

/*
 * Local types definition
 */


 /*!
 * FSK bandwidth definition
 */
typedef struct {
	uint32_t bandwidth;
	uint8_t  RegValue;
} FskBandwidth_t;

/*!
 * Precomputed FSK bandwidth registers values
 */
const FskBandwidth_t FskBandwidths[] = {
	{ 4800  , 0x1F },
	{ 5800  , 0x17 },
	{ 7300  , 0x0F },
	{ 9700  , 0x1E },
	{ 11700 , 0x16 },
	{ 14600 , 0x0E },
	{ 19500 , 0x1D },
	{ 23400 , 0x15 },
	{ 29300 , 0x0D },
	{ 39000 , 0x1C },
	{ 46900 , 0x14 },
	{ 58600 , 0x0C },
	{ 78200 , 0x1B },
	{ 93800 , 0x13 },
	{ 117300, 0x0B },
	{ 156200, 0x1A },
	{ 187200, 0x12 },
	{ 234300, 0x0A },
	{ 312000, 0x19 },
	{ 373600, 0x11 },
	{ 467000, 0x09 },
	{ 500000, 0x00 }, // Invalid Bandwidth
};

radio_lora_bandwidth_t Bandwidths[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500 };

//					    SF12    SF11    SF10    SF9    SF8	  SF7
static double RadioLoRaSymbTime[3][6] = {{ 32.768, 16.384, 8.192, 4.096, 2.048, 1.024 },  // 125 KHz
					{ 16.384, 8.192,  4.096, 2.048, 1.024, 0.512 },  // 250 KHz
					{ 8.192,  4.096,  2.048, 1.024, 0.512, 0.256 }}; // 500 KHz

uint8_t MaxPayloadLength = 0xFF;

uint32_t TxTimeout = 0;
uint32_t RxTimeout = 0;

bool RxContinuous = false;

packet_status_t radio_pkt_status;
uint8_t radio_rx_payload[255];

bool IrqFired = false;

// Aron
struct timer tx_timeout_timer;
struct timer rx_timeout_timer;

float g_bandwidth_khz = 500.0;
float g_ts_xms = 1.024; // 1.024ms

/*
 * SX126x DIO IRQ callback functions prototype
 */

/*!
 * \brief DIO 0 IRQ callback
 */
void radio_on_dio_irq( void* context );

/*!
 * \brief Tx timeout timer callback
 */
void RadioOnTxTimeoutIrq( void* context );

/*!
 * \brief Rx timeout timer callback
 */
void RadioOnRxTimeoutIrq( void* context );

/*
 * Private global variables
 */


/*!
 * Holds the current network type for the radio
 */
typedef struct {
	bool Previous;
	bool Current;
} RadioPublicNetwork_t;

static RadioPublicNetwork_t RadioPublicNetwork = { false };

/*!
 * Radio callbacks variable
 */
static radio_events_t* radio_events;

/*
 * Public global variables
 */

/*!
 * Radio hardware and global parameters
 */
sx126x_t SX126x;

extern void radio_irq_cb(void);

/*!
 * Returns the known FSK bandwidth registers value
 *
 * \param [IN] bandwidth Bandwidth value in Hz
 * \retval regValue Bandwidth register value.
 */
static uint8_t RadioGetFskBandwidthRegValue( uint32_t bandwidth )
{
	uint8_t i;

	if( bandwidth == 0 ) {
		return( 0x1F );
	}

	for( i = 0; i < ( sizeof( FskBandwidths ) / sizeof( FskBandwidth_t ) ) - 1; i++ ) {
		if( ( bandwidth >= FskBandwidths[i].bandwidth ) && ( bandwidth < FskBandwidths[i + 1].bandwidth ) ) {
			return FskBandwidths[i+1].RegValue;
		}
	}
	// ERROR: Value not found
	while( 1 );
}

bool radio_config_check(radio_config_t *config)
{
        if ((config->freq < 410000000) || (config->freq > 810000000)) {
                return false;
        }

        if ((Bandwidths[config->bandwidth] & 0x0F) > 0x0A) {
                return false;
        }

        switch (Bandwidths[config->bandwidth]) {
        case LORA_BW_500:
                g_bandwidth_khz = 500.0;
                break;
        case LORA_BW_250:
                g_bandwidth_khz = 250.0;
                break;
        case LORA_BW_125:
                g_bandwidth_khz = 125.0;
                break;
        case LORA_BW_062:
                g_bandwidth_khz = 62.5;
                break;
        case LORA_BW_041:
                g_bandwidth_khz = 41.67;
                break;
        case LORA_BW_031:
                g_bandwidth_khz = 31.25;
                break;
        case LORA_BW_020:
                g_bandwidth_khz = 20.83;
                break;
        case LORA_BW_015:
                g_bandwidth_khz = 15.63;
                break;
        case LORA_BW_010:
                g_bandwidth_khz = 10.42;
                break;
        case LORA_BW_007:
                g_bandwidth_khz = 7.81;
                break;
        }

        if ((config->spreading_factor > LORA_SF12) || (config->spreading_factor < LORA_SF5)) {
                return false;
        }

        g_ts_xms = (2 << ((config->spreading_factor) - 1)) / g_bandwidth_khz;

        if ((config->coding_rate > LORA_CR_4_8) || (config->coding_rate < LORA_CR_4_5)) {
                return false;
        }

        if (config->power_config > 22) {
                return false;
        }

        printf("\r\n================== LoRa config==================\r\n");
        printf("[Radio] Frequency: %ld Hz\r\n", config->freq);
        printf("[Radio] Bandwidth: %f KHz\r\n", g_bandwidth_khz);
        printf("[Radio] Spreading Factor: %d\r\n", config->spreading_factor);
        printf("[Radio] Coding Rate: %d\r\n", config->coding_rate);
        printf("[Radio] Power config: %d\r\n", config->power_config);
        printf("[Radio] Fixed packet length: %s\r\n", (config->fix_len) ? "YES" : "NO");
        printf("[Radio] CRC mode: %s\r\n", (config->crc_on) ? "ON" : "OFF");
        printf("[Radio] Invert IQ: %s\r\n", (config->invert_iq) ? "ON" : "OFF");
        printf("[Radio] Symbol timeout: %d\r\n", config->symbol_timeout);
        printf("[Radio] Preamble length: %d\r\n", config->preamble_len);
        printf("[Radio] Payload length: %d\r\n", config->payload_len);
        printf("================================================\r\n\r\n");

        return true;
}

bool radio_init( radio_events_t *events, radio_config_t *config)
{
        if (!radio_config_check(config)) {
                printf("[Radio] Config is incorrect\r\n");
                return false;
        }

        radio_events = events;

        sx126x_init( radio_on_dio_irq );
        sx126x_set_standby( STDBY_RC );
        sx126x_set_regulator_mode( USE_DCDC );

        sx126x_set_buffer_base_addr( 0x00, 0x00 );
#if 0 // Aron
        sx126x_set_tx_params( 0, RADIO_RAMP_200_US );
#endif
        sx126x_set_dio_irq_params( IRQ_RADIO_ALL, IRQ_RADIO_ALL, IRQ_RADIO_NONE, IRQ_RADIO_NONE );

        IrqFired = false;

        return true;
}

RadioState_t radio_get_status( void )
{
        switch( sx126x_get_operating_mode( ) ) {
        case MODE_TX:
                return RF_TX_RUNNING;
        case MODE_RX:
                return RF_RX_RUNNING;
        case MODE_CAD:
                return RF_CAD;
        default:
                return RF_IDLE;
        }
}

void radio_set_modem( RadioModems_t modem )
{
        switch( modem ) {
        default:
        case MODEM_FSK:
                sx126x_set_packet_type( PACKET_TYPE_GFSK );
                // When switching to GFSK mode the LoRa SyncWord register value is reset
                // Thus, we also reset the RadioPublicNetwork variable
                RadioPublicNetwork.Current = false;
                break;
        case MODEM_LORA:
                sx126x_set_packet_type( PACKET_TYPE_LORA );
                // Public/Private network register is reset when switching modems
                if( RadioPublicNetwork.Current != RadioPublicNetwork.Previous ) {
                        RadioPublicNetwork.Current = RadioPublicNetwork.Previous;
                        radio_set_public_network( RadioPublicNetwork.Current );
                }
                break;
        }
}

void radio_set_channel( uint32_t freq )
{
        sx126x_set_rf_frequency( freq );
}

bool radio_is_channel_free( RadioModems_t modem, uint32_t freq, int16_t rssiThresh, uint32_t maxCarrierSenseTime )
{
        bool status = true;
#if 0 /* Aron */
        int16_t rssi = 0;
        uint32_t carrierSenseTime = 0;
#endif

        radio_set_modem( modem );

        radio_set_channel( freq );

        radio_start_rx( 0 );

        mdelay( 1 );
    
#if 0 /* Aron */
        carrierSenseTime = TimerGetCurrentTime( );

        // Perform carrier sense for maxCarrierSenseTime
        while( TimerGetElapsedTime( carrierSenseTime ) < maxCarrierSenseTime ) {
                rssi = radio_rssi( modem );

                if( rssi > rssiThresh ) {
                        status = false;
                        break;
                }
        }
#endif

        radio_sleep( );
        return status;
}

uint32_t radio_random( void )
{
        uint8_t i;
        uint32_t rnd = 0;

        /*
         * Radio setup for random number generation
         */
        // Set LoRa modem ON
        radio_set_modem( MODEM_LORA );

        // Set radio in continuous reception
        sx126x_set_rx( 0 );

        for( i = 0; i < 32; i++ ) {
                mdelay( 1 );
                // Unfiltered RSSI value reading. Only takes the LSB value
                rnd |= ( ( uint32_t )sx126x_get_rssi_inst( ) & 0x01 ) << i;
        }

        radio_sleep( );

        return rnd;
}

void radio_set_rx_config( RadioModems_t modem, uint32_t bandwidth,
			 uint32_t datarate, uint8_t coderate,
			 uint32_t bandwidthAfc, uint16_t preambleLen,
			 uint16_t symbTimeout, bool fixLen,
			 uint8_t payloadLen,
			 bool crcOn, bool freqHopOn, uint8_t hopPeriod,
			 bool iqInverted, bool rxContinuous )
{
        RxContinuous = rxContinuous;

        if( rxContinuous == true ) {
                symbTimeout = 0;
        }

        if( fixLen == true ) {
                MaxPayloadLength = payloadLen;
        } else {
                MaxPayloadLength = 0xFF;
        }

        switch( modem ) {
        case MODEM_FSK:
                sx126x_set_stop_rx_timer_on_preamble_detect(false);
                // Modulation Parameter
                SX126x.ModulationParams.packet_type= PACKET_TYPE_GFSK;
                SX126x.ModulationParams.params.gfsk.bit_rate = datarate;
                SX126x.ModulationParams.params.gfsk.modulation_shaping = MOD_SHAPING_G_BT_1;
                SX126x.ModulationParams.params.gfsk.bandwidth = RadioGetFskBandwidthRegValue( bandwidth );
                // Packet Parameter
                SX126x.PacketParams.packet_type= PACKET_TYPE_GFSK;
                SX126x.PacketParams.params.gfsk.preamble_length = ( preambleLen << 3 ); // convert byte into bit
                SX126x.PacketParams.params.gfsk.preamble_min_detect = PREAMBLE_DETECTOR_08_BITS;
                SX126x.PacketParams.params.gfsk.sync_word_length = 3 << 3; // convert byte into bit
                SX126x.PacketParams.params.gfsk.addr_comp = ADDR_COMP_FILT_OFF;
                SX126x.PacketParams.params.gfsk.header_type = ( fixLen == true ) ? PACKET_FIXED_LENGTH : PACKET_VARIABLE_LENGTH;
                SX126x.PacketParams.params.gfsk.payload_len = MaxPayloadLength;
                if( crcOn == true ) {
                        SX126x.PacketParams.params.gfsk.crc_len = CRC_2_BYTES_CCIT;
                } else {
                        SX126x.PacketParams.params.gfsk.crc_len = CRC_OFF;
                }
                SX126x.PacketParams.params.gfsk.dc_free = DC_FREE_WHITENING;

                radio_stanby( );
                radio_set_modem( ( SX126x.ModulationParams.packet_type== PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
                sx126x_set_modulation_params( &SX126x.ModulationParams );
                sx126x_set_packet_params( &SX126x.PacketParams );
                sx126x_set_sync_word( ( uint8_t[] ){ 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 } );
                sx126x_set_whitening_seed( 0x01FF );

                RxTimeout = ( uint32_t )( symbTimeout * ( ( 1.0 / ( double )datarate ) * 8.0 ) * 1000 );
                break;

        case MODEM_LORA:
                sx126x_set_stop_rx_timer_on_preamble_detect( false );
                sx126x_set_lora_symbol_num_timeout( symbTimeout );
                // Modulation parameter
                SX126x.ModulationParams.packet_type= PACKET_TYPE_LORA;
                SX126x.ModulationParams.params.lora.spreading_factor = ( radio_lora_spreading_factor_t ) datarate;
                SX126x.ModulationParams.params.lora.bandwith = Bandwidths[bandwidth];
                SX126x.ModulationParams.params.lora.coding_rate = ( radio_lora_coding_rate_t ) coderate;
                if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
                        ( ( bandwidth == 1 ) && ( datarate == 12 ) ) ) {
                        SX126x.ModulationParams.params.lora.low_data_rate_optimize = 0x01;
                } else {
                        SX126x.ModulationParams.params.lora.low_data_rate_optimize = 0x00;
                }

                // Packet parameter
                SX126x.PacketParams.packet_type= PACKET_TYPE_LORA;
                if( ( SX126x.ModulationParams.params.lora.spreading_factor == LORA_SF5 ) ||
                        ( SX126x.ModulationParams.params.lora.spreading_factor == LORA_SF6 ) ) {
                        if( preambleLen < 12 ) {
                                SX126x.PacketParams.params.lora.preamble_length = 12;
                        } else {
                                SX126x.PacketParams.params.lora.preamble_length = preambleLen;
                        }
                } else {
                        SX126x.PacketParams.params.lora.preamble_length = preambleLen;
                }
                SX126x.PacketParams.params.lora.header_type = ( radio_lora_packet_length_mode_t ) fixLen;
                SX126x.PacketParams.params.lora.payload_len = MaxPayloadLength;
                SX126x.PacketParams.params.lora.crc_mode = ( radio_lora_crc_mode_t ) crcOn;
                SX126x.PacketParams.params.lora.invert_iq = ( radio_lora_iq_mode_t ) iqInverted;

                radio_set_modem( ( SX126x.ModulationParams.packet_type== PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
                sx126x_set_modulation_params( &SX126x.ModulationParams );
                sx126x_set_packet_params( &SX126x.PacketParams );

                // Timeout Max, Timeout handled directly in SetRx function
                RxTimeout = 0xFFFF;

                break;
        }
}

void radio_set_tx_config( RadioModems_t modem, int8_t power, uint32_t fdev,
			uint32_t bandwidth, uint32_t datarate,
			uint8_t coderate, uint16_t preambleLen,
			bool fixLen, bool crcOn, bool freqHopOn,
			uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
        switch( modem ) {
        case MODEM_FSK:
                // Modulation parameter
                SX126x.ModulationParams.packet_type= PACKET_TYPE_GFSK;
                SX126x.ModulationParams.params.gfsk.bit_rate = datarate;
                SX126x.ModulationParams.params.gfsk.modulation_shaping = MOD_SHAPING_G_BT_1;
                SX126x.ModulationParams.params.gfsk.bandwidth = RadioGetFskBandwidthRegValue( bandwidth );
                SX126x.ModulationParams.params.gfsk.fdev = fdev;
                // Packet parameter
                SX126x.PacketParams.packet_type= PACKET_TYPE_GFSK;
                SX126x.PacketParams.params.gfsk.preamble_length = ( preambleLen << 3 ); // convert byte into bit
                SX126x.PacketParams.params.gfsk.preamble_min_detect = PREAMBLE_DETECTOR_08_BITS;
                SX126x.PacketParams.params.gfsk.sync_word_length = 3 << 3 ; // convert byte into bit
                SX126x.PacketParams.params.gfsk.addr_comp = ADDR_COMP_FILT_OFF;
                SX126x.PacketParams.params.gfsk.header_type = ( fixLen == true ) ? PACKET_FIXED_LENGTH : PACKET_VARIABLE_LENGTH;

                if( crcOn == true ) {
                        SX126x.PacketParams.params.gfsk.crc_len = CRC_2_BYTES_CCIT;
                } else {
                        SX126x.PacketParams.params.gfsk.crc_len = CRC_OFF;
                }
                SX126x.PacketParams.params.gfsk.dc_free = DC_FREE_WHITENING;

                radio_stanby( );
                radio_set_modem( ( SX126x.ModulationParams.packet_type== PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
                sx126x_set_modulation_params( &SX126x.ModulationParams );
                sx126x_set_packet_params( &SX126x.PacketParams );
                sx126x_set_sync_word( ( uint8_t[] ){ 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 } );
                sx126x_set_whitening_seed( 0x01FF );
                break;

        case MODEM_LORA:
                // Modulation parameter
                SX126x.ModulationParams.packet_type= PACKET_TYPE_LORA;
                SX126x.ModulationParams.params.lora.spreading_factor = ( radio_lora_spreading_factor_t ) datarate;
                SX126x.ModulationParams.params.lora.bandwith =  Bandwidths[bandwidth];
                SX126x.ModulationParams.params.lora.coding_rate= ( radio_lora_coding_rate_t ) coderate;
                if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
                        ( ( bandwidth == 1 ) && ( datarate == 12 ) ) ) {
                        SX126x.ModulationParams.params.lora.low_data_rate_optimize = 0x01;
                } else {
                        SX126x.ModulationParams.params.lora.low_data_rate_optimize = 0x00;
                }

                if ((datarate == 11) || (datarate == 12)) {
                        sx126x_write_register(0x889, sx126x_read_register(0x889) & 0xfB);
                }

                // Packet parameter
                SX126x.PacketParams.packet_type= PACKET_TYPE_LORA;
                if( ( SX126x.ModulationParams.params.lora.spreading_factor == LORA_SF5 ) ||
                        ( SX126x.ModulationParams.params.lora.spreading_factor == LORA_SF6 ) ) {
                        if( preambleLen < 12 ) {
                                SX126x.PacketParams.params.lora.preamble_length = 12;
                        } else {
                                SX126x.PacketParams.params.lora.preamble_length = preambleLen;
                        }
                } else {
                        SX126x.PacketParams.params.lora.preamble_length = preambleLen;
                }

                SX126x.PacketParams.params.lora.header_type = ( radio_lora_packet_length_mode_t ) fixLen;
                SX126x.PacketParams.params.lora.payload_len = MaxPayloadLength;
                SX126x.PacketParams.params.lora.crc_mode = ( radio_lora_crc_mode_t ) crcOn;
                SX126x.PacketParams.params.lora.invert_iq = ( radio_lora_iq_mode_t ) iqInverted;

                radio_stanby( );
                radio_set_modem( ( SX126x.ModulationParams.packet_type== PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
                sx126x_set_modulation_params( &SX126x.ModulationParams );
                sx126x_set_packet_params( &SX126x.PacketParams );
                break;
        }
        sx126x_set_rf_tx_power( power );
        TxTimeout = timeout;
}

bool radio_check_rf_frequency( uint32_t frequency )
{
        return true;
}

uint32_t radio_time_on_air( RadioModems_t modem, uint8_t pktLen )
{
        uint32_t airTime = 0;

        switch( modem ) {
        case MODEM_FSK: {
                airTime = (uint32_t) rint( ( 8 * ( SX126x.PacketParams.params.gfsk.preamble_length +
                        ( SX126x.PacketParams.params.gfsk.sync_word_length >> 3 ) +
                        ( ( SX126x.PacketParams.params.gfsk.header_type == PACKET_FIXED_LENGTH ) ? 0.0 : 1.0 ) +
                        pktLen +
                        ( ( SX126x.PacketParams.params.gfsk.crc_len == CRC_2_BYTES ) ? 2.0 : 0 ) ) /
                        SX126x.ModulationParams.params.gfsk.bit_rate ) * 1e3 );
        }
                break;
        case MODEM_LORA: {
                double ts = RadioLoRaSymbTime[SX126x.ModulationParams.params.lora.bandwith - 4][12 - SX126x.ModulationParams.params.lora.spreading_factor];
                // time of preamble
                double tPreamble = ( SX126x.PacketParams.params.lora.preamble_length + 4.25 ) * ts;
                // Symbol length of payload and time
                double tmp = ceil( ( 8 * pktLen - 4 * SX126x.ModulationParams.params.lora.spreading_factor +
                        28 + 16 * SX126x.PacketParams.params.lora.crc_mode -
                        ( ( SX126x.PacketParams.params.lora.header_type == LORA_PACKET_FIXED_LENGTH ) ? 20 : 0 ) ) /
                        ( double )( 4 * ( SX126x.ModulationParams.params.lora.spreading_factor -
                        ( ( SX126x.ModulationParams.params.lora.low_data_rate_optimize > 0 ) ? 2 : 0 ) ) ) ) *
                        ( ( SX126x.ModulationParams.params.lora.coding_rate % 4 ) + 4 );
                double nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
                double tPayload = nPayload * ts;
                // Time on air
                double tOnAir = tPreamble + tPayload;
                // return milli seconds
                airTime = floor( tOnAir + 0.999 );
        }
                break;
        }
        return airTime;
}

void radio_send( uint8_t *buffer, uint8_t size )
{
        // Aron
        sx126x_tx_en();
        sx126x_clear_irq_status(IRQ_RADIO_ALL);
        sx126x_set_dio_irq_params( IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                IRQ_RADIO_NONE,
                IRQ_RADIO_NONE );

        if( sx126x_get_packet_type( ) == PACKET_TYPE_LORA ) {
                SX126x.PacketParams.params.lora.payload_len = size;
        } else {
                SX126x.PacketParams.params.gfsk.payload_len = size;
        }
        sx126x_set_packet_params( &SX126x.PacketParams );

        // Aron, important
        // sx126x_set_buffer_base_addr(0x00, 0x00);

        sx126x_send_payload(buffer, size, 0);

        // Aron
        timer_set(&tx_timeout_timer, TxTimeout);
}

void radio_sleep( void )
{
        sleep_params_t params = { 0 };

        params.fields.warm_start = 1;
        sx126x_set_sleep( params );

        //mdelay( 2 );
}

void radio_stanby( void )
{
        sx126x_set_standby( STDBY_RC );
}

void radio_start_rx( uint32_t timeout )
{
        // Aron
        sx126x_rx_en();
        sx126x_clear_irq_status(IRQ_RADIO_ALL);
        sx126x_set_dio_irq_params(IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT,
                IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                IRQ_RADIO_NONE,
                IRQ_RADIO_NONE );

        if (timeout != 0) {
                // Aron
                timer_set(&rx_timeout_timer, timeout);
        }

        if( RxContinuous == true ) {
                sx126x_set_rx( 0xFFFFFF ); // Rx Continuous
        } else {
                sx126x_set_rx(RxTimeout << 6);
        }
}

void radio_rx_boosted( uint32_t timeout )
{
        sx126x_set_dio_irq_params( IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                IRQ_RADIO_NONE,
                IRQ_RADIO_NONE );

        if (timeout != 0) {
                // Aron
                timer_set(&rx_timeout_timer, timeout);
        }

        if( RxContinuous == true ) {
                sx126x_set_rx_boosted( 0xFFFFFF ); // Rx Continuous
        } else {
                sx126x_set_rx_boosted( RxTimeout << 6 );
        }
}

void radio_set_rx_duty_cycle( uint32_t rxTime, uint32_t sleepTime )
{
        sx126x_set_rx_duty_cycle( rxTime, sleepTime );
}

void radio_start_cad( void )
{
        sx126x_set_cad( );
}

void radio_start_tx( uint32_t timeout )
{
        sx126x_set_tx( timeout << 6 );
}

void radio_set_tx_continuous_wave( uint32_t freq, int8_t power, uint16_t time )
{
        sx126x_set_rf_frequency( freq );
        sx126x_set_rf_tx_power( power );
        sx126x_set_tx_continuous_wave( );

        // Aron
#if 0
        TimerSetValue( &RxTimeoutTimer, time  * 1e3 );
        TimerStart( &RxTimeoutTimer );
#endif
}

int16_t radio_rssi( RadioModems_t modem )
{
        return sx126x_get_rssi_inst( );
}

void radio_write( uint16_t addr, uint8_t data )
{
        sx126x_write_register( addr, data );
}

uint8_t radio_read( uint16_t addr )
{
        return sx126x_read_register( addr );
}

void radio_write_buffer( uint16_t addr, uint8_t *buffer, uint8_t size )
{
        sx126x_write_registers( addr, buffer, size );
}

void radio_read_buffer( uint16_t addr, uint8_t *buffer, uint8_t size )
{
	sx126x_read_registers( addr, buffer, size );
}

void radio_write_fifo( uint8_t *buffer, uint8_t size )
{
        sx126x_write_buffer( 0, buffer, size );
}

void radio_read_fifo( uint8_t *buffer, uint8_t size )
{
        sx126x_read_buffer( 0, buffer, size );
}

void radio_set_max_payload_length( RadioModems_t modem, uint8_t max )
{
        if( modem == MODEM_LORA ) {
                SX126x.PacketParams.params.lora.payload_len = MaxPayloadLength = max;
                sx126x_set_packet_params( &SX126x.PacketParams );
        } else {
                if( SX126x.PacketParams.params.gfsk.header_type == PACKET_VARIABLE_LENGTH ) {
                        SX126x.PacketParams.params.gfsk.payload_len = MaxPayloadLength = max;
                        sx126x_set_packet_params( &SX126x.PacketParams );
                }
        }
}

void radio_set_public_network( bool enable )
{
        RadioPublicNetwork.Current = RadioPublicNetwork.Previous = enable;

        radio_set_modem( MODEM_LORA );

        if( enable == true ) {
                // Change LoRa modem SyncWord
                sx126x_write_register( REG_LR_SYNCWORD, ( LORA_MAC_PUBLIC_SYNCWORD >> 8 ) & 0xFF );
                sx126x_write_register( REG_LR_SYNCWORD + 1, LORA_MAC_PUBLIC_SYNCWORD & 0xFF );
        } else {
                // Change LoRa modem SyncWord
                sx126x_write_register( REG_LR_SYNCWORD, ( LORA_MAC_PRIVATE_SYNCWORD >> 8 ) & 0xFF );
                sx126x_write_register( REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF );
        }
}

uint32_t radio_get_wakeup_time( void )
{
        return (uint32_t) sx126x_get_board_txco_wakeup_time( ) + RADIO_WAKEUP_TIME;
}

void RadioOnTxTimeoutIrq( void* context )
{
	if( ( radio_events != NULL ) && ( radio_events->tx_timeout != NULL ) ) {
		radio_events->tx_timeout( );
	}
}

void RadioOnRxTimeoutIrq( void* context )
{
	if( ( radio_events != NULL ) && ( radio_events->rx_timeout != NULL ) ) {
		radio_events->rx_timeout( );
	}
}

void radio_on_dio_irq( void* context )
{
        IrqFired = true;

        // Aron, st_lorawan version add.
        //radio_irq_process();
        radio_irq_cb();
}

void radio_irq_process( void )
{
        if( IrqFired == true ) {
                uint16_t irq_flag;
                radio_err_t err;

                BACKUP_PRIMASK( );
                __disable_irq( );
                // Clear IRQ flag
                IrqFired = false;
                RESTORE_PRIMASK( );

                irq_flag = sx126x_get_irq_status( );
                sx126x_clear_irq_status( IRQ_RADIO_ALL );
                err = sx126x_get_device_errors();
                printf("irq_flag = 0x%04lx, err = 0x%04lx\r\n", (uint32_t) irq_flag, (uint32_t) err.value);

                if( ( irq_flag & IRQ_TX_DONE ) == IRQ_TX_DONE ) {
                        // Aron, TimerStop
                        timer_stop(&tx_timeout_timer);
                        //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
                        sx126x_set_operating_mode( MODE_STDBY_RC );
                        if( ( radio_events != NULL ) && ( radio_events->tx_done != NULL ) ) {
                                radio_events->tx_done( );
                        }
                }

                if((irq_flag & (IRQ_RX_DONE | IRQ_CRC_ERROR)) == IRQ_RX_DONE) {
                        uint8_t size;
                        // Aron, TimerStop
                        timer_stop(&rx_timeout_timer);
                        if(RxContinuous == false) {
                                //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
                                sx126x_set_operating_mode(MODE_STDBY_RC);
                        }
                        sx126x_get_payload(radio_rx_payload, &size , 255);
                        sx126x_get_packet_status( &radio_pkt_status );
                        if((radio_events != NULL) && (radio_events->rx_done != NULL)) {
                                radio_events->rx_done(radio_rx_payload, size, radio_pkt_status.params.lora.rssi_pkt, radio_pkt_status.params.lora.snr_pkt);
                        }
                }

                if( ( irq_flag & IRQ_CRC_ERROR ) == IRQ_CRC_ERROR ) {
                        if( RxContinuous == false ) {
                                //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
                                sx126x_set_operating_mode( MODE_STDBY_RC );
                        }
                        if( ( radio_events != NULL ) && ( radio_events->rx_error ) ) {
                                radio_events->rx_error( );
                        }
                }

                if( ( irq_flag & IRQ_CAD_DONE ) == IRQ_CAD_DONE ) {
                        //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
                        sx126x_set_operating_mode( MODE_STDBY_RC );
                        if( ( radio_events != NULL ) && ( radio_events->CadDone != NULL ) ) {
                                radio_events->CadDone( ( ( irq_flag & IRQ_CAD_ACTIVITY_DETECTED ) == IRQ_CAD_ACTIVITY_DETECTED ) );
                        }
                }

                if( ( irq_flag & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT ) {
                        if( sx126x_get_operating_mode( ) == MODE_TX ) {
                                // Aron, TimerStop
                                timer_stop(&tx_timeout_timer);
                                //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
                                sx126x_set_operating_mode( MODE_STDBY_RC );
                                if( ( radio_events != NULL ) && ( radio_events->tx_timeout != NULL ) ) {
                                        radio_events->tx_timeout( );
                                }
                        } else if( sx126x_get_operating_mode( ) == MODE_RX ) {
                                // Aron, TimerStop
                                timer_stop(&rx_timeout_timer);
                                //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
                                sx126x_set_operating_mode( MODE_STDBY_RC );
                                if( ( radio_events != NULL ) && ( radio_events->rx_timeout != NULL ) ) {
                                        radio_events->rx_timeout( );
                                }
                        }
                }

                if( ( irq_flag & IRQ_PREAMBLE_DETECTED ) == IRQ_PREAMBLE_DETECTED ) {
                        //__NOP( );
                        // printf("PHY PREAMBLE OK\r\n");
                }

                if( ( irq_flag & IRQ_SYNCWORD_VALID ) == IRQ_SYNCWORD_VALID ) {
                        //__NOP( );
                        // printf("PHY SYNCWORD OK\r\n");
                }

                if( ( irq_flag & IRQ_HEADER_VALID ) == IRQ_HEADER_VALID ) {
                        //__NOP( );
                        // printf("PHY HEADER OK\r\n");
                }

                if( ( irq_flag & IRQ_HEADER_ERROR ) == IRQ_HEADER_ERROR ) {
                        // Aron, TimerStop
                        timer_stop(&rx_timeout_timer);
                        if( RxContinuous == false ) {
                                //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
                                sx126x_set_operating_mode( MODE_STDBY_RC );
                        }
                        if( ( radio_events != NULL ) && ( radio_events->rx_timeout != NULL ) ) {
                                radio_events->rx_timeout( );
                                // printf("PHY HEADER ERROR\r\n");
                        }
                }
        }
}

