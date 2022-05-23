#include "ibus.h"
#include "src/uvos/uvos_usart.h"


/* Static variables */
static uint8_t uart_rx_buffer[IBUS_LENGTH] = {0};
static uint8_t ibus_lost_flag = 0;

/**
 * @brief ibus receive transfer complete interrupt callback fn
 */
void ibus_rx_idle_detected_callback( void )
{
	UVOS_USART_rx_receive_start_DMA(); // Enable USART DMAR and Rx DMA Stream
}

/**
 * @brief ibus receive transfer complete interrupt callback fn
 */
void ibus_rx_transfer_complete_callback( void )
{
	UVOS_USART_rx_receive_stop_DMA(); // Disable Rx DMA Stream (USART DMAR disabled in USART DMA int handler)
	ibus_lost_flag = 0;
}

/* Ibus init function */
void ibus_init()
{
	UVOS_USART_rx_enable_idle_it();
	UVOS_USART_RegisterRxUSARTCallback( ibus_rx_idle_detected_callback );

	UVOS_USART_rx_config_DMA( uart_rx_buffer, IBUS_LENGTH );
	UVOS_USART_RegisterRxDMACallback( ibus_rx_transfer_complete_callback );
}

/**
 * @brief Check ibus is valid and parse ibus data from uart dma buffer
 */
bool ibus_read( uint16_t ibus_channel[], uint8_t ch_num )
{
	if ( !ibus_is_valid() )
		return false;

	if ( !ibus_checksum() )
		return false;

	ibus_update( ibus_channel, ch_num );
	return true;
}


/* Sub Functions */
bool ibus_is_valid()
{
	// is it valid ibus?
	return ( ( uart_rx_buffer[0] == IBUS_LENGTH ) && ( uart_rx_buffer[1] == IBUS_COMMAND40 ) );
}

bool ibus_checksum()
{
	uint16_t checksum_cal = 0xffff;
	uint16_t checksum_ibus;

	for ( int i = 0; i < 30; i++ )
		checksum_cal -= uart_rx_buffer[i];

	checksum_ibus = uart_rx_buffer[31] << 8 | uart_rx_buffer[30]; // checksum value from ibus
	return ( checksum_ibus == checksum_cal );
}

void ibus_update( uint16_t ibus_channel[], uint8_t ch_num )
{
	for ( int i = 0, j = 2; i < ch_num; i++, j += 2 ) {
		ibus_channel[i] = uart_rx_buffer[j + 1] << 8 | uart_rx_buffer[j];
	}
}

bool is_ibus_lost()
{
	uint8_t max_lost_flag = 20;

	if ( max_lost_flag > ibus_lost_flag ) {
		ibus_lost_flag++;
		return false;
	}

	else
		return true;
}
