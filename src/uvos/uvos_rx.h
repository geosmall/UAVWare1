#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "../core/main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void ( *rx_callback_t ) ( uint8_t c );
typedef void ( *dma_callback_t ) ( void );

void UVOS_USART_rx_enable_idle_it( void );
void UVOS_USART_rx_config_DMA( uint8_t *ptrBuffer, uint16_t length );
void UVOS_USART_rx_receive_start_DMA( void );
void UVOS_USART_rx_receive_stop_DMA( void );
void UVOS_USART_RegisterRxUSARTCallback( rx_callback_t rx_callback );
void UVOS_USART_RegisterRxDMACallback( dma_callback_t dma_callback );

#ifdef __cplusplus
}
#endif