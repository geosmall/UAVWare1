#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "../libraries/lwrb/lwrb.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DMA_XFER_SIZE_MAX 64

typedef void ( *telem_callback_t ) ( uint8_t c );

void UVOS_TELEM_enable_rxne_it( void );
void UVOS_TELEM_init( lwrb_t * telem_tx_ring_buff, uint8_t * pData );
void UVOS_TELEM_putc(uint8_t c);
void UVOS_TELEM_puts( uint8_t * pData, uint16_t len );
uint8_t UVOS_TELEM_start_tx_dma_transfer( void );
int UVOS_TELEM_get_usart_tx_dma_busy_flag( void );

#ifdef __cplusplus
}
#endif