#include "uvos.h"
#include "uvos_usart.h"
#include "uvos_telem.h"
#include "hardware.h"
#include "../libraries/lwrb/lwrb.h"

telem_callback_t telem_rxne_callback = NULL;

lwrb_t * pTx_rb;
uint8_t * pTx_rb_data;
volatile size_t usart_tx_dma_current_len = 0;

volatile int usart_tx_dma_transfer_complete = 1;

int UVOS_TELEM_get_usart_tx_dma_transfer_complete( void )
{
  return usart_tx_dma_transfer_complete;
}

void UVOS_TELEM_RegisterTelemUSARTCallback( telem_callback_t telem_callback )
{
  telem_rxne_callback = telem_callback;
}

#if defined ( TELEM_USART_IRQHandler )
void TELEM_USART_IRQHandler( void )
{

  // LL_GPIO_SetOutputPin( testPin1_GPIO_Port, testPin1_Pin );

  /* Check for RXNE interrupt */
  if ( LL_USART_IsEnabledIT_RXNE( TELEM_USART ) && LL_USART_IsActiveFlag_RXNE( TELEM_USART ) ) {
    LL_USART_ClearFlag_RXNE( TELEM_USART ); /* Clear RXNE line flag */
    /* Read Received character. RXNE flag is cleared by reading of DR register */
    uint8_t c = LL_USART_ReceiveData8( TELEM_USART );
    if ( telem_rxne_callback != NULL ) {
      telem_rxne_callback( c );
    }
  }

  // LL_GPIO_ResetOutputPin( testPin1_GPIO_Port, testPin1_Pin );

}
#endif // defined ( RX_USART_IRQHandler )

#if defined ( TELEM_DMA_STREAM_IRQHandler )
void TELEM_DMA_STREAM_IRQHandler( void )
{
  /* Check transfer-complete interrupt */
  if ( LL_DMA_IsEnabledIT_TC( TELEM_DMAx, TELEM_DMA_STREAMy ) && TELEM_DMA_IsActiveFlag_TCx( TELEM_DMAx ) ) {
    TELEM_DMA_ClearFlag_TCx( TELEM_DMAx );         /* Clear transfer complete flag */

    // debug_putc( ( uint8_t ) usart_tx_dma_current_len );
    // debug_put_hex32( usart_tx_dma_current_len );

    lwrb_skip( pTx_rb, usart_tx_dma_current_len ); /* Data sent, ignore these */
    usart_tx_dma_current_len = 0;
    UVOS_TELEM_start_tx_dma_transfer();            /* Try to send more data */
  }
}
#endif // defined ( TELEM_DMA_STREAM_IRQHandler )

void UVOS_TELEM_init( lwrb_t * pTelem_tx_ring_buf, uint8_t * pData )
{
  pTx_rb = pTelem_tx_ring_buf;
  pTx_rb_data = pData;

  /* Aim peripheral DMA at telem USART */
  LL_DMA_SetPeriphAddress( TELEM_DMAx, TELEM_DMA_STREAMy, LL_USART_DMA_GetRegAddr( TELEM_USART ) );

  /* Enable DMA TC interrupt */
  LL_DMA_EnableIT_TC( TELEM_DMAx, TELEM_DMA_STREAMy );

  /* Enable USART DMA Mode for transmission */
  LL_USART_EnableDMAReq_TX( TELEM_USART );

  /* Enable USART global interrupt */
  UVOS_USART_enable_it( TELEM_USART, TELEM_USART_IRQn );

}

void UVOS_TELEM_enable_rxne_it( void )
{
  // Telem USART enable RXNE interrupt
  UVOS_USART_enable_RXNE_it( TELEM_USART, TELEM_USART_IRQn );
}

void UVOS_TELEM_putc( uint8_t c )
{
  UVOS_USART_putc( TELEM_USART, c );
}

void UVOS_TELEM_puts( uint8_t * pData, uint16_t len )
{
  if ( lwrb_get_free( pTx_rb ) >= len ) {
    lwrb_write( pTx_rb, pData, len ); /* Write data to transmit buffer */
    UVOS_TELEM_start_tx_dma_transfer();
  }
}

uint8_t UVOS_TELEM_start_tx_dma_transfer( void )
{
  uint8_t started = 0;
  /* First check if transfer is currently inactive by checking usart_tx_dma_current_len variable.
   * This variable is set before DMA transfer is started and cleared in DMA TX complete interrupt.
   * It is not necessary to disable the interrupts before checking the variable:
   * Disabling interrupts before checking for next transfer is advised only if
   *   - multiple operating system threads can access to this function w/o
   *        exclusive access protection (mutex) configured,
   *   - application calls this function from multiple interrupts. */
  if ( usart_tx_dma_current_len == 0
       && ( usart_tx_dma_current_len = lwrb_get_linear_block_read_length( pTx_rb ) ) > 0 ) {
    /* Limit maximal size to transmit at a time */
    if ( usart_tx_dma_current_len > DMA_XFER_SIZE_MAX ) {
      usart_tx_dma_current_len = DMA_XFER_SIZE_MAX;
    }
    /* Configure DMA */
    LL_DMA_SetDataLength( TELEM_DMAx, TELEM_DMA_STREAMy, usart_tx_dma_current_len );
    LL_DMA_SetMemoryAddress( TELEM_DMAx, TELEM_DMA_STREAMy, ( uint32_t )lwrb_get_linear_block_read_address( pTx_rb ) );
    /* Clear all flags */
    TELEM_DMA_ClearFlag_TCx( TELEM_DMAx );
    /* Start transfer */
    LL_DMA_EnableStream( TELEM_DMAx, TELEM_DMA_STREAMy );
    started = 1;
  }
  return started;
}
