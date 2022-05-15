#include "uvos.h"
#include "uvos_usart.h"
#include "uvos_rx.h"
#include "hardware.h"
#include "stm32yyxx_ll_dma.h"

rx_callback_t rx_idle_detected_callback = NULL;
dma_callback_t rx_dma_transfer_cplt_callback = NULL;


void UVOS_USART_RegisterRxUSARTCallback( rx_callback_t rx_callback )
{
  rx_idle_detected_callback = rx_callback;
}

#if defined ( RX_USART_IRQHandler )
void RX_USART_IRQHandler( void )
{

  // LL_GPIO_SetOutputPin( testPin1_GPIO_Port, testPin1_Pin );

  /* Check for IDLE line interrupt */
  if ( LL_USART_IsEnabledIT_IDLE( RX_USART ) && LL_USART_IsActiveFlag_IDLE( RX_USART ) ) {
    LL_USART_ClearFlag_IDLE( RX_USART ); /* Clear IDLE line flag */
    /* Read Received character. RXNE flag is cleared by reading of DR register */
    uint8_t c = LL_USART_ReceiveData8( RX_USART );
    if ( rx_idle_detected_callback != NULL ) {
      rx_idle_detected_callback( c );
    }
  }

  // LL_GPIO_ResetOutputPin( testPin1_GPIO_Port, testPin1_Pin );

}
#endif // defined ( RX_USART_IRQHandler )

void UVOS_USART_RegisterRxDMACallback( dma_callback_t dma_callback )
{
  rx_dma_transfer_cplt_callback = dma_callback;
}

#if defined ( RX_DMA_STREAM_IRQHandler )
void RX_DMA_STREAM_IRQHandler( void )
{

  // LL_GPIO_SetOutputPin( testPin2_GPIO_Port, testPin2_Pin );

  /* Check for DMA Transfer Complete interrupt */
  // if ( LL_DMA_IsEnabledIT_TC( RX_DMAx, RX_DMA_STREAMy ) && ( RX_DMA_IsActiveFlag_TCx( RX_DMAx ) ) {
  if ( RX_DMA_IsActiveFlag_TCx( RX_DMAx ) ) {
    RX_DMA_ClearFlag_TCx( RX_DMAx );
    /* Disable DMA RX Interrupt */
    LL_USART_DisableDMAReq_RX( RX_USART );
    if ( rx_dma_transfer_cplt_callback != NULL ) {
      rx_dma_transfer_cplt_callback();
    }
  }

  // LL_GPIO_ResetOutputPin( testPin2_GPIO_Port, testPin2_Pin );

}
#endif // defined ( RX_DMA_STREAM_IRQHandler )


/*----------------------------------------------------------------------------------------*/

void UVOS_USART_rx_enable_idle_it( void )
{
  // RX USART interrupt Init
  NVIC_SetPriority( RX_USART_IRQn, 0 );
  NVIC_EnableIRQ( RX_USART_IRQn );
  // USART enable IDLE interrupt
  LL_USART_EnableIT_IDLE( RX_USART );
}

// HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void UVOS_USART_rx_config_DMA( uint8_t * ptrBuffer, uint16_t length )
{
  /* LL_DMA_ConfigAddresses(DMA_TypeDef* DMAx, uint32_t Stream,
                            uint32_t SrcAddress,
                            uint32_t DstAddress,
                            uint32_t Direction) */
  LL_DMA_ConfigAddresses( RX_DMAx, RX_DMA_STREAMy,
                          LL_USART_DMA_GetRegAddr( RX_USART ),
                          ( uint32_t )ptrBuffer,
                          LL_DMA_GetDataTransferDirection( RX_DMAx, RX_DMA_STREAMy ) );

  LL_DMA_SetDataLength( RX_DMAx, RX_DMA_STREAMy, length );

  /* (4) Enable DMA transfer complete (TC) interrupt */
  LL_DMA_EnableIT_TC( RX_DMAx, RX_DMA_STREAMy );

}

void UVOS_USART_rx_receive_start_DMA( void )
{
  /* Enable DMA RX Interrupt */
  LL_USART_EnableDMAReq_RX( RX_USART );

  // Enable DMA Channel Rx
  LL_DMA_EnableStream( RX_DMAx, RX_DMA_STREAMy );
}

void UVOS_USART_rx_receive_stop_DMA( void )
{
  // Enable DMA Channel Rx
  LL_DMA_DisableStream( RX_DMAx, RX_DMA_STREAMy );
}
