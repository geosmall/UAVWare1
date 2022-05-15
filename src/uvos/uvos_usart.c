#include "uvos.h"
#include "uvos_usart.h"


void UVOS_USART_enable_it( USART_TypeDef * USARTx, IRQn_Type IRQn )
{
  // RX USART interrupt Init
  NVIC_SetPriority( IRQn, 0 );
  NVIC_EnableIRQ( IRQn );
}

void UVOS_USART_enable_RXNE_it( USART_TypeDef * USARTx, IRQn_Type IRQn )
{
  // RX USART interrupt Init
  NVIC_SetPriority( IRQn, 0 );
  NVIC_EnableIRQ( IRQn );
  // USART enable RXNE interrupt
  LL_USART_EnableIT_RXNE( USARTx );
}

void UVOS_USART_enable_IDLE_it( USART_TypeDef * USARTx, IRQn_Type IRQn )
{
  // RX USART interrupt Init
  NVIC_SetPriority( IRQn, 0 );
  NVIC_EnableIRQ( IRQn );
  // USART enable IDLE interrupt
  LL_USART_EnableIT_IDLE( USARTx );
}

int UVOS_USART_putc( USART_TypeDef * USARTx, char const c )
{
  while ( !LL_USART_IsActiveFlag_TXE( USARTx ) );
  LL_USART_TransmitData8( USARTx, ( uint8_t )c );
  return 1;
}

void UVOS_USART_puts( USART_TypeDef * USARTx, uint8_t * pData, uint16_t len )
{
  uint8_t c = *pData++;
  while ( len > 0U ) {
    UVOS_USART_putc( USARTx, c );
    c = *pData++;
    len--;
  }
}

int UVOS_USART_getc( USART_TypeDef * USARTx )
{
  return LL_USART_ReceiveData8( USARTx );
}
