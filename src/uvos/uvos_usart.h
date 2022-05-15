#pragma once

#include "../core/main.h"

#ifdef __cplusplus
extern "C" {
#endif

void UVOS_USART_enable_it( USART_TypeDef * USARTx, IRQn_Type IRQn );
void UVOS_USART_enable_RXNE_it( USART_TypeDef * USARTx, IRQn_Type IRQn );
void UVOS_USART_enable_IDLE_it( USART_TypeDef * USARTx, IRQn_Type IRQn );
int UVOS_USART_putc( USART_TypeDef * USARTx, char const c );
int UVOS_USART_getc( USART_TypeDef * USARTx );

#ifdef __cplusplus
}
#endif