#include "uvos.h"
#include "uvos_usart.h"
#include "hardware.h"

int UVOS_USART_debug_putc( char const c )
{
  return UVOS_USART_putc ( DEBUG_USART, c );
}
