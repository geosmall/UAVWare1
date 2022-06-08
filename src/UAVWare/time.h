#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Main Functions */
void time_init( void );
void time_delay_us( uint32_t uSec );

#ifdef __cplusplus
}
#endif