#pragma once

#include <stdint.h>
#include "../core/main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void ( *time_tick_callback_t ) ( void );

void UVOS_TIME_init( void );
void UVOS_TIME_sched_init( const uint32_t tick_hz );
uint32_t UVOS_TIME_sched_start( void );
uint32_t UVOS_TIME_sched_stop( void );
uint32_t UVOS_TIME_getCurrentMillis(void);
uint32_t UVOS_TIME_getCurrentMicros(void);
uint32_t UVOS_TIME_gettime_us( void );
uint32_t UVOS_TIME_gettime_ms( void );
void UVOS_TIME_delay_us( uint32_t us );

#ifdef __cplusplus
}
#endif