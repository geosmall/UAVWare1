#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void telem_init( void );
void telem_heartbeat( void );
void telem_system_stats( void );
void telem_update( void );
void telem_byte_received_callback( uint8_t c );
void telem_send_queued_params( void );

#ifdef __cplusplus
}
#endif