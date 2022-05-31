#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Main Functions */
void actuator_init( void );
void actuator_enable_outputs( void );
void actuator_update_outputs( void );

#ifdef __cplusplus
}
#endif