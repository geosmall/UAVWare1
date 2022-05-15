#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void UVOS_LED_on( void );
void UVOS_LED_off( void );
void UVOS_LED_toggle( void );
void UVOS_LED_flash( uint32_t period, uint8_t duty ); // duty: 0 .. 15
void UVOS_LED_pwm( uint8_t pwmval ); // pwmval: 0 .. 15

#ifdef __cplusplus
}
#endif
