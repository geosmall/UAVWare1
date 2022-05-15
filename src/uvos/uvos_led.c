#include "uvos_led.h"
#include "uvos_time.h"
#include "../core/main.h"
#include "stm32f4xx_ll_gpio.h"


void UVOS_LED_on( void )
{
#ifdef LED_INVERT
	LL_GPIO_ResetOutputPin( STATUS_LED_GPIO_Port, STATUS_LED_Pin );
#else
	LL_GPIO_SetOutputPin( STATUS_LED_GPIO_Port, STATUS_LED_Pin );
#endif
}

void UVOS_LED_off( void )
{
#ifdef LED_INVERT
	LL_GPIO_SetOutputPin( STATUS_LED_GPIO_Port, STATUS_LED_Pin );
#else
	LL_GPIO_ResetOutputPin( STATUS_LED_GPIO_Port, STATUS_LED_Pin );
#endif
}


void UVOS_LED_toggle( void )
{
	LL_GPIO_TogglePin( STATUS_LED_GPIO_Port, STATUS_LED_Pin );
}

void UVOS_LED_flash( uint32_t period, uint8_t duty )
{
	if ( uvos_gettime_us() % period > ( period * duty ) >> 4 ) {
		uvos_ledon();
	} else {
		uvos_ledoff();
	}
}

void UVOS_LED_pwm( uint8_t pwmval )
{
	static uint8_t loopcount = 0;
	++loopcount;
	loopcount &= 0xF;
	if ( pwmval > loopcount ) {
		uvos_ledon();
	} else {
		uvos_ledoff();
	}
}

// Delta-Sigma first order modulator.
// void led_pwm2( uint8_t pwmval )
// {
// 	static uint32_t lastledtime;
// 	static float lastledbrightness = 0;
// 	static float ds_integrator = 0;
// 	const uint32_t time = gettime();
// 	const uint32_t ledtime = time - lastledtime;
// 	lastledtime = time;
// 	float desiredbrightness = pwmval / 15.0f;
// 	if ( ds_integrator > 2 ) {
// 		ds_integrator = 2;
// 	}
// 	ds_integrator += ( desiredbrightness - lastledbrightness ) * ledtime / (float)LOOPTIME;
// 	if ( ds_integrator > 0.49f ) {
// 		ledon();
// 		lastledbrightness = 1.0f;
// 	} else {
// 		ledoff();
// 		lastledbrightness = 0;
// 	}
// }
