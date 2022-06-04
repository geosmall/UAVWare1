#include "led.h"
#include "../uvos/uvos_led.h"

/* Main Functions */
void led_status_on( void )
{
  UVOS_LED_on();
}

void led_status_off( void )
{
  UVOS_LED_off();
}

void led_status_toggle( void )
{
  UVOS_LED_toggle();
}
