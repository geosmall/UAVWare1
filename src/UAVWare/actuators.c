#include "actuators.h"
#include "src/uvos/uvos_act.h"


/* Static variables */


/* Actuator init function */
void actuator_init( void )
{
	UVOS_ACT_init();
}

/* Actuator enable */
void actuator_enable_outputs( void )
{
	UVOS_ACT_enable();
}

void actuator_update_outputs( void )
{
	UVOS_ACT_update_outputs();
}
