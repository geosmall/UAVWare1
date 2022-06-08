#include "time.h"
#include "../uvos/uvos_time.h"

/* Main Functions */
void time_init( void )
{
	UVOS_TIME_init();
}

void time_delay_us( uint32_t uSec )
{
	UVOS_TIME_delay_us( uSec );
}