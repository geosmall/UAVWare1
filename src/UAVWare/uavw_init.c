#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "uavw_defines.h"
#include "mpu.h"
#include "tasks.h"
#include "debug.h"


// used by printf.c library
void putchar_( char c )
{
	UVOS_TELEM_putc( c );
}

int UAVW_init( void )
{
	UVOS_PERIPH_init();

	debug_init();

	time_init(); // must be called before mpu_init()

	rc_init();

	if ( FS_Init() != 0 ) {
		return EXIT_FAILURE;
	}

#if defined (SKIP_MPU_CHECK)
	mpu_init();
#else
	if ( !mpu_init() ) {
		return EXIT_FAILURE;
	}
#endif
	mpu_set_fullscale_gyro_range( GYRO_FS_2000DPS );
	mpu_set_fullscale_accel_range( ACCEL_FS_4G );

	telem_init();

	actuator_init();

	return EXIT_SUCCESS;
}

void HardFault_Handler( void )
{
	failloop( 5 );
}

void MemManage_Handler( void )
{
	failloop( 5 );
}

void BusFault_Handler( void )
{
	failloop( 5 );
}

void UsageFault_Handler( void )
{
	failloop( 5 );
}
