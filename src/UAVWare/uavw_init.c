#include <stdbool.h>
#include <stdint.h>

#include "uavw_defines.h"
#include "mpu.h"
#include "tasks.h"
#include "debug.h"


void failloop( int val );

// used by printf.c library
void _putchar( char c )
{
	UVOS_TELEM_putc( c );
}

void UAVW_init( void )
{
	UVOS_PERIPH_init();

	debug_init();

	storage_set_defaults();

	if ( FS_Init() != 0 ) {
		failloop( 4 );
	}

	sch_init_hz( OAV_LOOP_FREQ_HZ ); // must come before mpu init()

#define SKIP_MPU_CHECK
#if defined (SKIP_MPU_CHECK)
	mpu_init();
#else
	if ( !mpu_init() ) {
		failloop( 4 );
	}
#endif

	mpu_set_fullscale_gyro_range( GYRO_FS_2000DPS );
	mpu_set_fullscale_accel_range( ACCEL_FS_4G );

	telem_init();

  actuator_init();
  actuator_enable_outputs();
  actuator_update_outputs();

	return 1;
}

// 2 - low battery at powerup - if enabled by config
// 3 - radio chip not detected
// 4 - Gyro not found
// 5 - clock, interrupts, systick, bad code
// 6 - flash write error
// 7 - ESC pins on more than two distinct GPIO ports
void failloop( int val )
{
	// for ( int i = 0; i <= 3; ++i ) {  <<<-- GLS: TODO as this disables 4 motors
	// 	pwm_set( i, 0 );
	// }
	while ( true ) {
		for ( int i = 0; i < val; ++i ) {
			UVOS_LED_on();
			UVOS_TIME_delay_us( 200000 );
			UVOS_LED_off();
			UVOS_TIME_delay_us( 200000 );
		}
		UVOS_TIME_delay_us( 800000 );
	}
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
