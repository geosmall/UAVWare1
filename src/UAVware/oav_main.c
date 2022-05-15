#include <stdbool.h>
#include <stdint.h>

#include "oav_defines.h"
#include "mpu.h"
#include "tasks.h"
#include "debug.h"
#include "storage.h"


void failloop( int val );


void _putchar( char c )
{
	UVOS_TELEM_putc( c );
}

void OAV_main( void )
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

	const uint32_t hb_interval = 1000; //heartbeat interval in milliseconds - 1 second
	uint32_t t_last_hb = 0;
	const uint32_t sys_stat_interval = 100; //10 system status messages per second
	uint32_t t_last_sys_stat = 0;

	while ( 1 ) {
		// rc_update();
		telem_update();
		if ( ( millis() - t_last_hb ) > hb_interval ) {
			telem_heartbeat();
			t_last_hb = millis();
		}
		if ( ( millis() - t_last_sys_stat ) > sys_stat_interval ) {
			telem_system_stats();
			telem_send_queued_params();
			t_last_sys_stat = millis();
		}
		// UVOS_TIME_delay_us( 1000 );
		// delay( 1 );
	}

	RC_TASK_init();
	SYS_STATS_TASK_init();
	HEARTBEAT_TASK_init();

// Add tasks to schedule.
// Parameters are:
// A. Task name
// B. Initial delay / offset (in Ticks)
// C. Task period (in Ticks): Must be > 0
//            A                        B  C
	sch_add_task( RC_TASK_update,          0, 1 ); 		// Update RC command
	sch_add_task( SYS_STATS_TASK_update,   0, 100 ); 	// Update MAVLink system stats
	sch_add_task( HEARTBEAT_TASK_update,   0, 1000 ); // MAVLink heartbeat update

// Start the scheduler
	sch_start();

	while ( 1 ) {
		sch_dispatch_tasks();
	}

	return 1;

}

#if 0 // GLS

void usermain()
{
	ledoff();

	time_init();
	pwm_init(); // For legacy reasons it's called pwm even in the Dshot driver.
	sixaxis_init();
	adc_init(); // DMA takes about 1 us once installed. Must be done early, adc is used in battery_init().
	flash_calculate_pid_c_identifier(); // Must be called before flash_load().
	flash_load(); // Must be called before rx_init() for autobind to work.
	rx_init();
	battery_init(); // Must be called before gyro_cal() to send initial battery voltage there.
	gyro_cal_check_flash();
	// imu_init(); Not really necessary since the gravity vector in brought in sync with accel values in imu() all the time.
	blackbox_init();
#ifdef OSD_ENABLE
	osd_init();
#endif // OSD_ENABLE

#ifdef AUTO_BOOTLOADER
	if ( vbattfilt < 1.0f ) {
		jump_to_bootloader();
	}
#endif // AUTO_BOOTLOADER

	lastlooptime = gettime() - LOOPTIME;
	while ( true ) { // main loop
		// Time measurements with ARMCLANG -O3:
		// sixaxis_read(): 11 +2 per gyro filter (contains 10 SPI bitbang time)
		// control(): 23 acro (+3 angle)
		// checkrx(): 17 us worst case for LOOPTIME < 1000; 39 us otherwise [+1 in case of nrf24 scrambling]

		const uint32_t loop_start_time = gettime();
		looptime = ( loop_start_time - lastlooptime ) * 1e-6f;
		lastlooptime = loop_start_time;
		++loops_since_last_packet;

		blackbox_log(); // First thing in the loop to get equal spacing between the calls.

		if ( loops_since_last_packet == 2 ) { // In this loops_since_last_packet we have enough time to waste, since telemetry
			// gets transmitted and therefore sending motor values is omitted. This reads the accel data only once every 5 ms.
			sixaxis_read(); // read gyro (and accelerometer data for blackbox logging)
		} else {
			gyro_read(); // read just gyro data
		}
		// Gyro filtering is done in sixaxis.c, i.e., at the end of gyro_read() and sixaxis_read().

#ifdef LEVELMODE
		imu(); // attitude calculations for level mode
#endif // LEVELMODE

		telemetry_transmitted = false;
		packet_received = checkrx(); // receiver function (This sets telemetry_transmitted = true in case telemetry was transmitted)
		if ( packet_received ) {
			loops_since_last_packet = 0;
		}

		const bool send_motor_values = ! telemetry_transmitted; // Skip to not interfere with sending telemetry.
		control( send_motor_values ); // all flight calculations, pid and motors

		battery();

		if ( onground ) {
			gestures(); // check gestures
		}

		process_led_command();

#ifdef OSD_ENABLE
		osd();
#endif // OSD_ENABLE

		if ( ! onground ) {
			fly_time += LOOPTIME;
		}

		// max_used_loop_time (for debug)
		used_loop_time = gettime() - loop_start_time;
		if ( used_loop_time > max_used_loop_time ) {
			max_used_loop_time = used_loop_time;
		}

		// avg_used_loop_time (for debug)
		static uint32_t cumulated_used_loop_time, cumulated_used_loop_time_counter;
		cumulated_used_loop_time += used_loop_time;
		++cumulated_used_loop_time_counter;
		if ( cumulated_used_loop_time_counter > 1024 ) { // 0.256 second at 4k loop frequency
			cumulated_used_loop_time /= 1024;
			if ( cumulated_used_loop_time > avg_used_loop_time ) {
				avg_used_loop_time = cumulated_used_loop_time;
			}
			cumulated_used_loop_time = 0;
			cumulated_used_loop_time_counter = 0;
		}

//#define LOOP_TIME_STATS
#ifdef LOOP_TIME_STATS
		// determines most_frequently_used_loop_time (for debug)
		static uint32_t loop_count;
		static uint32_t loop_time_stats[ 256 ];
		if ( used_loop_time > 255 ) {
			used_loop_time = 255;
		}
		++loop_time_stats[ used_loop_time ];
		++loop_count;
		if ( loop_count == 100 ) {
			loop_count = 0;
			uint32_t max_value = 0;
			for ( int i = 0; i < 256; ++i ) {
				if ( loop_time_stats[ i ] > max_value ) {
					max_value = loop_time_stats[ i ];
					most_frequently_used_loop_time = i;
				}
				loop_time_stats[ i ] = 0;
			}
		}
#endif // LOOP_TIME_STATS

		static uint32_t next_loop_start = 0;
		if ( next_loop_start == 0 ) {
			next_loop_start = loop_start_time;
		}
		static float correction = 0;
		correction += LOOPTIME / ( float )WALLTIME_CORRECTION_FACTOR;
		next_loop_start += ( uint32_t )correction;
		correction -= ( uint32_t )correction;
		while ( gettime() < next_loop_start );
	}
}

#endif // 0; GLS

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
