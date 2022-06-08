#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ------ Public data type declarations ----------------------------

// User-defined type to store required data for each task
typedef struct {
	// Pointer to the task (must be a 'void (void)' function)
	void ( *pTask ) ( void );

	// Delay (ticks) until the task will (next) be run
	// - see SCH_Add_Task() for further details
	uint32_t Delay;

	// Interval (ticks) between subsequent runs.
	// - see SCH_Add_Task() for further details
	uint32_t Period;
} sTask_t;

// ------ Public function prototypes -----------------------------------------

void sch_init_hz( const uint32_t TICKhz );
void sch_start( void );
int sch_dispatch_tasks( void );
void sch_tick_handler( void );

int sch_add_task( void ( * pTask )(),
                   const uint32_t DELAY,    // Offset (Ticks)
                   const uint32_t PERIOD ); // Period (Ticks)

// ------ Public constants -------------------------------------------------------------

// The maximum number of tasks required at any one time
// during the execution of the program
//
// MUST BE CHECKED FOR EACH PROJECT (*not* dynamic)
#define SCH_MAX_TASKS ( 5 )

// Usually set to 1, unless 'Long Tasks' are employed
#define SCH_TICK_COUNT_LIMIT ( 20 )

// Default value for pTask (no task at this location)
#define SCH_NULL_PTR ( ( void (*) ( void ) ) 0 )

#ifdef __cplusplus
}
#endif