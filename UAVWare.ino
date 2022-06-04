#include "src/UAVWare/uavw.h"


void setup()
{
  // Run scheduler task loop forever
  UAVW_init();

#if 0
  RC_TASK_init();
  SYS_STATS_TASK_init();
  HEARTBEAT_TASK_init();

  // Add tasks to schedule.
  // Parameters are:
  // A. Task name
  // B. Initial delay / offset (in Ticks)
  // C. Task period (in Ticks): Must be > 0
  //            A                        B  C
  sch_add_task( RC_TASK_update,          0, 1 );    // Update RC command
  sch_add_task( SYS_STATS_TASK_update,   0, 100 );  // Update MAVLink system stats
  sch_add_task( HEARTBEAT_TASK_update,   0, 1000 ); // MAVLink heartbeat update

// Start the scheduler
  sch_start();

  while ( 1 ) {
    sch_dispatch_tasks();
  }
#endif

}


const uint32_t hb_interval = 1000; //heartbeat interval in milliseconds - 1 second
uint32_t t_last_hb = 0;
const uint32_t sys_stat_interval = 100; //10 system status messages per second
uint32_t t_last_sys_stat = 0;

void loop ()
{
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
}
