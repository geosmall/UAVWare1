#include "src/UAVware/uavw_init.h"
#include "src/UAVware/telem.h"

void setup()
{
  // Run scheduler task loop forever
  UAVW_init();

  // while ( 1 ) {
  //   UVOS_TIME_delay_us( 1000 );
  //   delay( 1 );
  // }

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
    // UVOS_TIME_delay_us( 1000 );
    // delay( 1 );
}
