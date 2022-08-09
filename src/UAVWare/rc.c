#include "rc.h"
#include "ibus.h"

#define RC_NUM_CHANNELS 14

rc_command_t my_rc_command;
// angle_t my_target_angle;

/* Static Functions */
static void rc_clear_command( rc_command_t * rc_command );
static void rc_get_raw_command( uint16_t ibus_channel[], rc_raw_command_t * rc_raw_command );
static void rc_get_command( rc_raw_command_t rc_raw_command, rc_command_t * rc_command );
// static void get_target_angle(rc_command_t rc_command, angle_t* target_angle);
static void rc_get_command_usec( uint16_t ibus_channel[], uint16_t rc_command[], size_t size_rc_cmd );

/* Main Functions */
void rc_init()
{
  ibus_init();
}

bool rc_update( uint16_t rc_raw_command_arr[], size_t size_arr )
{
  static uint16_t ibus_channel[ RC_NUM_CHANNELS ] = { 0 };
  // rc_raw_command_t rc_raw_command = { 0 };

  if ( !ibus_read( ibus_channel, RC_NUM_CHANNELS ) )
    return false;

  if ( is_ibus_lost() ) {
    // rc_clear_command( &my_rc_command );
    return false;
  }

  // rc_get_raw_command( ibus_channel, rc_raw_command );
  // rc_get_command( rc_raw_command, &my_rc_command );
  // get_target_angle( my_rc_command, &my_target_angle );
  rc_get_command_usec( ibus_channel, rc_raw_command_arr, size_arr );


  return true;
}

static void rc_get_command_usec( uint16_t ibus_channel[], uint16_t rc_command[], size_t size )
{
  for ( size_t i = 0; i < size; ++i ) {
    rc_command[ i ] = ibus_channel[ i ];
  }
}

/* Static Functions */
static void rc_clear_command( rc_command_t * rc_command )
{
  rc_command->aux = false;
  rc_command->arming = false;

  rc_command->throttle = 0;

  rc_command->roll = 0;
  rc_command->pitch = 0;
  rc_command->yaw = 0;
}

static void rc_get_raw_command( uint16_t ibus_channel[], rc_raw_command_t * rc_raw_command )
{
  rc_raw_command->aux = ibus_channel[ 9 ];
  rc_raw_command->arming = ibus_channel[ 6 ];

  rc_raw_command->throttle = ibus_channel[ 2 ];

  rc_raw_command->roll = ibus_channel[ 0 ];
  rc_raw_command->pitch = ibus_channel[ 1 ];
  rc_raw_command->yaw = ibus_channel[ 3 ];
}

/**
 * @brief set angle command range
 */
static uint8_t rc_set_angle_range( int8_t max_angle, int8_t min_angle )
{
  uint8_t angle_range = max_angle - min_angle;
  uint8_t scale_factor = 1000 / angle_range;

  return scale_factor;
}

static void rc_get_command( rc_raw_command_t rc_raw_command, rc_command_t * rc_command )
{
  rc_command->aux  = ( rc_raw_command.aux >= 1750 ) ? true : false;
  rc_command->arming = ( rc_raw_command.arming >= 1750 ) ? true : false;

  rc_command->throttle = rc_raw_command.throttle - 1000;

  uint8_t scale_factor = rc_set_angle_range( 20, -20 ); // send angle command upto ±20 degree
  rc_command->roll  = ( int8_t )( ( rc_raw_command.roll  - 1500 ) / scale_factor );
  rc_command->pitch = ( int8_t )( ( rc_raw_command.pitch - 1500 ) / scale_factor );
  rc_command->yaw   = ( int8_t )( ( rc_raw_command.yaw   - 1500 ) / scale_factor );
}

/**
 * @brief get target angle from rc command
 * @param[in] rc_command
 * @param[out] target_angle
 */
// static void get_target_angle( rc_command_t rc_command, angle_t* target_angle )
// {
//   target_angle->roll  = rc_command.roll;
//   target_angle->pitch = rc_command.pitch;
//   target_angle->yaw   = rc_command.yaw;
// }