#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  // bool start;     // if start is true, start init functions and control loop.
  bool arming;    // start the motors.
  bool aux;

  uint16_t throttle;

  int16_t roll;   // target angle (unit : degree)
  int16_t pitch;
  int16_t yaw;
} rc_command_t;

typedef struct {
  // struct for mapping ibus channel to rc command.
  // have same data type(uint16_t) as ibus data.
  // uint16_t start;
  uint16_t arming;
  uint16_t aux;

  uint16_t throttle;

  uint16_t roll;
  uint16_t pitch;
  uint16_t yaw;
} rc_raw_command_t;


// extern rc_command_t my_rc_command;
// extern angle_t my_target_angle;


/* Main Functions */
void rc_init();
bool rc_update( uint16_t rc_raw_command_arr[], size_t size_arr );

#ifdef __cplusplus
}
#endif