void controlMixer()
{
  /* DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
   *
   * Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and appropriately mixes them for the desired
   * vehicle configuration. For example on a quadcopter, the left two motors should have +roll_PID while the right two motors
   * should have -roll_PID. Front two should have -pitch_PID and the back two should have +pitch_PID etc... every motor has
   * normalized (0 to 1) thro_des command for throttle control. Can also apply direct unstabilized commands from the transmitter with
   * roll_passthru, pitch_passthru, and yaw_passthu. mX_command_scaled and sX_command scaled variables are used in scaleCommands()
   * in preparation to be sent to the motor ESCs and servos.
   */
  // Quad mixing
  // m1 = front left, m2 = front right, m3 = back right, m4 = back left
  m1_command_scaled = thro_des - pitch_PID + roll_PID + yaw_PID;
  m2_command_scaled = thro_des - pitch_PID - roll_PID - yaw_PID;
  m3_command_scaled = thro_des + pitch_PID - roll_PID + yaw_PID;
  m4_command_scaled = thro_des + pitch_PID + roll_PID - yaw_PID;
  m5_command_scaled = 0;
  m6_command_scaled = 0;

  // 0.5 is centered servo, 0 is zero throttle if connecting to ESC for conventional PWM, 1 is max throttle
  s1_command_scaled = 0;
  s2_command_scaled = 0;
  s3_command_scaled = 0;
  s4_command_scaled = 0;
  s5_command_scaled = 0;
  s6_command_scaled = 0;
  s7_command_scaled = 0;

  // Example use of the linear fader for float type variables. Linearly interpolate between minimum and maximum values for Kp_pitch_rate variable based on state of channel 6:
#if 0
  if ( channel_6_pwm > 1500 ) { //go to max specified value in 5.5 seconds
    Kp_pitch_rate = floatFaderLinear( Kp_pitch_rate, 0.1, 0.3, 5.5, 1, 2000 ); //parameter, minimum value, maximum value, fadeTime (seconds), state (0 min or 1 max), loop frequency
  }
  if ( channel_6_pwm < 1500 ) { //go to min specified value in 2.5 seconds
    Kp_pitch_rate = floatFaderLinear( Kp_pitch_rate, 0.1, 0.3, 2.5, 0, 2000 ); //parameter, minimum value, maximum value, fadeTime, state (0 min or 1 max), loop frequency
  }
#endif
}