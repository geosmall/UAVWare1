void controlANGLE()
{
  /* DESCRIPTION: Computes control commands based on state error (angle)
   *
   * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in
   * getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
   * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I
   * terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
   * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().
   */

  // Roll
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll * dt;
  if ( channel_1_pwm < 1060 ) { //don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain( integral_roll, -config.i_limit, config.i_limit ); //saturate integrator to prevent unsafe buildup
  derivative_roll = GyroX;
  roll_PID = 0.01 * ( config.Kp_roll_angle * error_roll + config.Ki_roll_angle * integral_roll - config.Kd_roll_angle * derivative_roll ); //scaled by .01 to bring within -1 to 1 range

  // Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch * dt;
  if ( channel_1_pwm < 1060 ) { //don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain( integral_pitch, -config.i_limit, config.i_limit ); //saturate integrator to prevent unsafe buildup
  derivative_pitch = GyroY;
  pitch_PID = .01 * ( config.Kp_pitch_angle * error_pitch + config.Ki_pitch_angle * integral_pitch - config.Kd_pitch_angle * derivative_pitch ); //scaled by .01 to bring within -1 to 1 range

  // Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw * dt;
  if ( channel_1_pwm < 1060 ) { //don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain( integral_yaw, -config.i_limit, config.i_limit ); //saturate integrator to prevent unsafe buildup
  derivative_yaw = ( error_yaw - error_yaw_prev ) / dt;
  yaw_PID = .01 * ( config.Kp_yaw * error_yaw + config.Ki_yaw * integral_yaw + config.Kd_yaw * derivative_yaw ); //scaled by .01 to bring within -1 to 1 range

  // Update roll variables
  integral_roll_prev = integral_roll;
  // Update pitch variables
  integral_pitch_prev = integral_pitch;
  // Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void controlANGLE2()
{
  /* DESCRIPTION: Computes control commands based on state error (angle) in cascaded scheme
   *
   * Gives better performance than controlANGLE() but requires much more tuning. Not reccommended for first-time setup.
   * See the documentation for tuning this controller.
   */
  // Outer loop - PID on angle
  float roll_des_ol, pitch_des_ol;
  // Roll
  error_roll = roll_des - roll_IMU;
  integral_roll_ol = integral_roll_prev_ol + error_roll * dt;
  if ( channel_1_pwm < 1060 ) { //don't let integrator build if throttle is too low
    integral_roll_ol = 0;
  }
  integral_roll_ol = constrain( integral_roll_ol, -config.i_limit, config.i_limit ); //saturate integrator to prevent unsafe buildup
  derivative_roll = ( roll_IMU - roll_IMU_prev ) / dt;
  roll_des_ol = config.Kp_roll_angle * error_roll + config.Ki_roll_angle * integral_roll_ol - config.Kd_roll_angle * derivative_roll;

  // Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch_ol = integral_pitch_prev_ol + error_pitch * dt;
  if ( channel_1_pwm < 1060 ) { //don't let integrator build if throttle is too low
    integral_pitch_ol = 0;
  }
  integral_pitch_ol = constrain( integral_pitch_ol, -config.i_limit, config.i_limit ); //saturate integrator to prevent unsafe buildup
  derivative_pitch = ( pitch_IMU - pitch_IMU_prev ) / dt;
  pitch_des_ol = config.Kp_pitch_angle * error_pitch + config.Ki_pitch_angle * integral_pitch_ol - config.Kd_pitch_angle * derivative_pitch;

  // Apply loop gain, constrain, and LP filter for artificial damping
  float Kl = 30.0;
  roll_des_ol = Kl * roll_des_ol;
  pitch_des_ol = Kl * pitch_des_ol;
  roll_des_ol = constrain( roll_des_ol, -240.0, 240.0 );
  pitch_des_ol = constrain( pitch_des_ol, -240.0, 240.0 );
  roll_des_ol = ( 1.0 - config.B_loop_roll ) * roll_des_prev + config.B_loop_roll * roll_des_ol;
  pitch_des_ol = ( 1.0 - config.B_loop_pitch ) * pitch_des_prev + config.B_loop_pitch * pitch_des_ol;

  // Inner loop - PID on rate
  // Roll
  error_roll = roll_des_ol - GyroX;
  integral_roll_il = integral_roll_prev_il + error_roll * dt;
  if ( channel_1_pwm < 1060 ) { //don't let integrator build if throttle is too low
    integral_roll_il = 0;
  }
  integral_roll_il = constrain( integral_roll_il, -config.i_limit, config.i_limit ); //saturate integrator to prevent unsafe buildup
  derivative_roll = ( error_roll - error_roll_prev ) / dt;
  roll_PID = .01 * ( config.Kp_roll_rate * error_roll + config.Ki_roll_rate * integral_roll_il + config.Kd_roll_rate * derivative_roll ); //scaled by .01 to bring within -1 to 1 range

  // Pitch
  error_pitch = pitch_des_ol - GyroY;
  integral_pitch_il = integral_pitch_prev_il + error_pitch * dt;
  if ( channel_1_pwm < 1060 ) { //don't let integrator build if throttle is too low
    integral_pitch_il = 0;
  }
  integral_pitch_il = constrain( integral_pitch_il, -config.i_limit, config.i_limit ); //saturate integrator to prevent unsafe buildup
  derivative_pitch = ( error_pitch - error_pitch_prev ) / dt;
  pitch_PID = .01 * ( config.Kp_pitch_rate * error_pitch + config.Ki_pitch_rate * integral_pitch_il + config.Kd_pitch_rate * derivative_pitch ); //scaled by .01 to bring within -1 to 1 range

  // Yaw
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw * dt;
  if ( channel_1_pwm < 1060 ) { //don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain( integral_yaw, -config.i_limit, config.i_limit ); //saturate integrator to prevent unsafe buildup
  derivative_yaw = ( error_yaw - error_yaw_prev ) / dt;
  yaw_PID = .01 * ( config.Kp_yaw * error_yaw + config.Ki_yaw * integral_yaw + config.Kd_yaw * derivative_yaw ); //scaled by .01 to bring within -1 to 1 range

  // Update roll variables
  integral_roll_prev_ol = integral_roll_ol;
  integral_roll_prev_il = integral_roll_il;
  error_roll_prev = error_roll;
  roll_IMU_prev = roll_IMU;
  roll_des_prev = roll_des_ol;
  // Update pitch variables
  integral_pitch_prev_ol = integral_pitch_ol;
  integral_pitch_prev_il = integral_pitch_il;
  error_pitch_prev = error_pitch;
  pitch_IMU_prev = pitch_IMU;
  pitch_des_prev = pitch_des_ol;
  // Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;

}

void controlRATE()
{
  /* DESCRIPTION: Computes control commands based on state error (rate)
   *
   * See explanation for controlANGLE(). Everything is the same here except the error is now the desired rate - raw gyro reading.
   */
  // Roll
  error_roll = roll_des - GyroX;
  integral_roll = integral_roll_prev + error_roll * dt;
  if ( channel_1_pwm < 1060 ) { //don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain( integral_roll, -config.i_limit, config.i_limit ); //saturate integrator to prevent unsafe buildup
  derivative_roll = ( error_roll - error_roll_prev ) / dt;
  roll_PID = .01 * ( config.Kp_roll_rate * error_roll + config.Ki_roll_rate * integral_roll + config.Kd_roll_rate * derivative_roll ); //scaled by .01 to bring within -1 to 1 range

  // Pitch
  error_pitch = pitch_des - GyroY;
  integral_pitch = integral_pitch_prev + error_pitch * dt;
  if ( channel_1_pwm < 1060 ) { //don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain( integral_pitch, -config.i_limit, config.i_limit ); //saturate integrator to prevent unsafe buildup
  derivative_pitch = ( error_pitch - error_pitch_prev ) / dt;
  pitch_PID = .01 * ( config.Kp_pitch_rate * error_pitch + config.Ki_pitch_rate * integral_pitch + config.Kd_pitch_rate * derivative_pitch ); //scaled by .01 to bring within -1 to 1 range

  // Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw * dt;
  if ( channel_1_pwm < 1060 ) { //don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain( integral_yaw, -config.i_limit, config.i_limit ); //saturate integrator to prevent unsafe buildup
  derivative_yaw = ( error_yaw - error_yaw_prev ) / dt;
  yaw_PID = .01 * ( config.Kp_yaw * error_yaw + config.Ki_yaw * integral_yaw + config.Kd_yaw * derivative_yaw ); //scaled by .01 to bring within -1 to 1 range

  // Update roll variables
  error_roll_prev = error_roll;
  integral_roll_prev = integral_roll;
  GyroX_prev = GyroX;
  // Update pitch variables
  error_pitch_prev = error_pitch;
  integral_pitch_prev = integral_pitch;
  GyroY_prev = GyroY;
  // Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}