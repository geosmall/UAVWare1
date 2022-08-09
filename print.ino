#include "src/uvos/uvos_telem.h"

#define PRN_BUFFER_SIZE  128
static char prn_buffer[PRN_BUFFER_SIZE] = {0};

// void printGyroData()
// {
//   if ( current_time - print_counter > 10000 ) {
//     print_counter = micros();
//     uint16_t prn_buflen = snprintf_( prn_buffer, sizeof( prn_buffer ), "GyroX: %7.1f GyroY: %7.1f GyroZ: %7.1f\r\n", GyroX, GyroY, GyroZ );
//     UVOS_TELEM_puts( ( uint8_t * ) prn_buffer, prn_buflen );
//   }
// }

// void printAccelData()
// {
//   if ( current_time - print_counter > 10000 ) {
//     print_counter = micros();
//     uint16_t prn_buflen = snprintf_( prn_buffer, sizeof( prn_buffer ), "AccX: %7.3f AccY: %7.3f AccZ: %7.3f\r\n", AccX, AccY, AccZ );
//     UVOS_TELEM_puts( ( uint8_t * ) prn_buffer, prn_buflen );
//   }
// }

void printRadioData()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    // Serial.print(F(" CH1: "));
    // Serial.print(channel_1_pwm);
    // Serial.print(F(" CH2: "));
    // Serial.print(channel_2_pwm);
    // Serial.print(F(" CH3: "));
    // Serial.print(channel_3_pwm);
    // Serial.print(F(" CH4: "));
    // Serial.print(channel_4_pwm);
    // Serial.print(F(" CH5: "));
    // Serial.print(channel_5_pwm);
    // Serial.print(F(" CH6: "));
    // Serial.println(channel_6_pwm);
    uint16_t prn_buflen \
      = snprintf_\
        ( prn_buffer, sizeof( prn_buffer ), \
          "CH1: %-6d CH2: %-6d CH3: %-6d CH4: %-6d CH5: %-6d CH6: %-6d\r\n", \
          channel_1_pwm, \
          channel_2_pwm, \
          channel_3_pwm, \
          channel_4_pwm, \
          channel_5_pwm, \
          channel_6_pwm \
        );
    UVOS_TELEM_puts( ( uint8_t * ) prn_buffer, prn_buflen );
  }
}

void printDesiredState()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    // Serial.print(F("thro_des: "));
    // Serial.print(thro_des);
    // Serial.print(F(" roll_des: "));
    // Serial.print(roll_des);
    // Serial.print(F(" pitch_des: "));
    // Serial.print(pitch_des);
    // Serial.print(F(" yaw_des: "));
    // Serial.println(yaw_des);
    // uint16_t prn_buflen = snprintf_( prn_buffer, sizeof( prn_buffer ), "thro_des: %5.3f roll_des: %5.3f pitch_des: %5.3f yaw_des: %5.3f\r\n", thro_des, roll_des, pitch_des, yaw_des );
    uint16_t prn_buflen \
      = snprintf_\
        ( prn_buffer, sizeof( prn_buffer ), \
          "\
          thro_des: %5.3f \
          roll_des: %5.3f \
          pitch_des: %5.3f \
          yaw_des: %5.3f \
          \r\n", \
          thro_des, \
          roll_des, \
          pitch_des, \
          yaw_des \
        );
    UVOS_TELEM_puts( ( uint8_t * ) prn_buffer, prn_buflen );
  }
}

void printGyroData()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    // Serial.print(F("GyroX: "));
    // Serial.print(GyroX);
    // Serial.print(F(" GyroY: "));
    // Serial.print(GyroY);
    // Serial.print(F(" GyroZ: "));
    // Serial.println(GyroZ);
    // uint16_t prn_buflen = snprintf_( prn_buffer, sizeof( prn_buffer ), "GyroX: %7.1f GyroY: %7.1f GyroZ: %7.1f\r\n", GyroX, GyroY, GyroZ );
    uint16_t prn_buflen \
      = snprintf_\
        ( prn_buffer, sizeof( prn_buffer ), \
          "\
          GyroX: %7.1f \
          GyroY: %7.1f \
          GyroZ: %7.1f \
          \r\n", \
          GyroX, \
          GyroY, \
          GyroZ \
        );
    UVOS_TELEM_puts( ( uint8_t * ) prn_buffer, prn_buflen );
  }
}

void printAccelData()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    // Serial.print(F("AccX: "));
    // Serial.print(AccX);
    // Serial.print(F(" AccY: "));
    // Serial.print(AccY);
    // Serial.print(F(" AccZ: "));
    // Serial.println(AccZ);
    // uint16_t prn_buflen = snprintf_( prn_buffer, sizeof( prn_buffer ), "AccX: %7.3f AccY: %7.3f AccZ: %7.3f\r\n", AccX, AccY, AccZ );
    uint16_t prn_buflen \
      = snprintf_\
        ( prn_buffer, sizeof( prn_buffer ), \
          "\
          AccX: %7.3f \
          AccY: %7.3f \
          AccZ: %7.3f \
          \r\n", \
          AccX, \
          AccY, \
          AccZ \
        );
    UVOS_TELEM_puts( ( uint8_t * ) prn_buffer, prn_buflen );
  }
}

void printMagData()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    // Serial.print(F("MagX: "));
    // Serial.print(MagX);
    // Serial.print(F(" MagY: "));
    // Serial.print(MagY);
    // Serial.print(F(" MagZ: "));
    // Serial.println(MagZ);
    uint16_t prn_buflen \
      = snprintf_\
        ( prn_buffer, sizeof( prn_buffer ), \
          "\
          MagX: %7.3f \
          MagY: %7.3f \
          MagZ: %7.3f \
          \r\n", \
          MagX, \
          MagY, \
          MagZ \
        );
    UVOS_TELEM_puts( ( uint8_t * ) prn_buffer, prn_buflen );
  }
}

void printRollPitchYaw()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    // Serial.print(F("roll: "));
    // Serial.print(roll_IMU);
    // Serial.print(F(" pitch: "));
    // Serial.print(pitch_IMU);
    // Serial.print(F(" yaw: "));
    // Serial.println(yaw_IMU);
  }
}

void printPIDoutput()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    // Serial.print(F("roll_PID: "));
    // Serial.print(roll_PID);
    // Serial.print(F(" pitch_PID: "));
    // Serial.print(pitch_PID);
    // Serial.print(F(" yaw_PID: "));
    // Serial.println(yaw_PID);
  }
}

void printMotorCommands()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    // Serial.print(F("m1_command: "));
    // Serial.print(m1_command_PWM);
    // Serial.print(F(" m2_command: "));
    // Serial.print(m2_command_PWM);
    // Serial.print(F(" m3_command: "));
    // Serial.print(m3_command_PWM);
    // Serial.print(F(" m4_command: "));
    // Serial.print(m4_command_PWM);
    // Serial.print(F(" m5_command: "));
    // Serial.print(m5_command_PWM);
    // Serial.print(F(" m6_command: "));
    // Serial.println(m6_command_PWM);
  }
}

void printServoCommands()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    // Serial.print(F("s1_command: "));
    // Serial.print(s1_command_PWM);
    // Serial.print(F(" s2_command: "));
    // Serial.print(s2_command_PWM);
    // Serial.print(F(" s3_command: "));
    // Serial.print(s3_command_PWM);
    // Serial.print(F(" s4_command: "));
    // Serial.print(s4_command_PWM);
    // Serial.print(F(" s5_command: "));
    // Serial.print(s5_command_PWM);
    // Serial.print(F(" s6_command: "));
    // Serial.print(s6_command_PWM);
    // Serial.print(F(" s7_command: "));
    // Serial.println(s7_command_PWM);
  }
}

void printLoopRate()
{
  if ( current_time - print_counter > 10000 ) {
    print_counter = micros();
    // Serial.print(F("dt = "));
    // Serial.println(dt*1000000.0);
  }
}