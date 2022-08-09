#include "src/UAVWare/uavw.h"
#include "config.h"
#include "src/libraries/printf/printf.h"

//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES                                                 //
//========================================================================================================================//

// Uncomment only one IMU
// #define USE_MPU_6DOF //default
#define USE_MPU_9DOF

// Uncomment only one full scale gyro range (deg/sec)
#define GYRO_250DPS //default
// #define GYRO_500DPS
// #define GYRO_1000DPS
// #define GYRO_2000DPS

// Uncomment only one full scale accelerometer range (G's)
#define ACCEL_2G //default
// #define ACCEL_4G
// #define ACCEL_8G
// #define ACCEL_16G

// Main loop frequency in Hertz
#define LOOP_FREQ_HZ 1000

// Number of sample loops for imu calibration
#define CAL_ITERATION_LOOPS 12000

//========================================================================================================================//

#if defined GYRO_250DPS
#define GYRO_SCALE GYRO_FS_250DPS
#define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
#define GYRO_SCALE GYRO_FS_500DPS
#define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
#define GYRO_SCALE GYRO_FS_1000DPS
#define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
#define GYRO_SCALE GYRO_FS_2000DPS
#define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
#define ACCEL_SCALE ACCEL_FS_2G
#define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
#define ACCEL_SCALE ACCEL_FS_4G
#define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
#define ACCEL_SCALE ACCEL_FS_8G
#define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
#define ACCEL_SCALE ACCEL_FS_16G
#define ACCEL_SCALE_FACTOR 2048.0
#endif

//========================================================================================================================//

// NOTE: C convention is 0 for success and 1 for failure

//DECLARE GLOBAL VARIABLES

//General stuff
// const float dt = 1 / ( LOOP_FREQ_HZ );
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;

//Radio comm:
unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;

// #if defined USE_SBUS_RX
//   SBUS sbus(Serial5);
//   uint16_t sbusChannels[16];
//   bool sbusFailSafe;
//   bool sbusLostFrame;
// #endif
// uint16_t rcChannels[16];
// bool rcFailSafe;
// bool rcLostFrame;
#define NUM_CHANNELS 10
uint16_t rc_command_uSec[ NUM_CHANNELS ];

// IMU:
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float MagX, MagY, MagZ;
float MagX_prev, MagY_prev, MagZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
float q0 = 1.0f; // initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

// Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;

// Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

// Mixer
float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled, m6_command_scaled;
int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;
float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;
int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM;


void setup()
{

  // Initialize UAVWare services
  if ( UAVW_init() != EXIT_SUCCESS ) {
    failloop( 4 );
  }

  // Set config struct default values
  config_set_defaults();

  // LED on to signal startup & not to disturb vehicle during IMU calibration
  void led_status_on();

  time_delay_us( 10000 );

  // Set IMU full-scale ranges for accel and gyro
  mpu_set_fullscale_gyro_range( GYRO_SCALE );
  mpu_set_fullscale_accel_range( ACCEL_SCALE );

  //Set radio channels to default (safe) values before entering main loop
  channel_1_pwm = config.channel_1_fs;
  channel_2_pwm = config.channel_2_fs;
  channel_3_pwm = config.channel_3_fs;
  channel_4_pwm = config.channel_4_fs;
  channel_5_pwm = config.channel_5_fs;
  channel_6_pwm = config.channel_6_fs;

  // Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level
  calculate_IMU_error();

  time_delay_us( 10000 );

  //Arm servo channels
  // servo1.write(0); //command servo angle from 0-180 degrees (1000 to 2000 PWM)
  // servo2.write(0);
  // servo3.write(0);
  // servo4.write(0);
  // servo5.write(0);
  // servo6.write(0);
  // servo7.write(0);

  time_delay_us( 10000 );

  //Arm OneShot125 motors
  // m1_command_PWM = 125; //command OneShot125 ESC from 125 to 250us pulse length
  // m2_command_PWM = 125;
  // m3_command_PWM = 125;
  // m4_command_PWM = 125;
  // m5_command_PWM = 125;
  // m6_command_PWM = 125;
  // commandMotors();

  time_delay_us( 100000 );

  actuator_update_outputs();
  actuator_enable_outputs();

  // Warm up the loop
  calibrateAttitude(); // Helps to warm up IMU and Madgwick filter before finally entering main loop

  // Indicate entering main loop with 3 quick blinks
  setupBlink( 3, 160000, 70000 ); // NumBlinks, upTime (us), downTime (us)

  while ( 1 ) {

    prev_time = current_time;
    current_time = micros();
    dt = ( current_time - prev_time ) / 1000000.0;

    loopBlink(); // Indicate we are in main loop with short blink every 1.5 seconds

    // Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
    printRadioData();     // radio pwm values (expected: 1000 to 2000)
    // printDesiredState();  // prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0 to 1 for throttle)
    // printGyroData();      // prints filtered gyro data direct from IMU (expected: ~ -250 to 250, 0 at rest)
    // printAccelData();     // prints filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0 when level, z 1 when level)
    // printMagData();       // prints filtered magnetometer data direct from IMU (expected: ~ -300 to 300)
    // printRollPitchYaw();  // prints roll, pitch, and yaw angles in degrees from Madgwick filter (expected: degrees, 0 when level)
    // printPIDoutput();     // prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
    // printMotorCommands(); // prints the values being written to the motors (expected: 120 to 250)
    // printServoCommands(); // prints the values being written to the servos (expected: 0 to 180)
    // printLoopRate();      // prints the time between loops in microseconds (expected: microseconds between loop iterations)

    // Get vehicle state
    getIMUdata(); // Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
    Madgwick( GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt ); // Updates roll_IMU, pitch_IMU, and yaw_IMU (degrees)

    // Compute desired state
    getDesState(); // Convert raw commands to normalized values based on saturated control limits

    // PID Controller - SELECT ONE:
    controlANGLE();  // stabilize on angle setpoint
    // controlANGLE2(); // stabilize on angle setpoint using cascaded method
    // controlRATE();   // stabilize on rate setpoint

    // Actuator mixing and scaling to PWM values
    controlMixer(); // mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
    scaleCommands(); // scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)

    // Throttle cut check
    throttleCut(); // directly sets motor commands to low based on state of ch5

    // Command actuators
    commandMotors(); // sends command pulses to each motor pin using OneShot125 protocol
    // servo1.write( s1_command_PWM );
    // servo2.write( s2_command_PWM );
    // servo3.write( s3_command_PWM );
    // servo4.write( s4_command_PWM );
    // servo5.write( s5_command_PWM );
    // servo6.write( s6_command_PWM );
    // servo7.write( s7_command_PWM );

    // Get vehicle commands for next loop iteration
    getCommands(); // pulls current available radio commands

    failSafe();    // prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup

    // Regulate loop rate
    loopRate( 2000 ); // do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default

  }

#if 0

  // Init scheduler with our loop frequency as tick rate
  sch_init_hz( LOOP_FREQ_HZ );

  RC_TASK_init();
  SYS_STATS_TASK_init();
  HEARTBEAT_TASK_init();

  // Add tasks to schedule.
  // Parameters are:
  // A. Task name
  // B. Initial delay / offset (in Ticks)
  // C. Task period (in Ticks): Must be > 0
  //                 A                        B  C
  if ( sch_add_task( RC_TASK_update,          0, 1 ) )     // Update RC command
    failloop( 7 );
  if ( sch_add_task( SYS_STATS_TASK_update,   0, 100 ) )   // Update MAVLink system stats
    failloop( 7 );
  if ( sch_add_task( HEARTBEAT_TASK_update,   0, 1000 ) )  // MAVLink heartbeat update
    failloop( 7 );

  // Start the scheduler
  sch_start();

  // Scheduler task loop
  while ( 1 ) {
    if ( sch_dispatch_tasks() ) {
      failloop ( 7 );
    }
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

void getIMUdata()
{
  /* DESCRIPTION: Request full dataset from IMU and LP filter gyro, accelerometer, and magnetometer data
   *
   * Reads accelerometer and gyro data from IMU as AccX, AccY, AccZ, GyroX, GyroY, GyroZ.
   * These values are scaled according to the IMU datasheet to put them into correct units of g's, deg/sec, and uT.
   * A simple first-order low-pass filter is used to get rid of high frequency noise in these raw signals.
   * Generally you want to cut off everything past 80Hz. If your loop rate is not fast enough,
   * the low pass filter will cause a lag in the readings. The filter parameters B_gyro and B_accel
   * are set to be good for a 2kHz loop rate.
   * Finally, the constant errors found in calculate_IMU_error() on startup are subtracted from
   * the accelerometer and gyro readings.
   */
  int16_t accel[3]; // 0=X,1=Y,2=Z
  int16_t gyro[3];  // 0=X,1=Y,2=Z
  int16_t temp;

  mpu_read_imu( accel, gyro, &temp );

  // Accelerometer
  AccX = accel[0] / ACCEL_SCALE_FACTOR; //G's
  AccY = accel[1] / ACCEL_SCALE_FACTOR;
  AccZ = accel[2] / ACCEL_SCALE_FACTOR;
  // Correct the outputs with the calculated error values
  AccX = AccX - AccErrorX;
  AccY = AccY - AccErrorY;
  AccZ = AccZ - AccErrorZ;
  // LP filter accelerometer data
  AccX = ( 1.0 - config.B_accel ) * AccX_prev + config.B_accel * AccX;
  AccY = ( 1.0 - config.B_accel ) * AccY_prev + config.B_accel * AccY;
  AccZ = ( 1.0 - config.B_accel ) * AccZ_prev + config.B_accel * AccZ;
  AccX_prev = AccX;
  AccY_prev = AccY;
  AccZ_prev = AccZ;

  // Gyro
  GyroX = gyro[0] / GYRO_SCALE_FACTOR; //deg/sec
  GyroY = gyro[1] / GYRO_SCALE_FACTOR;
  GyroZ = gyro[2] / GYRO_SCALE_FACTOR;
  // Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;
  // LP filter gyro data
  GyroX = ( 1.0 - config.B_gyro ) * GyroX_prev + config.B_gyro * GyroX;
  GyroY = ( 1.0 - config.B_gyro ) * GyroY_prev + config.B_gyro * GyroY;
  GyroZ = ( 1.0 - config.B_gyro ) * GyroZ_prev + config.B_gyro * GyroZ;
  GyroX_prev = GyroX;
  GyroY_prev = GyroY;
  GyroZ_prev = GyroZ;
}


void calculate_IMU_error()
{
  /* DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
   *
   * The error values computed are applied to the raw gyro and accelerometer values AccX, AccY, AccZ,
   * GyroX, GyroY, GyroZ in getIMUdata(). This reduces measurement drift due to sensor bias.
   */
  int16_t accel[3]; // 0=X,1=Y,2=Z
  int16_t gyro[3];  // 0=X,1=Y,2=Z
  int16_t temp;

  // Read IMU values 'count' times
  int c = 0;
  while ( c < CAL_ITERATION_LOOPS ) {
    mpu_read_imu( accel, gyro, &temp );

    // Scale raw readings
    AccX  = accel[0] / ACCEL_SCALE_FACTOR;
    AccY  = accel[1] / ACCEL_SCALE_FACTOR;
    AccZ  = accel[2] / ACCEL_SCALE_FACTOR;
    GyroX = gyro[0] / GYRO_SCALE_FACTOR;
    GyroY = gyro[1] / GYRO_SCALE_FACTOR;
    GyroZ = gyro[2] / GYRO_SCALE_FACTOR;

    // Sum all readings
    AccErrorX  = AccErrorX + AccX;
    AccErrorY  = AccErrorY + AccY;
    AccErrorZ  = AccErrorZ + AccZ;
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;

    c++;
  }

  // Divide the sum by 'count' to get the error value
  AccErrorX  = AccErrorX / c;
  AccErrorY  = AccErrorY / c;
  AccErrorZ  = AccErrorZ / c - 1.0;
  GyroErrorX = GyroErrorX / c;
  GyroErrorY = GyroErrorY / c;
  GyroErrorZ = GyroErrorZ / c;

}


void calibrateAttitude()
{
  // DESCRIPTION: Used to warm up the main loop to allow the madwick filter to converge before commands can be sent to the actuators
  // Assuming vehicle is powered up on level surface!
  /*
   * This function is used on startup to warm up the attitude estimation and is what causes startup to take a few seconds
   * to boot.
   */
  // Warm up IMU and madgwick filter in simulated main loop
  for ( int i = 0; i <= 10000; i++ ) {
    prev_time = current_time;
    current_time = micros();
    dt = ( current_time - prev_time ) / 1000000.0;
    getIMUdata();
    Madgwick6DOF( GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, dt );
    loopRate( 2000 ); //do not exceed 2000Hz
  }
}

void getDesState()
{
  /* DESCRIPTION: Normalizes desired control values to appropriate values
   *
   * Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw
   * RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.
   * roll_des and pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec
   * (rate mode). yaw_des is scaled to be within max yaw in degrees/sec. Also creates roll_passthru, pitch_passthru, and
   * yaw_passthru variables, to be used in commanding motors/servos with direct unstabilized commands in controlMixer().
   */
  thro_des = ( channel_1_pwm - 1000.0 ) / 1000.0; // between 0 and 1
  roll_des = ( channel_2_pwm - 1500.0 ) / 500.0;  // between -1 and 1
  pitch_des = ( channel_3_pwm - 1500.0 ) / 500.0; // between -1 and 1
  yaw_des = ( channel_4_pwm - 1500.0 ) / 500.0;   // between -1 and 1
  // Constrain within normalized bounds
  thro_des = constrain( thro_des, 0.0, 1.0 );                      // between 0 and 1
  roll_des = constrain( roll_des, -1.0, 1.0 ) * config.maxRoll;    // between -maxRoll and +maxRoll
  pitch_des = constrain( pitch_des, -1.0, 1.0 ) * config.maxPitch; // between -maxPitch and +maxPitch
  yaw_des = constrain( yaw_des, -1.0, 1.0 ) * config.maxYaw;       // between -maxYaw and +maxYaw

  roll_passthru = roll_des / ( 2.0 * config.maxRoll );
  pitch_passthru = pitch_des / ( 2.0 * config.maxPitch );
  yaw_passthru = yaw_des / ( 2.0 * config.maxYaw );
}

void scaleCommands()
{
  /* DESCRIPTION: Scale normalized actuator commands to values for ESC/Servo protocol
   *
   * mX_command_scaled variables from the mixer function are scaled to 125-250us for OneShot125 protocol. sX_command_scaled variables from
   * the mixer function are scaled to 0-180 for the servo library using standard PWM.
   * mX_command_PWM are updated here which are used to command the motors in commandMotors(). sX_command_PWM are updated
   * which are used to command the servos.
   */
  // Scaled to 125us - 250us for oneshot125 protocol
  m1_command_PWM = m1_command_scaled * 125 + 125;
  m2_command_PWM = m2_command_scaled * 125 + 125;
  m3_command_PWM = m3_command_scaled * 125 + 125;
  m4_command_PWM = m4_command_scaled * 125 + 125;
  m5_command_PWM = m5_command_scaled * 125 + 125;
  m6_command_PWM = m6_command_scaled * 125 + 125;
  // Constrain commands to motors within oneshot125 bounds
  m1_command_PWM = constrain( m1_command_PWM, 125, 250 );
  m2_command_PWM = constrain( m2_command_PWM, 125, 250 );
  m3_command_PWM = constrain( m3_command_PWM, 125, 250 );
  m4_command_PWM = constrain( m4_command_PWM, 125, 250 );
  m5_command_PWM = constrain( m5_command_PWM, 125, 250 );
  m6_command_PWM = constrain( m6_command_PWM, 125, 250 );

  // Scaled to 0-180 for servo library
  s1_command_PWM = s1_command_scaled * 180;
  s2_command_PWM = s2_command_scaled * 180;
  s3_command_PWM = s3_command_scaled * 180;
  s4_command_PWM = s4_command_scaled * 180;
  s5_command_PWM = s5_command_scaled * 180;
  s6_command_PWM = s6_command_scaled * 180;
  s7_command_PWM = s7_command_scaled * 180;
  // Constrain commands to servos within servo library bounds
  s1_command_PWM = constrain( s1_command_PWM, 0, 180 );
  s2_command_PWM = constrain( s2_command_PWM, 0, 180 );
  s3_command_PWM = constrain( s3_command_PWM, 0, 180 );
  s4_command_PWM = constrain( s4_command_PWM, 0, 180 );
  s5_command_PWM = constrain( s5_command_PWM, 0, 180 );
  s6_command_PWM = constrain( s6_command_PWM, 0, 180 );
  s7_command_PWM = constrain( s7_command_PWM, 0, 180 );

}

void getCommands()
{
  //DESCRIPTION: Get raw PWM values for every channel from the radio
  /*
   * Updates radio PWM commands in loop based on current available commands. channel_x_pwm is the raw command used in the rest of
   * the loop. If using a PWM or PPM receiver, the radio commands are retrieved from a function in the readPWM file separate from this one which
   * is running a bunch of interrupts to continuously update the radio readings. If using an SBUS receiver, the alues are pulled from the SBUS library directly.
   * The raw radio commands are filtered with a first order low-pass filter to eliminate any really high frequency noise.
   */

// #if defined USE_PPM_RX || defined USE_PWM_RX
//   channel_1_pwm = getRadioPWM( 1 );
//   channel_2_pwm = getRadioPWM( 2 );
//   channel_3_pwm = getRadioPWM( 3 );
//   channel_4_pwm = getRadioPWM( 4 );
//   channel_5_pwm = getRadioPWM( 5 );
//   channel_6_pwm = getRadioPWM( 6 );

// #elif defined USE_SBUS_RX
//   if ( sbus.read( &sbusChannels[0], &sbusFailSafe, &sbusLostFrame ) ) {
//     //sBus scaling below is for Taranis-Plus and X4R-SB
//     float scale = 0.615;
//     float bias  = 895.0;
//     channel_1_pwm = sbusChannels[0] * scale + bias;
//     channel_2_pwm = sbusChannels[1] * scale + bias;
//     channel_3_pwm = sbusChannels[2] * scale + bias;
//     channel_4_pwm = sbusChannels[3] * scale + bias;
//     channel_5_pwm = sbusChannels[4] * scale + bias;
//     channel_6_pwm = sbusChannels[5] * scale + bias;
//   }
// #endif

  rc_update( rc_command_uSec, COUNT_OF( rc_command_uSec ) );
  channel_1_pwm = rc_command_uSec[ 2 ]; // thro
  channel_2_pwm = rc_command_uSec[ 0 ]; // ail
  channel_3_pwm = rc_command_uSec[ 1 ]; // elev
  channel_4_pwm = rc_command_uSec[ 3 ]; // rudd
  channel_5_pwm = rc_command_uSec[ 6 ]; // throttle cut (SWA)
  channel_6_pwm = rc_command_uSec[ 9 ]; // aux1 (SWD)

  //Low-pass the critical commands and update previous values
  float b = 0.2; //lower=slower, higher=noiser
  channel_1_pwm = ( 1.0 - b ) * channel_1_pwm_prev + b * channel_1_pwm;
  channel_2_pwm = ( 1.0 - b ) * channel_2_pwm_prev + b * channel_2_pwm;
  channel_3_pwm = ( 1.0 - b ) * channel_3_pwm_prev + b * channel_3_pwm;
  channel_4_pwm = ( 1.0 - b ) * channel_4_pwm_prev + b * channel_4_pwm;
  channel_1_pwm_prev = channel_1_pwm;
  channel_2_pwm_prev = channel_2_pwm;
  channel_3_pwm_prev = channel_3_pwm;
  channel_4_pwm_prev = channel_4_pwm;
}

void failSafe()
{
  //DESCRIPTION: If radio gives garbage values, set all commands to default values
  /*
   * Radio connection failsafe used to check if the getCommands() function is returning acceptable pwm values. If any of
   * the commands are lower than 800 or higher than 2200, then we can be certain that there is an issue with the radio
   * connection (most likely hardware related). If any of the channels show this failure, then all of the radio commands
   * channel_x_pwm are set to default failsafe values specified in the setup. Comment out this function when troubleshooting
   * your radio connection in case any extreme values are triggering this function to overwrite the printed variables.
   */
  unsigned minVal = 800;
  unsigned maxVal = 2200;
  int check1 = 0;
  int check2 = 0;
  int check3 = 0;
  int check4 = 0;
  int check5 = 0;
  int check6 = 0;

  //Triggers for failure criteria
  if ( channel_1_pwm > maxVal || channel_1_pwm < minVal ) check1 = 1;
  if ( channel_2_pwm > maxVal || channel_2_pwm < minVal ) check2 = 1;
  if ( channel_3_pwm > maxVal || channel_3_pwm < minVal ) check3 = 1;
  if ( channel_4_pwm > maxVal || channel_4_pwm < minVal ) check4 = 1;
  if ( channel_5_pwm > maxVal || channel_5_pwm < minVal ) check5 = 1;
  if ( channel_6_pwm > maxVal || channel_6_pwm < minVal ) check6 = 1;

  //If any failures, set to default failsafe values
  if ( ( check1 + check2 + check3 + check4 + check5 + check6 ) > 0 ) {
    channel_1_pwm = config.channel_1_fs;
    channel_2_pwm = config.channel_2_fs;
    channel_3_pwm = config.channel_3_fs;
    channel_4_pwm = config.channel_4_fs;
    channel_5_pwm = config.channel_5_fs;
    channel_6_pwm = config.channel_6_fs;
  }
}

void commandMotors()
{
  //DESCRIPTION: Send pulses to motor pins, oneshot125 protocol
  /*
   * My crude implimentation of OneShot125 protocol which sends 125 - 250us pulses to the ESCs (mXPin). The pulselengths being
   * sent are mX_command_PWM, computed in scaleCommands(). This may be replaced by something more efficient in the future.
   */
}

float floatFaderLinear( float param, float param_min, float param_max, float fadeTime, int state, int loopFreq )
{
  //DESCRIPTION: Linearly fades a float type variable between min and max bounds based on desired high or low state and time
  /*
   *  Takes in a float variable, desired minimum and maximum bounds, fade time, high or low desired state, and the loop frequency
   *  and linearly interpolates that param variable between the maximum and minimum bounds. This function can be called in controlMixer()
   *  and high/low states can be determined by monitoring the state of an auxillarly radio channel. For example, if channel_6_pwm is being
   *  monitored to switch between two dynamic configurations (hover and forward flight), this function can be called within the logical
   *  statements in order to fade controller gains, for example between the two dynamic configurations. The 'state' (1 or 0) can be used
   *  to designate the two final options for that control gain based on the dynamic configuration assignment to the auxillary radio channel.
   *
   */
  float diffParam = ( param_max - param_min ) / ( fadeTime * loopFreq ); //difference to add or subtract from param for each loop iteration for desired fadeTime

  if ( state == 1 ) { //maximum param bound desired, increase param by diffParam for each loop iteration
    param = param + diffParam;
  } else if ( state == 0 ) { //minimum param bound desired, decrease param by diffParam for each loop iteration
    param = param - diffParam;
  }

  param = constrain( param, param_min, param_max ); //constrain param within max bounds

  return param;
}

float switchRollYaw( int reverseRoll, int reverseYaw )
{
  //DESCRIPTION: Switches roll_des and yaw_des variables for tailsitter-type configurations
  /*
   * Takes in two integers (either 1 or -1) corresponding to the desired reversing of the roll axis and yaw axis, respectively.
   * Reversing of the roll or yaw axis may be needed when switching between the two for some dynamic configurations. Inputs of 1, 1 does not
   * reverse either of them, while -1, 1 will reverse the output corresponding to the new roll axis.
   * This function may be replaced in the future by a function that switches the IMU data instead (so that angle can also be estimated with the
   * IMU tilted 90 degrees from default level).
   */
  float switch_holder;

  switch_holder = yaw_des;
  yaw_des = reverseYaw * roll_des;
  roll_des = reverseRoll * switch_holder;
}

#if 0
void calibrateMagnetometer()
{
#if defined USE_MPU_9DOF
  float success;
  Serial_println( "Beginning magnetometer calibration in" );
  Serial_println( "3..." );
  delay( 1000 );
  Serial_println( "2..." );
  delay( 1000 );
  Serial_println( "1..." );
  delay( 1000 );
  Serial_println( "Rotate the IMU about all axes until complete." );
  Serial_println( " " );
  success = mpu9250.calibrateMag();
  if ( success ) {
    Serial_println( "Calibration Successful!" );
    Serial_println( "Please comment out the calibrateMagnetometer() function and copy these values into the code:" );
    Serial_print( "float MagErrorX = " );
    Serial_print( mpu9250.getMagBiasX_uT() );
    Serial_println( ";" );
    Serial_print( "float MagErrorY = " );
    Serial_print( mpu9250.getMagBiasY_uT() );
    Serial_println( ";" );
    Serial_print( "float MagErrorZ = " );
    Serial_print( mpu9250.getMagBiasZ_uT() );
    Serial_println( ";" );
    Serial_print( "float MagScaleX = " );
    Serial_print( mpu9250.getMagScaleFactorX() );
    Serial_println( ";" );
    Serial_print( "float MagScaleY = " );
    Serial_print( mpu9250.getMagScaleFactorY() );
    Serial_println( ";" );
    Serial_print( "float MagScaleZ = " );
    Serial_print( mpu9250.getMagScaleFactorZ() );
    Serial_println( ";" );
    Serial_println( " " );
    Serial_println( "If you are having trouble with your attitude estimate at a new flying location, repeat this process as needed." );
  } else {
    Serial_println( "Calibration Unsuccessful. Please reset the board and try again." );
  }

  while ( 1 ); //halt code so it won't enter main loop until this function commented out
#endif
  Serial_println( "Error: MPU9250 not selected. Cannot calibrate non-existent magnetometer." );
  while ( 1 ); //halt code so it won't enter main loop until this function commented out
}
#endif

void throttleCut()
{
  /* DESCRIPTION: Directly set actuator outputs to minimum value if triggered
   *
   * Monitors the state of radio command channel_5_pwm and directly sets the mx_command_PWM values to minimum (120 is
   * minimum for oneshot125 protocol, 0 is minimum for standard PWM servo library used) if channel 5 is high. This is the last function
   * called before commandMotors() is called so that the last thing checked is if the user is giving permission to command
   * the motors to anything other than minimum value. Safety first.
   */
  if ( channel_5_pwm > 1500 ) {
    m1_command_PWM = 120;
    m2_command_PWM = 120;
    m3_command_PWM = 120;
    m4_command_PWM = 120;
    m5_command_PWM = 120;
    m6_command_PWM = 120;

    //uncomment if using servo PWM variables to control motor ESCs
    //s1_command_PWM = 0;
    //s2_command_PWM = 0;
    //s3_command_PWM = 0;
    //s4_command_PWM = 0;
    //s5_command_PWM = 0;
    //s6_command_PWM = 0;
    //s7_command_PWM = 0;
  }
}

//========================================================================================================================//

// HELPER FUNCTIONS

float invSqrt( float x )
{
  // Fast inverse sqrt for madgwick filter
  /*
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
  */
  // alternate form:
  unsigned int i = 0x5F1F1412 - ( *( unsigned int * )&x >> 1 );
  float tmp = *( float * )&i;
  float y = tmp * ( 1.69000231f - 0.714158168f * x * tmp * tmp );
  return y;
}

void loopRate( int freq )
{
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters.
   */
  float invFreq = 1.0 / freq * 1000000.0;
  unsigned long checker = micros();

  //Sit in loop until appropriate time has passed
  while ( invFreq > ( checker - current_time ) ) {
    checker = micros();
  }
}

void loopBlink()
{
  /* DESCRIPTION: Blink LED on board to indicate main loop is running */
  if ( current_time - blink_counter > blink_delay ) {
    blink_counter = micros();
    led_status_toggle();
    if ( blinkAlternate == 1 ) {
      blinkAlternate = 0;
      blink_delay = 100000;
    } else if ( blinkAlternate == 0 ) {
      blinkAlternate = 1;
      blink_delay = 2000000;
    }
  }
}

void setupBlink( int numBlinks, int upTime_us, int downTime_us )
{
  //DESCRIPTION: Simple function to make LED on board blink as desired
  for ( int j = 1; j <= numBlinks; j++ ) {
    led_status_on();
    time_delay_us( downTime_us );
    led_status_off();
    time_delay_us( upTime_us );
  }
}

// 2 - low battery at powerup - if enabled by config
// 3 - radio chip not detected
// 4 - Gyro not found
// 5 - clock, interrupts, systick, bad code
// 6 - flash write error
// 7 - scheduler fault
extern "C" void failloop( int val )
{
  // for ( int i = 0; i <= 3; ++i ) {  <<<-- GLS: TODO as this disables 4 motors
  //  pwm_set( i, 0 );
  // }
  while ( true ) {
    for ( int i = 0; i < val; ++i ) {
      led_status_on();
      time_delay_us( 200000 );
      led_status_off();
      time_delay_us( 200000 );
    }
    time_delay_us( 800000 );
  }
}