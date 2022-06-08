//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Version: Beta 1.2



//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES                                                 //                                                                 
//========================================================================================================================//



//========================================================================================================================//

//REQUIRED LIBRARIES (included with download in main sketch folder)



//========================================================================================================================//



//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //                           
//========================================================================================================================//



//========================================================================================================================//
//                                                     DECLARE PINS                                                       //                           
//========================================================================================================================//



//========================================================================================================================//

//DECLARE GLOBAL VARIABLES


//========================================================================================================================//
//                                                      VOID SETUP                                                        //                           
//========================================================================================================================//

void setup() {

  Serial.begin(500000); //usb serial
  delay(3000); //3 second delay for plugging in battery before IMU calibration begins, feel free to comment this out to reduce boot time
  
  //Initialize all pins

  //Set built in LED to turn on to signal startup & not to disturb vehicle during IMU calibration

  //Initialize radio communication
  radioSetup();
  
  //Set radio channels to default (safe) values before entering main loop

  //Initialize IMU communication
  IMUinit();

  //Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level
  calculate_IMU_error();

  //Arm servo channels

  //Arm OneShot125 motors
  commandMotors();

  //Warm up the loop
  calibrateAttitude(); //helps to warm up IMU and Madgwick filter before finally entering main loop
  
  //Indicate entering main loop with 3 quick blinks
  setupBlink(3,160,70); //numBlinks, upTime (ms), downTime (ms)

  //If using MPU9250 IMU, uncomment for one-time magnetometer calibration (may need to repeat for new locations)
  //calibrateMagnetometer(); //generates magentometer error and scale factors

}


//========================================================================================================================//
//                                                       MAIN LOOP                                                        //                           
//========================================================================================================================//
                                                  
void loop() {

  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  loopBlink(); //indicate we are in main loop with short blink every 1.5 seconds

  //Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:

  //Get vehicle state
  getIMUdata(); //pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
  Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt); //updates roll_IMU, pitch_IMU, and yaw_IMU (degrees)

  //Compute desired state
  getDesState(); //convert raw commands to normalized values based on saturated control limits
  
  //PID Controller - SELECT ONE:
  controlANGLE(); //stabilize on angle setpoint
  //controlANGLE2(); //stabilize on angle setpoint using cascaded method 
  //controlRATE(); //stabilize on rate setpoint

  //Actuator mixing and scaling to PWM values
  controlMixer(); //mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
  scaleCommands(); //scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)

  //Throttle cut check
  throttleCut(); //directly sets motor commands to low based on state of ch5

  //Command actuators
  commandMotors(); //sends command pulses to each motor pin using OneShot125 protocol
  servo7.write(sX_command_PWM);
    
  //Get vehicle commands for next loop iteration
  getCommands(); //pulls current available radio commands
  failSafe(); //prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup

  //Regulate loop rate
  loopRate(2000); //do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default

}


//========================================================================================================================//
//                                                      FUNCTIONS                                                         //                           
//========================================================================================================================//

void IMUinit() {}

void getIMUdata() {}

void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq) {}

void getDesState() {}

void controlANGLE() {}

void controlANGLE2() {}

void controlRATE() {}

void controlMixer() {}

void scaleCommands() {}

void getCommands() {}

void failSafe() {}

void commandMotors() {}

float floatFaderLinear(float param, float param_min, float param_max, float fadeTime, int state, int loopFreq){}

float switchRollYaw(int reverseRoll, int reverseYaw) {}

void throttleCut() {}

void calibrateMagnetometer() {}

void loopRate(int freq) {}

void loopBlink() {}

void setupBlink(int numBlinks,int upTime, int downTime) {}

void printRadioData() {}

void printDesiredState() {}

void printGyroData() {}

void printAccelData() {}

void printMagData() {}

void printRollPitchYaw() {}

void printPIDoutput() {}

void printMotorCommands() {}

void printServoCommands() {}

void printLoopRate() {}

//=========================================================================================//

//HELPER FUNCTIONS

float invSqrt(float x) {}