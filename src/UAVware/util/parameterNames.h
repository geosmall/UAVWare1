// Version signature
const char*	parameterName_version	=	"version"	;	// Byte stamp ID to identify version and already setup
					
					
// Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
const char*	parameterName_channel_1_fs	=	"channel_1_fs"	;	// thro
const char*	parameterName_channel_2_fs	=	"channel_2_fs"	;	// ail
const char*	parameterName_channel_3_fs	=	"channel_3_fs"	;	// elev
const char*	parameterName_channel_4_fs	=	"channel_4_fs"	;	// rudd
const char*	parameterName_channel_5_fs	=	"channel_5_fs"	;	// gear, greater than 1500 = throttle cut
const char*	parameterName_channel_6_fs	=	"channel_6_fs"	;	// aux1
					
// Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
const char*	parameterName_B_madgwick	=	"B_madgwick"	;	// Madgwick filter parameter
const char*	parameterName_B_accel	=	"B_accel"	;	// Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
const char*	parameterName_B_gyro	=	"B_gyro"	;	// Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
const char*	parameterName_B_mag	=	"B_mag"	;	// Magnetometer LP filter parameter
					
// Magnetometer calibration parameters - if using MPU9250, uncomment calibrateMagnetometer() in void setup() to get these values, else just ignore these
const char*	parameterName_MagErrorX	=	"MagErrorX"	;	
const char*	parameterName_MagErrorY	=	"MagErrorY"	;	
const char*	parameterName_MagErrorZ	=	"MagErrorZ"	;	
const char*	parameterName_MagScaleX	=	"MagScaleX"	;	
const char*	parameterName_MagScaleY	=	"MagScaleY"	;	
const char*	parameterName_MagScaleZ	=	"MagScaleZ"	;	
					
// Controller parameters (take note of defaults before modifying!):
const char*	parameterName_i_limit	=	"i_limit"	;	// Integrator saturation level, mostly for safety (default 25.0)
const char*	parameterName_maxRoll	=	"maxRoll"	;	// Max roll angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
const char*	parameterName_maxPitch	=	"maxPitch"	;	// Max pitch angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
const char*	parameterName_maxYaw 	=	"maxYaw "	;	// Max yaw rate in deg/sec
					
const char*	parameterName_Kp_roll_angle	=	"Kp_roll_angle"	;	// Roll P-gain - angle mode
const char*	parameterName_Ki_roll_angle	=	"Ki_roll_angle"	;	// Roll I-gain - angle mode
const char*	parameterName_Kd_roll_angle	=	"Kd_roll_angle"	;	// Roll D-gain - angle mode (if using controlANGLE2(), set to 0.0)
const char*	parameterName_B_loop_roll	=	"B_loop_roll"	;	// Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
const char*	parameterName_Kp_pitch_angle	=	"Kp_pitch_angle"	;	// Pitch P-gain - angle mode
const char*	parameterName_Ki_pitch_angle	=	"Ki_pitch_angle"	;	// Pitch I-gain - angle mode
const char*	parameterName_Kd_pitch_angle	=	"Kd_pitch_angle"	;	// Pitch D-gain - angle mode (if using controlANGLE2(), set to 0.0)
const char*	parameterName_B_loop_pitch	=	"B_loop_pitch"	;	// Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
					
const char*	parameterName_Kp_roll_rate	=	"Kp_roll_rate"	;	// Roll P-gain - rate mode
const char*	parameterName_Ki_roll_rate	=	"Ki_roll_rate"	;	// Roll I-gain - rate mode
const char*	parameterName_Kd_roll_rate	=	"Kd_roll_rate"	;	// Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
const char*	parameterName_Kp_pitch_rate	=	"Kp_pitch_rate"	;	// Pitch P-gain - rate mode
const char*	parameterName_Ki_pitch_rate	=	"Ki_pitch_rate"	;	// Pitch I-gain - rate mode
const char*	parameterName_Kd_pitch_rate	=	"Kd_pitch_rate"	;	// Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
					
const char*	parameterName_Kp_yaw	=	"Kp_yaw"	;	// Yaw P-gain
const char*	parameterName_Ki_yaw	=	"Ki_yaw"	;	// Yaw I-gain
const char*	parameterName_Kd_yaw	=	"Kd_yaw"	;	// Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)

