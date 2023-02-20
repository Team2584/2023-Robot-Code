//Find offsets with bevel gears on the right side of the drive train
#define FL_WHEEL_OFFSET 0.474 // FB 0.11475   
#define FR_WHEEL_OFFSET 0.426 // FB 0.082
#define BR_WHEEL_OFFSET 0.865 // FB 0.841
#define BL_WHEEL_OFFSET 0.147 // FB 0.0312

//Assuming a rectangular drive train (input distance between center of wheels)
#define DRIVE_LENGTH 0.5906_m 
#define DRIVE_WIDTH 0.489_m 

//Encoder constants
#define DRIVE_MOTOR_GEAR_RATIO 7.36 
#define DRIVE_MOTOR_CIRCUMFERENCE 0.10469983 * M_PI
#define SPIN_MOTOR_GEAR_RATIO 15.43

//Numerical Constants TODO CHANGE
#define SWERVE_DRIVE_MAX_MPS 4.247 // unneccesary
#define SWERVE_DRIVE_MAX_ACCELERATION 3 // unneccesary
#define MAX_RADIAN_PER_SECOND 4 // unneccesary

// Swerve Module Wheel Spin PID Values
#define WHEEL_SPIN_KP 1
#define WHEEL_SPIN_KI 0
#define WHEEL_SPIN_KI_MAX 0.03

//PID VALUES FOR DRIVE TO POSE ODOMETRY
#define O_TRANSLATION_KP 1.1
#define O_TRANSLATION_KI 0.02
#define O_TRANSLATION_KI_MAX 0.03
#define O_TRANSLATION_MAX_SPEED 0.2
#define O_TRANSLATION_MAX_ACCEL 3
#define O_SPIN_KP 0.5
#define O_SPIN_KI 0.01
#define O_SPIN_KI_MAX 0.05
#define O_SPIN_MAX_SPEED 0.4
#define O_SPIN_MAX_ACCEL 1.5  
#define O_ALLOWABLE_ERROR_TRANSLATION 0.05
#define O_ALLOWABLE_ERROR_ROTATION 0.05

//PID VALUES FOR SPLINE
#define S_TRANSLATION_KP 6
#define S_TRANSLATION_KI 0
#define S_TRANSLATION_KD 0
#define S_TRANSLATION_KI_MAX 0.1
#define S_TRANSLATION_MAX_SPEED 1.0
#define S_TRANSLATION_MAX_ACCEL 1.5
#define S_SPIN_KP 4
#define S_SPIN_KI 0
#define S_SPIN_KI_MAX 0.1
#define S_SPIN_KD 0
#define S_SPIN_MAX_SPEED 4.0
#define S_SPIN_MAX_ACCEL 1.5
#define S_ALLOWABLE_ERROR_TRANSLATION 0.05
#define S_ALLOWABLE_ERROR_ROTATION 0.1

//PID VALUES FOR TURN TO Pole WITH LIMELIGHT
#define P_STRAFE_KP 0.6
#define P_STRAFE_KI 0.007
#define P_STRAFE_KI_MAX 0.05
#define P_STRAFE_MAX_SPEED 0.3
#define P_STRAFE_MAX_ACCEL 3
#define P_ALLOWABLE_ERROR_STRAFE 0.05 // in pixel values from -1 to 1
#define P_TRANS_KP 3.5
#define P_TRANS_KI 0.05 
#define P_TRANS_KI_MAX 0.05
#define P_TRANS_MAX_SPEED 0.2
#define P_TRANS_MAX_ACCEL 3 
#define P_ALLOWABLE_ERROR_TRANS 0.01
#define P_SPIN_KP 2.7
#define P_SPIN_KI 0
#define P_SPIN_KI_MAX 0.1
#define P_SPIN_MAX_SPEED 0.2
#define P_SPIN_MAX_ACCEL 0.6
#define P_ALLOWABLE_ERROR_ROTATION 0.05 // in pixel values from -1 to 1


// Going to an April Tag
#define A_ALLOWABLE_ERROR_TRANSLATION 0.05
#define A_TRANSLATION_KP 1.1
#define A_TRANSLATION_KI 0.02
#define A_TRANSLATION_KI_MAX 0.03
#define A_TRANSLATION_MAX_SPEED 0.3
#define A_TRANSLATION_MAX_ACCEL 3
// #define A_STRAFE_KP 0.7
// #define A_STRAFE_KI 0
// #define A_STRAFE_KI_MAX 0.03
// #define A_STRAFE_MAX_SPEED 0.2
// #define A_STRAFE_MAX_ACCEL 3

// Grabbing a Cone
#define C_ALLOWABLE_ERROR_TRANSLATION 0.05
#define C_TRANSLATION_KP 1.1
#define C_TRANSLATION_KI 0.02
#define C_TRANSLATION_KI_MAX 0.03
#define C_TRANSLATION_MAX_SPEED 0.3
#define C_TRANSLATION_MAX_ACCEL 3
#define C_STRAFE_KP 0.7
#define C_STRAFE_KI 0
#define C_STRAFE_KI_MAX 0.03
#define C_STRAFE_MAX_SPEED 0.2
#define C_STRAFE_MAX_ACCEL 3
#define C_SPIN_KP 0.6
#define C_SPIN_KI 0.01
#define C_SPIN_KD 0.01
#define C_SPIN_KI_MAX 0.05
#define C_SPIN_MAX_SPEED 0.4
#define C_SPIN_MAX_ACCEL 3
#define C_ALLOWABLE_ERROR_ROTATION 0.03
