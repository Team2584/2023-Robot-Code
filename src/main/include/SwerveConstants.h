//Find offsets with bevel gears on the right side of the drive train
#define FL_WHEEL_OFFSET 0.867   
#define FR_WHEEL_OFFSET 0.443
#define BR_WHEEL_OFFSET 0.109
#define BL_WHEEL_OFFSET 0.515

//Assuming a rectangular drive train (input distance between center of wheels)
#define DRIVE_LENGTH 0.29845 * 2
#define DRIVE_WIDTH 0.2953 * 2

//Encoder constants
#define DRIVE_MOTOR_GEAR_RATIO 6.54 
#define DRIVE_MOTOR_CIRCUMFERENCE 0.10322 * M_PI
#define SPIN_MOTOR_GEAR_RATIO 15.438

//Numerical Constants TODO CHANGE
#define SWERVE_DRIVE_MAX_MPS 5.556
#define SWERVE_DRIVE_MAX_ACCELERATION 3 //TODO
#define MAX_RADIAN_PER_SECOND 4 // TODO
#define WHEEL_SPIN_KP 0.6

//PID VALUES FOR DRIVE TO POSE
#define TRANSLATION_KP 0.7    //old is 1.45
#define TRANSLATION_KI 0
#define TRANSLATION_KI_MAX 0.1
#define TRANSLATION_MAX_SPEED 0.15
#define TRANSLATION_MAX_ACCEL 0.5
#define SPIN_KP 0.3  //old is 0.7
#define SPIN_KI 0
#define SPIN_KI_MAX 0.1
#define SPIN_MAX_SPEED 0.2
#define SPIN_MAX_ACCEL 0.7  
#define ALLOWABLE_ERROR_TRANSLATION 0.03
#define ALLOWABLE_ERROR_ROTATION 0.05

//Conbined Pose Estimation Values
#define ODOMETRY_REFRESH_TIME 1 //seconds



