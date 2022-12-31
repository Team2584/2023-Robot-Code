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
#define X_KP 5
#define X_KD 0
#define Y_KP 5
#define Y_KD 0
#define THETA_KP 2
#define THETA_KD 0

//PID VALUES FOR DRIVE USING ONLY VISION
#define V_X_KP 5
#define V_X_KD 0
#define V_Y_KP 5
#define V_Y_KD 0
#define V_THETA_KP 2
#define V_THETA_KD 0

//RAMP UP VALUES
#define AUTO_MAX_MPS 3_mps
#define AUTO_MAX_MPS_SQ 3_mps_sq
#define AUTO_MAX_RADPS 0.5_mps
#define AUTO_MAX_RADPS_SQ 1_mps_sq


