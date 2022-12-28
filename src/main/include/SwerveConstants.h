//Find offsets with bevel gears on the right side of the drive train
#define FL_WHEEL_OFFSET 0.867   
#define FR_WHEEL_OFFSET 0.443
#define BR_WHEEL_OFFSET 0.109
#define BL_WHEEL_OFFSET 0.515

//Assuming a rectangular drive train (input distance between center of wheels)
#define DRIVE_LENGTH 24
#define DRIVE_WIDTH 24

//Encoder constants
#define DRIVE_MOTOR_GEAR_RATIO 6.54
#define DRIVE_MOTOR_CIRCUMFERENCE 0.10322 * M_PI
#define SPIN_MOTOR_GEAR_RATIO 1 // TODO

//Numerical Constants TODO CHANGE
#define SWERVE_DRIVE_MAX_MPS 5.556
#define SWERVE_DRIVE_MAX_ACCELERATION 3 //TODO
#define MAX_RADIAN_PER_SECOND 4 // TODO
#define MAX_RAIDAN_PER_SECOND_SQ 3 // TODO

//PID VALUES
#define WHEEL_SPIN_KP 0.6
#define X_KP 3
#define Y_KP 3
#define THETA_KP 3