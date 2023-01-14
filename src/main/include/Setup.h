#pragma once

#include "Robot.h"

using namespace std;
using namespace frc;
using namespace frc2;

Orchestra orchestra;

PS4Controller *cont_Driver = new PS4Controller(0);
XboxController *xbox_Drive = new XboxController(0);

TalonFX swerveFL{11};
frc::PWMSparkMax lights{9};
TalonFX driveFL{01};
TalonFX swerveFR{12};
TalonFX driveFR{02};
TalonFX swerveBL{13};
TalonFX driveBL{03};
TalonFX swerveBR{14};
TalonFX driveBR{04};

DutyCycleEncoder FLMagEnc(3);
DutyCycleEncoder FRMagEnc(2);
DutyCycleEncoder BLMagEnc(1);
DutyCycleEncoder BRMagEnc(0);

//Change the number value to the port
Pigeon2 _pigeon(6);

double thetaInit;

//Customization Variabes (all in percent power so the driver's weak brain don't get confused)
#define CONTROLLER_DEADBAND 0.15
#define MAX_SPIN_SPEED 0.4
#define MAX_DRIVE_SPEED 0.4
#define MAX_DRIVE_ACCELERATION 1.5 //max change in percent per second
#define MAX_SPIN_ACCELERATION 1.5
#define STARTING_DRIVE_HEADING 0
#define CONTROLLER_TYPE 1

//PID FOR WHILE DRIVING TURN TO POINT
#define TURN_TO_POINT_ALLOWABLE_ERROR 0.05
#define TURN_TO_POINT_MAX_SPIN 0.2
#define TURN_TO_POINT_MAX_ACCEL 0.7
#define TURN_TO_TO_POINT_P 0.575
#define TURN_TO_TO_POINT_I 0
#define TURN_TO_TO_POINT_I_MAX 0.1


