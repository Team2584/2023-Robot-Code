#pragma once

#include "Robot.h"

using namespace std;
using namespace frc;
using namespace frc2;

Orchestra orchestra;

PS4Controller *cont_Driver = new PS4Controller(0);
XboxController *xbox_Drive = new XboxController(0);

TalonFX swerveFL{11};
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
//Find offsets with bevel gears on the right side of the drive train
#define FL_WHEEL_OFFSET 0.867   
#define FR_WHEEL_OFFSET 0.443
#define BR_WHEEL_OFFSET 0.109
#define BL_WHEEL_OFFSET 0.515
#define DRIVE_LENGTH 24
#define DRIVE_WIDTH 24

//Customization Variabes
#define MAX_SPIN_SPEED 0.2
#define MAX_DRIVE_SPEED 0.2
#define STARTING_DRIVE_HEADING 0
#define CONTROLLER_TYPE 1


