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

//Customization Variabes
#define CONTROLLER_DEADBAND 0.15
#define MAX_SPIN_SPEED 0.2
#define MAX_DRIVE_SPEED 0.2
#define MAX_DRIVE_ACCLERATION 0.3_mps_sq
#define STARTING_DRIVE_HEADING 0
#define CONTROLLER_TYPE 1

