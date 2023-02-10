#pragma once

#include "Robot.h"

using namespace std;
using namespace frc;
using namespace frc2;

Orchestra orchestra;

PS4Controller *cont_Driver = new PS4Controller(0);
XboxController *xbox_Drive = new XboxController(0);

rev::CANSparkMax swerveFL{11, rev::CANSparkMax::MotorType::kBrushless};
TalonFX driveFL{01};
rev::CANSparkMax swerveFR{12, rev::CANSparkMax::MotorType::kBrushless};
TalonFX driveFR{02};
rev::CANSparkMax swerveBL{13, rev::CANSparkMax::MotorType::kBrushless};
TalonFX driveBL{03};
rev::CANSparkMax swerveBR{14, rev::CANSparkMax::MotorType::kBrushless};
TalonFX driveBR{04};
rev::CANSparkMax winchR{5, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax winchL{6, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax wrist{7, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax clawM1{9, rev::CANSparkMax::MotorType::kBrushed};

DutyCycleEncoder FLMagEnc(8);
DutyCycleEncoder FRMagEnc(6);
DutyCycleEncoder BLMagEnc(9);
DutyCycleEncoder BRMagEnc(7);

Pigeon2 _pigeon(6);

TimeOfFlight TOFSensor(0);

double thetaInit;

//Customization Variabes (all in percent power so the driver's weak brain don't get confused)
#define CONTROLLER_DEADBAND 0.2
#define MAX_DRIVE_ACCELERATION 1    //max change in percent per second
#define MAX_SPIN_ACCELERATION 1
#define MAX_ELEV_ACCELERATION 3
#define STARTING_DRIVE_HEADING 0
#define CONTROLLER_TYPE 1

//PID FOR WHILE DRIVING TURN TO POINT
#define TURN_TO_POINT_ALLOWABLE_ERROR 0.05
#define TURN_TO_POINT_MAX_SPIN 0.2
#define TURN_TO_POINT_MAX_ACCEL 0.7
#define TURN_TO_TO_POINT_P 0.575
#define TURN_TO_TO_POINT_I 0
#define TURN_TO_TO_POINT_I_MAX 0.1


