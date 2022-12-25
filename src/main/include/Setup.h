#pragma once

#include "Robot.h"

using namespace std;
using namespace frc;
using namespace frc2;

int fl, fr, bl, br;

PS4Controller *cont_Driver = new PS4Controller(0);

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

Translation2d m_frontLeft{0.5388_m, 0.5388_m};
Translation2d m_frontRight{0.5388_m, -0.5388_m};
Translation2d m_backLeft{-0.5388_m, 0.5388_m};
Translation2d m_backRight{-0.5388_m, -0.5388_m};

SwerveDriveKinematics<4> m_kinematics{m_frontLeft, m_frontRight, m_backLeft, m_backRight};

ChassisSpeeds speeds{1_mps, 3_mps, 1.5_rad_per_s};

//Change the number value to the port
Pigeon2 _pigeon(6);

double thetaInit;
#define FL_WHEEL_OFFSET 0.952   
#define FR_WHEEL_OFFSET 0.395
#define BR_WHEEL_OFFSET 0.057
#define BL_WHEEL_OFFSET 0.429
#define DRIVE_LENGTH 24
#define DRIVE_WIDTH 24
#define MAX_SPIN_SPEED 0.5
#define MAX_DRIVE_SPEED 0.5
// auto [fl,fr,bl,br] = m_kinematics.ToSwerveModuleStates(speeds);