#pragma once

#include "Robot.h"

using namespace std;
using namespace frc;
using namespace frc2;

int fl,fr,bl,br;

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

 
Translation2d m_frontLeft{0.5388_m ,0.5388_m};
Translation2d m_frontRight{0.5388_m ,-0.5388_m};
Translation2d m_backLeft{-0.5388_m ,0.5388_m};
Translation2d m_backRight{-0.5388_m ,-0.5388_m};

SwerveDriveKinematics<4> m_kinematics{m_frontLeft, m_frontRight, m_backLeft, m_backRight};

ChassisSpeeds speeds{1_mps, 3_mps, 1.5_rad_per_s};


double thetaInit;
#define TEST_WHEEL_OFFSET 0.75
//auto [fl,fr,bl,br] = m_kinematics.ToSwerveModuleStates(speeds);

