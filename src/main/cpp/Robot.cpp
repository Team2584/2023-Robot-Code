// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "Setup.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

// Start with target = 0
double pigeon_initial;
double FLtarget_pos = 0;
double FRtarget_pos = 0;
double target_pos = 0;

double FR_Target_Angle = 0;
double FL_Target_Angle = 0;
double BL_Target_Angle = 0;
double BR_Target_Angle = 0;

void Robot::RobotInit()
{
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  // FLMagEnc.SetDutyCycleRange(0, 4095/4096);
  // FLMagEnc.Reset();
  // FRMagEnc.Reset();
  // BLMagEnc.Reset();
  // BRMagEnc.Reset();

  FLMagEnc.SetConnectedFrequencyThreshold(10);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit()
{
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom)
  {
    // Custom Auto goes here
  }
  else
  {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic()
{
  if (m_autoSelected == kAutoNameCustom)
  {
    // Custom Auto goes here
  }
  else
  {
    // Default Auto goes here
  }
}

void Robot::TeleopInit()
{
  //REMOVE THIS BEFORE COMPETITION
  pigeon_initial = _pigeon.GetYaw();
  thetaInit = FLMagEnc.GetAbsolutePosition() * 360;
  frc::SmartDashboard::PutNumber("Target", 0);
}

int sgn(double x)
{
  if (x > 0)
  {
    return 1;
  }
  if (x < 0)
  {
    return -1;
  }
  return 0;
}

double EncoderReadingToAngle(double reading, double offset)
{
  // subtract the encoder offset to make 0 degrees forward
  reading -= offset;
  if (reading < 0)
    reading += 1;
  // Flip the degrees to make clockwise positive
  reading = 1 - reading;
  // Convert from 0-1 to degrees
  reading *= 360;
  return reading;
}

double ControllerAxisToAngle(double xAxis, double yAxis)
{
  double controllerAngle = atan2(yAxis, xAxis);

  if (xAxis > 0 && yAxis > 0)
  {
    controllerAngle = (M_PI / 2) - atan2(yAxis, xAxis);
  }
  else if (xAxis > 0 && yAxis < 0)
  {
    controllerAngle = M_PI / 2 - atan2(yAxis, xAxis);
  }
  else if (xAxis < 0 && yAxis < 0)
  {
    controllerAngle = M_PI / 2 - atan2(yAxis, xAxis);
  }
  else if (xAxis < 0 && yAxis > 0)
  {
    controllerAngle = 5 * M_PI / 2 - atan2(yAxis, xAxis);
  }
  else if (xAxis == 0 && yAxis < 0)
  {
    return 180;
  }
  else if (xAxis == 0 && yAxis > 0)
  {
    return 0;
  }
  else if (yAxis == 0 && xAxis < 0)
  {
    return 270;
  }
  else if (yAxis == 0 && xAxis > 0)
  {
    return 90;
  }
  else
  {
    printf("Error\n");
  }

  // Convert angle to degrees
  controllerAngle *= 180 / M_PI;

  return controllerAngle;
}

// Input a distance to target and a direction, updates arr with rotation speed of wheel using PID
// arr[0] = wheel rotation speed, arr[1] = wheel rotation direction, arr[2] = wheel spin direction
void swervePID(double distanceToTarget, int direction, double arr[])
{
  double kP = MAX_SPIN_SPEED;
  // output = kp * distance / maxDistance
  double output = kP * (distanceToTarget / 90);
  arr[1] = direction;
  arr[0] = output;
}

// Input a current position and a target, updates arr with optimal wheel speeds and rotation
// arr[0] = wheel rotation speed, arr[1] = wheel rotation direction, arr[2] = wheel spin direction
void swerveWheel(double wheel_position, double wheel_target, double arr[])
{
  double driveDirection = 1;

  if (wheel_target < 0)
  {
    wheel_target += 360;
  }

  // Ask me about the logic if you want to understand it.
  // Basically this has 8 seperate scanrios, 
  // Either your wheel position is less than or greater than your target
  // And for each of those 2 scenarios, there are 4 possible rotation directions
  if (wheel_position < wheel_target)
  {
    if (wheel_target - wheel_position <= 180)
    {
      if (wheel_target - wheel_position <= 90)
      {
        swervePID(wheel_target - wheel_position, 1, arr);
      }
      else
      {
        swervePID(180 - (wheel_target - wheel_position), -1, arr);
        driveDirection = -1;
      }
    }
    else
    {
      if (wheel_target - wheel_position <= 270)
      {
        swervePID((wheel_target - wheel_position) - 180, 1, arr);
        driveDirection = -1;
      }
      else
      {
        swervePID(360 - (wheel_target - wheel_position), -1, arr);
      }
    }
  }
  else if (wheel_position > wheel_target)
  {
    if (wheel_position - wheel_target <= 180)
    {
      if (wheel_position - wheel_target <= 90)
      {
        swervePID(wheel_position - wheel_target, -1, arr);
      }
      else
      {
        swervePID(180 - (wheel_position - wheel_target), 1, arr);
        driveDirection = -1;
      }
    }
    else
    {
      if (wheel_position - wheel_target <= 270)
      {
        swervePID((wheel_position - wheel_target) - 180, -1, arr);
        driveDirection = -1;
      }
      else
      {
        swervePID(360 - (wheel_position - wheel_target), 1, arr);
      }
    }
  }
  else
  {
    swerveFL.Set(ControlMode::PercentOutput, 0);
  }

  arr[2] = driveDirection;
}

void Robot::TeleopPeriodic()
{

  // Find current position of wheel in degrees, 0 is forward and degrees increase clockwise
  double FL_current_pos = EncoderReadingToAngle(FLMagEnc.GetAbsolutePosition(), FL_WHEEL_OFFSET);
  double FR_current_pos = EncoderReadingToAngle(FRMagEnc.GetAbsolutePosition(), FR_WHEEL_OFFSET);
  double BR_current_pos = EncoderReadingToAngle(BRMagEnc.GetAbsolutePosition(), BR_WHEEL_OFFSET);
  double BL_current_pos = EncoderReadingToAngle(BLMagEnc.GetAbsolutePosition(), BL_WHEEL_OFFSET);

  // Print for debugging purposes
  SmartDashboard::PutNumber("FL Pos:", FLMagEnc.GetAbsolutePosition());
  SmartDashboard::PutNumber("FR Pos:", FRMagEnc.GetAbsolutePosition());
  SmartDashboard::PutNumber("BR Pos:", BRMagEnc.GetAbsolutePosition());
  SmartDashboard::PutNumber("BL Pos:", BLMagEnc.GetAbsolutePosition());

  // Find controller input
  double joy_lStick_Y = cont_Driver->GetLeftY(), joy_lStick_X = cont_Driver->GetLeftX(),
         joy_rStick_X = cont_Driver->GetRightX();
  joy_lStick_Y *= -1;

  // Remove ghost movement by making sure joystick is moved a certain amount
  double joy_lStick_distance = sqrt(pow(joy_lStick_X, 2.0) + pow(joy_lStick_Y, 2.0));
  double joystick_deadband = 0.05;

  if (joy_lStick_distance < joystick_deadband)
  {
    joy_lStick_X = 0;
    joy_lStick_Y = 0;
  }

  if (abs(joy_rStick_X) < joystick_deadband)
  {
    joy_rStick_X = 0;
  }

  // Find Pigeon IMU Angle TODO
  double pigeon_angle = fmod(_pigeon.GetYaw(), 360);
  pigeon_angle -= pigeon_initial;
  if (pigeon_angle < 0) 
    pigeon_angle += 360;
  pigeon_angle = 360 - pigeon_angle;
  if (pigeon_angle == 360)    
    pigeon_angle = 0;
  pigeon_angle *= M_PI / 180;

  SmartDashboard::PutNumber("Pigeon Angle:", pigeon_angle);

  SmartDashboard::PutNumber("Y Joystick:", joy_lStick_Y);
  SmartDashboard::PutNumber("X Joystick:", joy_lStick_X);

  // Use pigion_angle to determine what our target movement vector is in relation to the robot
  double FWD_Drive_Speed = joy_lStick_Y * cos(pigeon_angle) + joy_lStick_X * sin(pigeon_angle);
  double STRAFE_Drive_Speed = -1 * joy_lStick_Y * sin(pigeon_angle) + joy_lStick_X * cos(pigeon_angle);
  double Turn_Speed = joy_rStick_X;

  SmartDashboard::PutNumber("FWD_Drive_Speed:", FWD_Drive_Speed);
  SmartDashboard::PutNumber("STRAFE_Drive_Speed:", STRAFE_Drive_Speed);
  SmartDashboard::PutNumber("Turn_Speed:", Turn_Speed);

  // Determine wheel speeds / wheel target positions
  // Equations explained at:
  // https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
  // After clicking above link press the top download to see how the equations work
  double DRIVE_RADIUS = sqrt(pow(DRIVE_LENGTH, 2) + pow(DRIVE_WIDTH, 2));

  double A = STRAFE_Drive_Speed - Turn_Speed * (DRIVE_LENGTH / DRIVE_RADIUS);
  double B = STRAFE_Drive_Speed + Turn_Speed * (DRIVE_LENGTH / DRIVE_RADIUS);
  double C = FWD_Drive_Speed - Turn_Speed * (DRIVE_LENGTH / DRIVE_RADIUS);
  double D = FWD_Drive_Speed + Turn_Speed * (DRIVE_LENGTH / DRIVE_RADIUS);



  if (!(joy_lStick_X == 0 && joy_lStick_Y == 0 && joy_rStick_X == 0))
  {
      FR_Target_Angle = atan2(B, C) * 180 / M_PI;
      FL_Target_Angle = atan2(B, D) * 180 / M_PI;
      BL_Target_Angle = atan2(A, D) * 180 / M_PI;
      BR_Target_Angle = atan2(A, C) * 180 / M_PI;
  }

  SmartDashboard::PutNumber("FR_ANGLE:", FR_Target_Angle);
  SmartDashboard::PutNumber("FL_ANGLE:", FL_Target_Angle);
  SmartDashboard::PutNumber("BL_ANGLE:", BL_Target_Angle);
  SmartDashboard::PutNumber("BR_ANGLE:", BR_Target_Angle);


  //Do not change target angle if joystick values are 0
  double FR_Drive_Speed = sqrt(pow(B, 2) + pow(C, 2));
  double FL_Drive_Speed = sqrt(pow(B, 2) + pow(D, 2));
  double BL_Drive_Speed = sqrt(pow(A, 2) + pow(D, 2));
  double BR_Drive_Speed = sqrt(pow(A, 2) + pow(C, 2));

  //Above function makes wheel speeds correct in relation to one another, but not at the right values
  //Below we are scaling the wheel speeds down to have a max of 1

  double max = FR_Drive_Speed;
  if (FL_Drive_Speed > max)
    max = FL_Drive_Speed;
  if (BL_Drive_Speed > max)
    max = BL_Drive_Speed;
  if (BR_Drive_Speed > max)
    max = BR_Drive_Speed;
  

  if (max > 1)
  {
    FL_Drive_Speed /= max;
    FR_Drive_Speed /= max;
    BL_Drive_Speed /= max;
    BR_Drive_Speed /= max;
  }

  FL_Drive_Speed *= MAX_DRIVE_SPEED;
  BL_Drive_Speed *= MAX_DRIVE_SPEED;
  FR_Drive_Speed *= MAX_DRIVE_SPEED;
  BR_Drive_Speed *= MAX_DRIVE_SPEED;
  
  // Rotate and Spin Wheels to desired target at desired speed
  // arr[0] = wheel rotation speed, arr[1] = wheel rotation direction, arr[2] = wheel spin direction
  // An array is my way for a function to return multiple values, sorry
  double FLarr[3];
  // Set the values in the array
  swerveWheel(FL_current_pos, FL_Target_Angle, FLarr);
  // Set the drive speeds based on the returned values
  swerveFL.Set(ControlMode::PercentOutput, FLarr[0] * FLarr[1]);
  driveFL.Set(ControlMode::PercentOutput, FL_Drive_Speed * FLarr[2] * MAX_DRIVE_SPEED);

  double FRarr[3];
  swerveWheel(FR_current_pos, FR_Target_Angle, FRarr);
  swerveFR.Set(ControlMode::PercentOutput, FRarr[0] * FRarr[1]);
  driveFR.Set(ControlMode::PercentOutput, FR_Drive_Speed * FRarr[2] * MAX_DRIVE_SPEED);

  double BLarr[3];
  swerveWheel(BL_current_pos, BL_Target_Angle, BLarr);
  swerveBL.Set(ControlMode::PercentOutput, BLarr[0] * BLarr[1]);
  driveBL.Set(ControlMode::PercentOutput, BL_Drive_Speed * BLarr[2] * MAX_DRIVE_SPEED);

  double BRarr[2];
  swerveWheel(BR_current_pos, BR_Target_Angle, BRarr);
  swerveBR.Set(ControlMode::PercentOutput, BRarr[0] * BRarr[1]);
  driveBR.Set(ControlMode::PercentOutput, BR_Drive_Speed * BRarr[2] * MAX_DRIVE_SPEED);
}

void Robot::DisabledInit()
{
}

void Robot::DisabledPeriodic()
{
}

void Robot::TestInit()
{
}

void Robot::TestPeriodic()
{
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
