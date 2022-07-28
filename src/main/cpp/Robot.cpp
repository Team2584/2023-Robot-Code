// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "Setup.h"

#include <fmt/core.h>


#include <frc/smartdashboard/SmartDashboard.h>

// Start with target = 0
double target_pos = 0;

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  //FLMagEnc.SetDutyCycleRange(0, 4095/4096);
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
void Robot::RobotPeriodic() {}

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
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
  thetaInit = FLMagEnc.GetAbsolutePosition() * 360;
  frc::SmartDashboard::PutNumber("Target", 0);
}

int sgn(double x){
  if(x > 0){
    return 1;
  }
  if(x < 0){
    return -1;
  }
  return 0;
}

double EncoderReadingToAngle(double reading, double offset)
{
  reading -= offset;
  if (reading < 0)
    reading += 1;  
  reading = 1 - reading;
  reading *= 360;
  return reading;
}

double ControllerAxisToAngle(double xAxis, double yAxis)
{
  double controllerAngle = atan2(yAxis , xAxis);


  if (xAxis > 0 && yAxis > 0){
    controllerAngle = (M_PI / 2) - atan2(yAxis, xAxis);
  }
  else if (xAxis > 0 && yAxis < 0){
    controllerAngle = M_PI / 2 - atan2(yAxis, xAxis);
  }
  else if (xAxis < 0 && yAxis < 0){
    controllerAngle = M_PI / 2 - atan2(yAxis,  xAxis);
  }
  else if (xAxis < 0 && yAxis > 0){
    controllerAngle = 5 * M_PI / 2 - atan2(yAxis, xAxis);
  }
  else if (xAxis == 0 && yAxis < 0){
    return 180;
  }
  else if (xAxis == 0 && yAxis > 0){
    return 0;
  }
  else if (yAxis == 0 && xAxis < 0){
    return 270;
  }
  else if (yAxis == 0 && xAxis > 0){
    return 90;
  }
  else {
    printf("Error\n");
  }

  // Convert angle to degrees
  controllerAngle *= 180 / M_PI;

  return controllerAngle;
}

void swervePID (double distanceToTarget, int direction, double arr[]){
  double kP=0.5;
  //output = kp * distance / maxDistance
  double output = kP * (distanceToTarget/90);
  arr[1] = direction;
  arr[0] = output;
}

// arr[0] = wheel rotation speed, arr[1] = wheel rotation direction, arr[2] = wheel spin direction
void swerveWheel (double wheel_position, double wheel_target, double arr[])
{
  double driveDirection = 1;

  if (wheel_position < wheel_target) {
    if (wheel_target - wheel_position  <= 180) {
      if (wheel_target - wheel_position <= 90) {
        swervePID(wheel_target - wheel_position, 1, arr);
      }
      else {
        swervePID(180 - (wheel_target - wheel_position), -1, arr);
        driveDirection = -1;
      }
    }
    else {
      if (wheel_target - wheel_position <= 270) {
        swervePID((wheel_target - wheel_position) - 180, 1, arr);
        driveDirection = -1;
      }
      else {
        swervePID(360 - (wheel_target - wheel_position), -1, arr);
      }
    }
  }
  else if (wheel_position > wheel_target){
    if (wheel_position - wheel_target <= 180) {
      if (wheel_position - wheel_target <= 90) {
        swervePID(wheel_position - wheel_target, -1, arr);
      }
      else
      {
        swervePID(180 - (wheel_position - wheel_target), 1, arr);
        driveDirection = -1;
      }
    }
    else {
      if (wheel_position - wheel_target <= 270) {
        swervePID((wheel_position - wheel_target) - 180, -1, arr);
        driveDirection = -1;
      }
      else {
        swervePID(360 - (wheel_position - wheel_target), 1, arr);
      }
    }
  }
  else {
        swerveFL.Set(ControlMode::PercentOutput, 0);          
  }

  arr[2] = driveDirection;
}

void Robot::TeleopPeriodic(){

  // Find current position in degrees, 0 is roughly forward
  double FL_current_pos = EncoderReadingToAngle(FLMagEnc.GetAbsolutePosition(), FL_WHEEL_OFFSET);
  double FR_current_pos = EncoderReadingToAngle(FRMagEnc.GetAbsolutePosition(), FR_WHEEL_OFFSET);
  double BR_current_pos = EncoderReadingToAngle(BRMagEnc.GetAbsolutePosition(), BR_WHEEL_OFFSET);
  double BL_current_pos = EncoderReadingToAngle(BLMagEnc.GetAbsolutePosition(), BL_WHEEL_OFFSET);

  SmartDashboard::PutNumber ("FR Pos:", FRMagEnc.GetAbsolutePosition());
  SmartDashboard::PutNumber ("BR Pos:", BRMagEnc.GetAbsolutePosition());
  SmartDashboard::PutNumber ("BL Pos:", BLMagEnc.GetAbsolutePosition());

  //Find target from controller
  double joy_lStick_Y = cont_Driver->GetLeftY(), joy_lStick_X = cont_Driver->GetLeftX(), joy_rStick_X = cont_Driver->GetRightX();
  joy_lStick_Y *= -1;

  double joy_lStick_dist_deadband = 0.05;
  
  //Find Controller Angle
  double joy_lStick_distance = sqrt(pow(joy_lStick_X, 2.0) + pow(joy_lStick_Y,2.0));

  // Target in degrees
  if (joy_lStick_distance > joy_lStick_dist_deadband)
    target_pos = ControllerAxisToAngle(joy_lStick_X, joy_lStick_Y);

  SmartDashboard::PutNumber ("target position:", target_pos);

  double driveMotorPower = 0.5 * joy_lStick_distance;

  double FLarr[3];
  swerveWheel(FL_current_pos, target_pos, FLarr);
  swerveFL.Set(ControlMode::PercentOutput, FLarr[0] * FLarr[1]);
  driveFL.Set(ControlMode::PercentOutput, driveMotorPower * FLarr[2]);


  double FRarr[3];
  swerveWheel(FR_current_pos, target_pos, FRarr);
  swerveFR.Set(ControlMode::PercentOutput, FRarr[0] * FRarr[1]);
  driveFR.Set(ControlMode::PercentOutput, driveMotorPower * FRarr[2] * -1);

  double BLarr[3];
  swerveWheel(BL_current_pos, target_pos, BLarr);
  swerveBL.Set(ControlMode::PercentOutput, BLarr[0] * BLarr[1]);
  driveBL.Set(ControlMode::PercentOutput, driveMotorPower * BLarr[2]);

  double BRarr[2];
  swerveWheel(BR_current_pos, target_pos, BRarr);
  swerveBR.Set(ControlMode::PercentOutput, BRarr[0] * BRarr[1]);
  driveBR.Set(ControlMode::PercentOutput, driveMotorPower * BRarr[2]);
}


/*
void Robot::TeleopPeriodic() {
  double joy_lStick_Y_deadband = 0.05, joy_rStick_Y_deadband = 0.05, joy_lStick_X_deadband = 0.05;  
  double joy_lStick_Y = cont_Driver->GetLeftY(), joy_lStick_X = cont_Driver->GetLeftX(), joy_rStick_Y = cont_Driver->GetRightY();
  double FLPos = FLMagEnc.GetAbsolutePosition();
  auto output = FLMagEnc.Get();
  bool con = FLMagEnc.IsConnected();

  SmartDashboard::PutNumber ("FLMag:", FLPos);
  SmartDashboard::PutNumber ("FL Values:", output.value());
  SmartDashboard::PutBoolean ("FL Connected:", con);
  SmartDashboard::PutNumber("Channel", FLMagEnc.GetSourceChannel());
  SmartDashboard::PutNumber("current position", FLPos);


  //Remove Deadzone
  if (abs(joy_lStick_Y) < joy_lStick_Y_deadband){
    joy_lStick_Y = 0;
  }

  if (abs(joy_rStick_Y) < joy_rStick_Y_deadband){
    joy_rStick_Y = 0;
  }

  if (abs(joy_lStick_X) < joy_lStick_X_deadband){
    joy_rStick_Y = 0;
  }
  
  //set target and power wheel will spin with
  double target = 45;
  double power = 0.2;

  double thetaRelative = FLMagEnc.GetAbsolutePosition()*360 - thetaInit;
  double angle = 0;
  double currentRelative;
  SmartDashboard::PutNumber("Theta Initial", thetaInit);
  SmartDashboard::PutNumber("Theta Raw", angle);
  //target = SmartDashboard::GetNumber("Target",0);


  //Find Controller Angle
  joy_lStick_Y *= -1;

  double controllerAngle = atan2(joy_lStick_Y , joy_lStick_X);
  SmartDashboard::PutNumber("atan2 angle", controllerAngle);
  SmartDashboard::PutNumber("XAxis", joy_lStick_X);
  SmartDashboard::PutNumber("YAxis", joy_lStick_Y);


  if (joy_lStick_X >= 0 && joy_lStick_Y >= 0){
    controllerAngle = (M_PI / 2) - atan2(joy_lStick_Y, joy_lStick_X);
  }
  else if (joy_lStick_X >= 0 && joy_lStick_Y <= 0){
    controllerAngle = M_PI / 2 - atan2(joy_lStick_Y, joy_lStick_X);
  }
  else if (joy_lStick_X <= 0 && joy_lStick_Y <= 0){
    controllerAngle = M_PI / 2 - atan2(joy_lStick_Y, joy_lStick_X);
  }
  else if (joy_lStick_X <= 0 && joy_lStick_Y >= 0){
    controllerAngle = 5 * M_PI / 2 - atan2(joy_lStick_Y, joy_lStick_X);
  }
  SmartDashboard::PutNumber("controller angle", controllerAngle);


  //Finds Relative Angle of Wheel
  if (thetaRelative < thetaInit){ 
    currentRelative = abs(thetaRelative - thetaInit);
  }
  else{
    currentRelative = abs((thetaRelative - 360) - thetaInit);
  }

  SmartDashboard::PutNumber("X:", currentRelative);

  //move towards target
  if(currentRelative < target){ 
    swerveFL.Set(ControlMode::PercentOutput, power);
  }
  else{
    swerveFL.Set(ControlMode::PercentOutput, 0);
  }

  power = sqrt(pow(joy_lStick_X, 2.0) + pow(joy_lStick_Y,2.0));
  if (abs(FLMagEnc.GetAbsolutePosition()*360 - target)> 20){
    swerveFL.Set(ControlMode::PercentOutput, 0);
    swerveFR.Set(ControlMode::PercentOutput, 0);
    swerveBL.Set(ControlMode::PercentOutput, 0);
    swerveBR.Set(ControlMode::PercentOutput, 0);
  }
  else{
    swerveFL.Set(ControlMode::PercentOutput,0);
    swerveFR.Set(ControlMode::PercentOutput,0);
    swerveBL.Set(ControlMode::PercentOutput,0);
    swerveBR.Set(ControlMode::PercentOutput,0);
  }

  driveFL.Set(ControlMode::PercentOutput,power * 0.2 * sgn(joy_lStick_Y));
  driveFR.Set(ControlMode::PercentOutput,power * 0.2 * sgn(joy_lStick_Y));
  driveBL.Set(ControlMode::PercentOutput,power * 0.2 * sgn(joy_lStick_Y));
  driveBR.Set(ControlMode::PercentOutput,power * 0.2 * sgn(joy_lStick_Y));


  } 



  //Wait(0.02_t);
*/

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}



#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
