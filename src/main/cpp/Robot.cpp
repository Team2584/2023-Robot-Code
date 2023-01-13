// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Setup.h"

#include "Swerve.cpp"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

double pigeon_initial;
// Instantiates a SwerveDrive object with all the correct references to motors and offset values
SwerveDrive *swerveDrive;

// To find values from cameras
nt::NetworkTableInstance inst;
shared_ptr<nt::NetworkTable> table;
nt::DoubleArrayTopic xTopic;
nt::DoubleArrayTopic yTopic;
nt::DoubleArrayTopic thetaTopic;
nt::IntegerTopic sanityTopic;
nt::DoubleArrayEntry xEntry;
nt::DoubleArrayEntry yEntry;
nt::DoubleArrayEntry thetaEntry;
nt::IntegerEntry sanityEntry;

// To track time for slew rate and accleration control
frc::Timer timer;
double lastTime = 0;
bool startedTimer = false;

// To Calibrate Odometry to april tag
bool isCalibrated = false;
double calibrationTime = 0;
double calibrationAmount = 0;
double caliX = 0;
double caliY = 0;
double caliTheta = 0;

void Robot::RobotInit()
{
  // Autonomous Choosing
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  // Setting motor breaktypes
  driveFL.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  driveBL.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  driveFR.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  driveBR.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);

  // Finding values from network tables
  inst = nt::NetworkTableInstance::GetDefault();
  inst.StartServer();
  table = inst.GetTable("vision/localization");
  xTopic = table->GetDoubleArrayTopic("x");
  yTopic = table->GetDoubleArrayTopic("y");
  thetaTopic = table->GetDoubleArrayTopic("theta");
  sanityTopic = table->GetIntegerTopic("sanitycheck");
  xEntry = xTopic.GetEntry({});
  yEntry = yTopic.GetEntry({});
  thetaEntry = thetaTopic.GetEntry({});
  sanityEntry = sanityTopic.GetEntry(10000);

  // Initializing things
  timer = Timer();

  // Initializing Subsystems
  swerveDrive = new SwerveDrive(&driveFL, &swerveFL, &FLMagEnc, FL_WHEEL_OFFSET, &driveFR, &swerveFR, &FRMagEnc,
                                FR_WHEEL_OFFSET, &driveBR, &swerveBR, &BRMagEnc, BR_WHEEL_OFFSET, &driveBL,
                                &swerveBL, &BLMagEnc, BL_WHEEL_OFFSET, &_pigeon, STARTING_DRIVE_HEADING);

  // Initializing Autonomous Trajectory (For Splines)
  swerveDrive->InitializeTrajectory();
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
  /*
  m_autoSelected = m_chooser.GetSelected();
  m_autoSelected = SmartDashboard::GetString("Auto Selector",
       kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom)
  {
    // Custom Auto goes here
  }
  else
  {
    // Default Auto goes here
  }
  */


  //Start our match timer and reset our odometry to the robot's starting position
  startedTimer = false;
  lastTime = 0;
  timer.Reset();
  swerveDrive->ResetOdometry(Pose2d(-2_m,  2_m, Rotation2d(0_rad)));
  swerveDrive->BeginPIDLoop();
}

void Robot::AutonomousPeriodic()
{
  /*
  if (m_autoSelected == kAutoNameCustom)
  {
    // Custom Auto goes here
  }
  else
  {
    // Default Auto goes here
  }
  */

  //If we haven't started the timer yet, start the timer
  if (!startedTimer)
  {
    timer.Start();
    startedTimer = false;
  }


  //Follow the trajectory of the swerve drive
  swerveDrive->FollowTrajectory(timer.Get(), timer.Get().value() - lastTime);
  lastTime = timer.Get().value();
}
/*
void Robot::TeleopInit() {
  inst = nt::NetworkTableInstance::GetDefault();
  inst.StartServer();
  table = inst.GetTable("vision/localization");
  xTopic = table->GetDoubleTopic("x");
  xEntry = xTopic.GetEntry(100000);
}

int hi = 0;

void Robot::TeleopPeriodic() {
  frc::SmartDashboard::PutNumber("x I'm getting", xEntry.Get());
  frc::SmartDashboard::PutNumber("hi", hi);
  wpi::outs() << "yo";
  frc::SmartDashboard::PutNumber("aj counter", sanityEntry.Get());
  hi += 1;
  inst.Flush();
}
*/

void Robot::TeleopInit()
{
  // Prepare swerve drive odometry
  pigeon_initial = fmod(_pigeon.GetYaw() + STARTING_DRIVE_HEADING, 360);
  swerveDrive->pigeon_initial = pigeon_initial;
  swerveDrive->ResetOdometry();

  timer.Reset();
  timer.Start();
  lastTime = 0;

  isCalibrated = false;
  calibrationTime = 0;
  calibrationAmount = 0;
  caliX = 0;
  caliY = 0;
  caliTheta = 0;
  /*
    orchestra.LoadMusic("CHIRP");
    orchestra.AddInstrument(swerveBL);
    orchestra.AddInstrument(driveBL);
    orchestra.Play();
  */
}

void Robot::TeleopPeriodic()
{
  double joy_lStick_Y, joy_lStick_X, joy_rStick_X;
  // Find controller input
  if (CONTROLLER_TYPE == 0)
  {
    joy_lStick_Y = cont_Driver->GetLeftY();
    joy_lStick_X = cont_Driver->GetLeftX();
    joy_rStick_X = cont_Driver->GetRightX();
    joy_lStick_Y *= -1;
  }
  else if (CONTROLLER_TYPE == 1)
  {
    joy_lStick_Y = xbox_Drive->GetLeftY();
    joy_lStick_X = xbox_Drive->GetLeftX();
    joy_rStick_X = xbox_Drive->GetRightX();
    joy_lStick_Y *= -1;
  }

  // Remove ghost movement by making sure joystick is moved a certain amount
  double joy_lStick_distance = sqrt(pow(joy_lStick_X, 2.0) + pow(joy_lStick_Y, 2.0));

  if (joy_lStick_distance < CONTROLLER_DEADBAND)
  {
    joy_lStick_X = 0;
    joy_lStick_Y = 0;
  }

  if (abs(joy_rStick_X) < CONTROLLER_DEADBAND)
  {
    joy_rStick_X = 0;
  }

  //Scale our joystick inputs to our intended max drive speeds
  double FWD_Drive_Speed = joy_lStick_Y * MAX_DRIVE_SPEED;
  double STRAFE_Drive_Speed = joy_lStick_X * MAX_DRIVE_SPEED;
  double Turn_Speed = joy_rStick_X * MAX_SPIN_SPEED;

  //update our timer
  double time = timer.Get().value();
  double elapsedTime = time - lastTime;
  lastTime = time;

  //Update our odometry 
  swerveDrive->UpdateOdometry(units::microsecond_t{RobotController::GetFPGATime()});

  SmartDashboard::PutNumber("FPGA Time", RobotController::GetFPGATime());

  //Update our vision pose from the network table (calculated by a coprocessor, ask Avrick / AJ for details)
  for (int i = 0; i < xEntry.Get().size(); i++)
  {

  }


  // WIP not important
  if (!isCalibrated && existsEntry.Get())
  {
    isCalibrated = true;
    swerveDrive->ResetOdometry(visionPose);
  }

  /*

    if (existsEntry.Get() && !isCalibrated && calibrationTime < 0.25)
    {
      calibrationAmount += 1;
      caliX += visionPose.X().value();
      caliY += visionPose.Y().value();
      caliTheta += visionPose.Rotation().Radians().value();
    }
    else if (existsEntry.Get() && !isCalibrated)
    {
      isCalibrated = true;
      caliX /= calibrationAmount;
      caliY /= calibrationAmount;
      caliTheta /= calibrationAmount;
      swerveDrive->ResetOdometry(Pose2d(units::meter_t{caliX}, units::meter_t{caliY}, Rotation2d(units::radian_t{caliTheta})));
    }*/


  // DEBUG INFO
  Pose2d pose = swerveDrive->GetPose();
  Pose2d visionOdometry = swerveDrive->GetPoseVisionOdometry();

  frc::SmartDashboard::PutBoolean("Was 0", thetaEntry.Get() < 0.05 && thetaEntry.Get() > -0.05 && thetaEntry.Get() != 0.0);

  frc::SmartDashboard::PutNumber("FWD Drive Speed", FWD_Drive_Speed);
  frc::SmartDashboard::PutNumber("Strafe Drive Speed", STRAFE_Drive_Speed);
  frc::SmartDashboard::PutNumber("Turn Drive Speed", Turn_Speed);

  SmartDashboard::PutNumber("Network Table X", xEntry.Get());
  SmartDashboard::PutBoolean("Network Table X New Value", (bool)xEntry.ReadQueue().size());
  SmartDashboard::PutNumber("Network Table Y", yEntry.Get());
  SmartDashboard::PutNumber("Network Table Theta", thetaEntry.Get() * 180 / M_PI);
  SmartDashboard::PutNumber("Network Table Sanity", sanityEntry.Get());
  SmartDashboard::PutBoolean("Network Table Tag Exists", existsEntry.Get());

  frc::SmartDashboard::PutNumber("Odometry X", pose.X().value());
  frc::SmartDashboard::PutNumber("Odometry Y", pose.Y().value());
  frc::SmartDashboard::PutNumber("Odometry Theta", pose.Rotation().Degrees().value());

  frc::SmartDashboard::PutNumber("Vision Odometry X", visionOdometry.X().value());
  frc::SmartDashboard::PutNumber("Vision Odometry Y", visionOdometry.Y().value());
  frc::SmartDashboard::PutNumber("Vision Odometry Theta", visionOdometry.Rotation().Degrees().value());

  frc::SmartDashboard::PutNumber("TIMER", timer.Get().value());

  
  //Here is our Test Drive Control Code that runs different functions when different buttons are pressed
  if (xbox_Drive->GetLeftBumper())
    Turn_Speed = swerveDrive->TurnToPointDesiredSpin(pose, Translation2d(0_m, 0_m), elapsedTime, TURN_TO_POINT_ALLOWABLE_ERROR, TURN_TO_POINT_MAX_SPIN, TURN_TO_POINT_MAX_ACCEL, TURN_TO_TO_POINT_P, TURN_TO_TO_POINT_I);

  swerveDrive->DriveSwervePercent(STRAFE_Drive_Speed, FWD_Drive_Speed, Turn_Speed);

  if (xbox_Drive->GetBButtonPressed())
    swerveDrive->BeginPIDLoop();
  if ((CONTROLLER_TYPE == 0 && cont_Driver->GetSquareButtonPressed()) || (CONTROLLER_TYPE == 1 && xbox_Drive->GetBButton()))
    swerveDrive->DriveToPoseVisionOdometry(Pose2d(0_m, -1_m, Rotation2d(0_rad)), elapsedTime);

  if (xbox_Drive->GetAButtonPressed())
    swerveDrive->BeginPIDLoop();
  if ((CONTROLLER_TYPE == 0 && cont_Driver->GetTriangleButton()) || (CONTROLLER_TYPE == 1 && xbox_Drive->GetAButton()))
    swerveDrive->DriveToPoseVisionOdometry(Pose2d(0_m, -2_m, Rotation2d(0_rad)), elapsedTime);

  if (xbox_Drive->GetXButtonPressed())
    swerveDrive->BeginPIDLoop();
  if ((CONTROLLER_TYPE == 0 && cont_Driver->GetTriangleButton()) || (CONTROLLER_TYPE == 1 && xbox_Drive->GetXButton()))
    swerveDrive->DriveToPoseVisionOdometry(Pose2d(-0.5_m, -3_m, Rotation2d(0.5_rad)), elapsedTime);

  if (xbox_Drive->GetRightBumperPressed())
    swerveDrive->BeginPIDLoop();
  if (xbox_Drive->GetRightBumper())
    swerveDrive->DriveToPoseOdometry(Pose2d(0_m, 0_m, Rotation2d(0_rad)), elapsedTime);

  // Reset Pigion Heading
  if (CONTROLLER_TYPE == 0 && cont_Driver->GetCircleButtonPressed())
  {
    pigeon_initial = fmod(_pigeon.GetYaw(), 360);
    swerveDrive->pigeon_initial = pigeon_initial;
  }
  else if (CONTROLLER_TYPE == 1 && xbox_Drive->GetYButtonPressed())
  {
    pigeon_initial = fmod(_pigeon.GetYaw(), 360);
    swerveDrive->pigeon_initial = pigeon_initial;
  }
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
