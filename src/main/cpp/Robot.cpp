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
nt::DoubleTopic xTopic;
nt::DoubleTopic yTopic;
nt::DoubleTopic thetaTopic;
nt::StringTopic sanityTopic;
nt::BooleanTopic existsTopic;
nt::DoubleEntry xEntry;
nt::DoubleEntry yEntry;
nt::DoubleEntry thetaEntry;
nt::StringEntry sanityEntry;
nt::BooleanEntry existsEntry;

// To track time for slew rate and accleration control
frc::Timer timer;
double lastTime = 0;
double lastFwd = 0;
double lastStrafe = 0;
double lastTurn = 0;

void Robot::RobotInit()
{
  //Autonomous Choosing
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  //Finding values from network tables
  inst = nt::NetworkTableInstance::GetDefault();
  inst.StartServer();
  table = inst.GetTable("vision/localization");
  xTopic = table->GetDoubleTopic("x");
  yTopic = table->GetDoubleTopic("y");
  thetaTopic = table->GetDoubleTopic("theta");
  sanityTopic = table->GetStringTopic("sanitycheck");
  existsTopic = table->GetBooleanTopic("robot_pos_good");
  xEntry = xTopic.GetEntry(0);
  yEntry = yTopic.GetEntry(0);
  thetaEntry = thetaTopic.GetEntry(0);
  sanityEntry = sanityTopic.GetEntry("didn't work");
  existsEntry = existsTopic.GetEntry(false);

  //Initializing things
  timer = Timer();

  //Initializing Subsystems
  swerveDrive = new SwerveDrive(&driveFL, &swerveFL, &FLMagEnc, FL_WHEEL_OFFSET, &driveFR, &swerveFR, &FRMagEnc,
                                        FR_WHEEL_OFFSET, &driveBR, &swerveBR, &BRMagEnc, BR_WHEEL_OFFSET, &driveBL,
                                        &swerveBL, &BLMagEnc, BL_WHEEL_OFFSET, &_pigeon, STARTING_DRIVE_HEADING);
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
  //Prepare swerve drive odometry
  pigeon_initial = fmod(_pigeon.GetYaw() + STARTING_DRIVE_HEADING, 360);
  swerveDrive->pigeon_initial = pigeon_initial;
  swerveDrive->ResetOdometry();

  timer.Reset();
  timer.Start();
  lastTime = 0;
  lastFwd = 0;
  lastTurn = 0;
  lastStrafe = 0;

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

  // Find Pigeon IMU input and convert it to an angle
  double pigeon_angle = fmod(_pigeon.GetYaw(), 360);
  pigeon_angle -= pigeon_initial;
  if (pigeon_angle < 0)
    pigeon_angle += 360;
  pigeon_angle = 360 - pigeon_angle;
  if (pigeon_angle == 360)
    pigeon_angle = 0;
  pigeon_angle *= M_PI / 180;

  // Use pigion_angle to determine what our target movement vector is in relation to the robot
  // This code keeps the driving "field oriented" by determining the angle in relation to the front of the robot we
  // really want to move towards
  double FWD_Drive_Speed = joy_lStick_Y * cos(pigeon_angle) + joy_lStick_X * sin(pigeon_angle);
  double STRAFE_Drive_Speed = -1 * joy_lStick_Y * sin(pigeon_angle) + joy_lStick_X * cos(pigeon_angle);
  double Turn_Speed = joy_rStick_X;

  FWD_Drive_Speed *= MAX_DRIVE_SPEED;
  STRAFE_Drive_Speed *= MAX_DRIVE_SPEED;
  Turn_Speed *= MAX_SPIN_SPEED;

  swerveDrive->UpdateOdometry();
  Pose2d pose = swerveDrive->GetPose();

  swerveDrive->SetPoseVision(Pose2d(units::meter_t{xEntry.Get()}, units::meter_t{yEntry.Get()}, Rotation2d(units::radian_t{thetaEntry.Get()})));

  //Slew Rate Control
  double time = timer.Get().value();
  double elapsedTime = time - lastTime;
  lastFwd += std::clamp(FWD_Drive_Speed - lastFwd, -1 * MAX_DRIVE_ACCELERATION * elapsedTime,
                   MAX_DRIVE_ACCELERATION * elapsedTime);
  FWD_Drive_Speed = lastFwd;
  lastStrafe += std::clamp(STRAFE_Drive_Speed - lastStrafe, -1 * MAX_DRIVE_ACCELERATION * elapsedTime,
                   MAX_DRIVE_ACCELERATION * elapsedTime);
  STRAFE_Drive_Speed = lastStrafe;
  lastTurn += std::clamp(Turn_Speed - lastTurn, -1 * MAX_SPIN_ACCELERATION * elapsedTime,
                   MAX_SPIN_ACCELERATION * elapsedTime);
  Turn_Speed = lastTurn;
  lastTime = time;

  frc::SmartDashboard::PutNumber("FWD Drive Speed", FWD_Drive_Speed);
  frc::SmartDashboard::PutNumber("Strafe Drive Speed", STRAFE_Drive_Speed);
  frc::SmartDashboard::PutNumber("Turn Drive Speed", Turn_Speed);

  SmartDashboard::PutNumber("Network Table X", xEntry.Get());
  SmartDashboard::PutNumber("Network Table Y", yEntry.Get());
  SmartDashboard::PutNumber("Network Table Theta", thetaEntry.Get());
  SmartDashboard::PutString("Network Table Sanity", sanityEntry.Get());
  SmartDashboard::PutBoolean("Network Table Tag Exists", existsEntry.Get());

  frc::SmartDashboard::PutNumber("Odometry X", pose.X().value());
  frc::SmartDashboard::PutNumber("Odometry Y", pose.Y().value());
  frc::SmartDashboard::PutNumber("Odometry Theta", pose.Rotation().Degrees().value());

  frc::SmartDashboard::PutNumber("TIMER", timer.Get().value());

  // Moves the swerve drive in the intended direction, with the speed scaled down by our pre-chosen, 
  // max drive and spin speeds
  if (xbox_Drive->GetXButton())
  {
    swerveDrive->TurnToPointWhileDriving(FWD_Drive_Speed, STRAFE_Drive_Speed, Translation2d(1_m, 0_m), elapsedTime);
  }
  else
  {
    swerveDrive->DriveSwervePercent(FWD_Drive_Speed, STRAFE_Drive_Speed, Turn_Speed);
  }

  if (CONTROLLER_TYPE == 0 && cont_Driver->GetSquareButtonPressed())
  {
    swerveDrive->DriveToPoseOdometry(Pose2d(0_m, 0_m, Rotation2d(0_rad)), elapsedTime);
  }
  else if (CONTROLLER_TYPE == 1 && xbox_Drive->GetBButton())
  {
    swerveDrive->DriveToPoseOdometry(Pose2d(0_m, 0_m, Rotation2d(0_rad)), elapsedTime);  
  }
  
  if ((CONTROLLER_TYPE == 0 && cont_Driver->GetTriangleButton()) || (CONTROLLER_TYPE == 1 && xbox_Drive->GetAButton()))
  {
    if (existsEntry.Get())
    {
      swerveDrive->SetPoseVision(Pose2d(units::meter_t{xEntry.Get()}, units::meter_t{yEntry.Get()}, Rotation2d(units::radian_t{thetaEntry.Get()})));
      swerveDrive->ResetOdometry(Pose2d(units::meter_t{xEntry.Get()}, units::meter_t{yEntry.Get()}, Rotation2d(units::radian_t{thetaEntry.Get()})));
      swerveDrive->DriveToPoseVision(Pose2d(0_m, -2_m, Rotation2d(0_rad)), elapsedTime);
    }
    else 
    {
      swerveDrive->DriveToPoseOdometry(Pose2d(0_m, -2_m, Rotation2d(0_rad)), elapsedTime);
    }
  }

  //Reset Pigion Heading*
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
