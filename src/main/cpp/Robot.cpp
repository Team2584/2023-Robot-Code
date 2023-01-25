// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Setup.h"

#include "Swerve.cpp"
#include "Elevator.cpp"
#include "Limelight.cpp"
#include "Claw.cpp"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

double pigeon_initial;
// Our future subsystem objects
SwerveDrive *swerveDrive;
ElevatorLift *elevatorLift;
Limelight *limelight;
Claw *claw;

// To find values from cameras
nt::NetworkTableInstance inst;
shared_ptr<nt::NetworkTable> aprilTagTable;
shared_ptr<nt::NetworkTable> limelightTable;
shared_ptr<nt::NetworkTable> objectTable;
nt::DoubleArrayTopic poseTopic;
nt::DoubleTopic sanityTopic;
nt::DoubleArrayTopic curPoseTopic;
nt::DoubleArraySubscriber poseSub;
nt::DoubleEntry sanityEntry;
nt::DoubleArrayEntry curPoseEntry;
nt::DoubleTopic polePixelTopic;
nt::DoubleEntry polePixelEntry;
nt::DoubleTopic coneXTopic;
nt::DoubleEntry coneXEntry;
nt::DoubleTopic coneYTopic;
nt::DoubleEntry coneYEntry;
// To track time for slew rate and pid controll
frc::Timer timer;
double lastTime = 0;
bool startedTimer = false;
double lastFwdSpeed = 0;
double lastStrafeSpeed = 0;
double lastTurnSpeed = 0;

// Values to Set with ShuffleBoard
double MAX_DRIVE_SPEED = 0.4;
double MAX_SPIN_SPEED = 0.4;
double ELEVATOR_SPEED = 0.1;

// Cringe Auto Values S**FF
double splineSection = 1;
bool limelightTracking = false;
bool done = false;

double lastSanity = 0;

void Robot::RobotInit()
{
  // Set all Values from Shuffleboard (Smartdashboard but cooler)
  frc::SmartDashboard::PutNumber("MAX DRIVE SPEED", 0.4);
  frc::SmartDashboard::PutNumber("ELEVATOR_SPEED", 0.1) ;

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
  aprilTagTable = inst.GetTable("vision/localization");
  limelightTable = inst.GetTable("limelight");
  objectTable = inst.GetTable("vision/objects");
  poseTopic = aprilTagTable->GetDoubleArrayTopic("poseArray");
  sanityTopic = objectTable->GetDoubleTopic("sanitycheck");
  curPoseTopic = limelightTable->GetDoubleArrayTopic("curPose");
  polePixelTopic = limelightTable->GetDoubleTopic("polePixel");
  coneXTopic = objectTable->GetDoubleTopic("coneX");
  coneYTopic = objectTable->GetDoubleTopic("coneY");
  poseSub = poseTopic.Subscribe({});
  sanityEntry = sanityTopic.GetEntry(10000);
  curPoseEntry = curPoseTopic.GetEntry({});
  polePixelEntry = polePixelTopic.GetEntry(1000);
  coneXEntry = coneXTopic.GetEntry(0);
  coneYEntry = coneYTopic.GetEntry(0);

  // Initializing things
  timer = Timer();

  // Initializing Subsystems
  swerveDrive = new SwerveDrive(&driveFL, &swerveFL, &FLMagEnc, FL_WHEEL_OFFSET, &driveFR, &swerveFR, &FRMagEnc,
                                FR_WHEEL_OFFSET, &driveBR, &swerveBR, &BRMagEnc, BR_WHEEL_OFFSET, &driveBL,
                                &swerveBL, &BLMagEnc, BL_WHEEL_OFFSET, &_pigeon, STARTING_DRIVE_HEADING);

  elevatorLift = new ElevatorLift(&winchL, &winchR, &TOFSensor);
  limelight = new Limelight(limelightTable);
  claw = new Claw(&wrist);

  // Initializing Autonomous Trajectory (For Splines)
  swerveDrive->InitializeTrajectory("RedRight3GamePiece1");
  swerveDrive->InitializeTrajectory("RedRight3GamePiece2");
  swerveDrive->InitializeTrajectory("RedRight3GamePiece3");
  swerveDrive->InitializeTrajectory("RedRight3GamePiece4");
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

  swerveDrive->ResetOdometry(Pose2d(4.74_m,  1.89_m, Rotation2d(3.14_rad)));
  swerveDrive->BeginPIDLoop();
  swerveDrive->SetNextTrajectory();


 splineSection = 1;
 limelightTracking = false;
 done = false;
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
    startedTimer = true;
  }

  // Update Odometry
  swerveDrive->UpdateOdometry(timer.Get());

  //Follow the trajectory of the swerve drive
  if (!limelightTracking && !done)
  {
    bool splineDone = swerveDrive->FollowTrajectory(timer.Get(), timer.Get().value() - lastTime);
    SmartDashboard::PutBoolean("Spline Done", splineDone);
    if (splineDone && (splineSection == 2 || splineSection == 4))
    {
      limelightTracking = true;
      splineSection += 1;
      swerveDrive->SetNextTrajectory();
      timer.Reset();
      lastTime = 0;
    }
    else if (splineDone)
    {
      splineSection += 1;
      swerveDrive->SetNextTrajectory(); 
      timer.Reset();
      lastTime = 0;
    }
  }
  else if (!done)
  {
    bool limelightDone = swerveDrive->StrafeToPole(limelight->getTargetX(), timer.Get().value() - lastTime);
    if (limelightDone)
    {
      limelightTracking = false;
      done = true;
      timer.Reset();  
      lastTime = 0;
    }
  }
  else
  {
    swerveDrive->DriveSwervePercent(0,0,0);
  }

  lastTime = timer.Get().value();
SmartDashboard::PutBoolean("limelightTracking", limelightTracking);

  SmartDashboard::PutNumber("SPline Section", splineSection);
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
  swerveDrive->ResetOdometry(Pose2d(0_m, 1_m, Rotation2d(180_deg)));

  //Reset all our values throughout the code
  timer.Reset();
  timer.Start();
  lastTime = 0;

  lastFwdSpeed = 0;
  lastStrafeSpeed = 0;
  lastTurnSpeed = 0;

  /*
    orchestra.LoadMusic("CHIRP");
    orchestra.AddInstrument(swerveBL);
    orchestra.AddInstrument(driveBL);
    orchestra.Play();
  */
}

int counter = 0;
void Robot::TeleopPeriodic()
{
  SmartDashboard::PutNumber("encoder", elevatorLift->winchEncoderReading());

  // Take values from Smartdashboard
  MAX_DRIVE_SPEED = frc::SmartDashboard::GetNumber("MAX DRIVE SPEED", 0.4);
  ELEVATOR_SPEED = frc::SmartDashboard::GetNumber("ELEVATOR_SPEED", 0.1);

  SmartDashboard::PutNumber("Spin Encoder Tracker", swerveDrive->FLModule->GetSpinEncoderRadians());

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

  lastFwdSpeed += std::clamp(FWD_Drive_Speed - lastFwdSpeed, -1 * MAX_DRIVE_ACCELERATION * elapsedTime,
                   MAX_DRIVE_ACCELERATION * elapsedTime);
  lastStrafeSpeed += std::clamp(STRAFE_Drive_Speed - lastStrafeSpeed, -1 * MAX_DRIVE_ACCELERATION * elapsedTime,
                   MAX_DRIVE_ACCELERATION * elapsedTime);
  lastTurnSpeed += std::clamp(Turn_Speed - lastTurnSpeed, -1 * MAX_SPIN_ACCELERATION * elapsedTime,
                   MAX_SPIN_ACCELERATION * elapsedTime);
  lastTime = time;


  //Update our odometry 
  swerveDrive->UpdateOdometry(units::microsecond_t{RobotController::GetFPGATime()});

  for (auto array : poseSub.ReadQueue()) 
  {
    Pose2d poseEst = Pose2d(units::meter_t{array.value[0]}, units::meter_t{array.value[1]}, Rotation2d(units::radian_t{array.value[3]}));
   frc::SmartDashboard::PutNumber("Vision X", poseEst.X().value());
   frc::SmartDashboard::PutNumber("Vision Y", poseEst.Y().value());
   frc::SmartDashboard::PutNumber("vision Theta", poseEst.Rotation().Degrees().value());
    swerveDrive->AddPositionEstimate(poseEst, units::microsecond_t{array.time - array.value[4]});
  }

  Pose2d pose = swerveDrive->GetPose();
  double poseArray[] = {pose.X().value(), pose.Y().value(), 0.75, pose.Rotation().Radians().value(), 0};
  curPoseEntry.Set(poseArray);

  // DEBUG INFO

  frc::SmartDashboard::PutNumber("TOF", elevatorLift->TOFSReading());

  // frc::SmartDashboard::PutNumber("FWD Drive Speed", lastFwdSpeed);
  // frc::SmartDashboard::PutNumber("Strafe Drive Speed", lastStrafeSpeed);
  // frc::SmartDashboard::PutNumber("Turn Drive Speed", lastTurnSpeed);
   frc::SmartDashboard::PutNumber("Odometry X", pose.X().value());
   frc::SmartDashboard::PutNumber("Odometry Y", pose.Y().value());
   frc::SmartDashboard::PutNumber("Odometry Theta", pose.Rotation().Degrees().value());

  SmartDashboard::PutNumber("Robot Controller FPGA Time", RobotController::GetFPGATime());
  SmartDashboard::PutNumber("Timer Class FPGA Time", Timer::GetFPGATimestamp().value());

/*
  frc::SmartDashboard::PutBoolean("Was 0", thetaEntry.Get() < 0.05 && thetaEntry.Get() > -0.05 && thetaEntry.Get() != 0.0);

  SmartDashboard::PutNumber("Robot Controller FPGA Time", RobotController::GetFPGATime());
  SmartDashboard::PutNumber("Timer Class FPGA Time", Timer::GetFPGATimestamp().value());

  SmartDashboard::PutNumber("Network Table Sanity", sanityEntry.Get());

  frc::SmartDashboard::PutNumber("TIMER", timer.Get().value());
*/

  swerveDrive->DriveSwervePercent(lastStrafeSpeed, lastFwdSpeed, lastTurnSpeed);

  // LIMELIGHT CODE
  // if (xbox_Drive->GetRightBumper())
  // {
  //  // double offset = coneXEntry.Get(); for cones
  //   double offset = limelight->getTargetX();
  //   bool thing = swerveDrive->StrafeToPole(offset, elapsedTime);
  //   SmartDashboard::PutNumber("thing", thing);
  // }

  if (xbox_Drive->GetRightBumperPressed())
    swerveDrive->BeginPIDLoop();
  if (xbox_Drive->GetRightBumper())
    swerveDrive->DriveToPose(Pose2d(0_m, 1_m, Rotation2d(180_deg)), elapsedTime);


  // BASIC ELEVATOR CODE
  if (xbox_Drive->GetBButtonPressed())
    elevatorLift->StartPIDLoop();

  if (xbox_Drive->GetYButton())
    elevatorLift->MoveElevatorPercent(0.2);
  else if (xbox_Drive->GetAButton())
    elevatorLift->MoveElevatorPercent(-0.2);
  else if (xbox_Drive->GetBButton())
    elevatorLift->SetElevatorHeightPID(5, elapsedTime);
  else if (xbox_Drive->GetXButton())
    elevatorLift->StopElevatorBreak();
  else 
    elevatorLift->StopElevatorBreak();


  if (xbox_Drive->GetStartButton())
    claw->MoveWristPercent(0.2);
  else if (xbox_Drive->GetBackButton())
    claw->MoveWristPercent(-0.2);
  else 
    claw->MoveWristPercent(0);  

  //Here is our Test Drive Control Code that runs different functions when different buttons are pressed
  /*
  if (xbox_Drive->GetLeftBumper())
    Turn_Speed = swerveDrive->TurnToPointDesiredSpin(Translation2d(0_m, 0_m), elapsedTime, TURN_TO_POINT_ALLOWABLE_ERROR, TURN_TO_POINT_MAX_SPIN, TURN_TO_POINT_MAX_ACCEL, TURN_TO_TO_POINT_P, TURN_TO_TO_POINT_I);

  swerveDrive->DriveSwervePercent(lastStrafeSpeed, lastFwdSpeed, lastTurnSpeed);

  if (xbox_Drive->GetBButtonPressed())
    swerveDrive->BeginPIDLoop();
  if ((CONTROLLER_TYPE == 0 && cont_Driver->GetSquareButtonPressed()) || (CONTROLLER_TYPE == 1 && xbox_Drive->GetBButton()))
    swerveDrive->DriveToPose(Pose2d(0_m, -1_m, Rotation2d(0_rad)), elapsedTime);

  if (xbox_Drive->GetAButtonPressed())
    swerveDrive->BeginPIDLoop();
  if ((CONTROLLER_TYPE == 0 && cont_Driver->GetTriangleButton()) || (CONTROLLER_TYPE == 1 && xbox_Drive->GetAButton()))
    swerveDrive->DriveToPose(Pose2d(0_m, -2_m, Rotation2d(0_rad)), elapsedTime);

  if (xbox_Drive->GetXButtonPressed())
    swerveDrive->BeginPIDLoop();
  if ((CONTROLLER_TYPE == 0 && cont_Driver->GetTriangleButton()) || (CONTROLLER_TYPE == 1 && xbox_Drive->GetXButton()))
    swerveDrive->DriveToPose(Pose2d(-0.5_m, -3_m, Rotation2d(0.5_rad)), elapsedTime);

  if (xbox_Drive->GetRightBumperPressed())
    swerveDrive->BeginPIDLoop();
  if (xbox_Drive->GetRightBumper())
    swerveDrive->DriveToPose(Pose2d(0_m, 0_m, Rotation2d(0_rad)), elapsedTime);

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
  */
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
