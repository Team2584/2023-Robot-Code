// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Setup.h"

#include "Swerve.h"
#include "Elevator.H"
#include "Limelight.h"
#include "Claw.h"

double pigeon_initial;
// Our future subsystem objects
SwerveDrive *swerveDrive;
Elevator *elevator;
Limelight *limelight;
Claw *claw;

// To find values from cameras
nt::NetworkTableInstance inst;
shared_ptr<nt::NetworkTable> visionTable;
shared_ptr<nt::NetworkTable> limelightTable;
nt::DoubleArrayTopic poseTopic;
nt::DoubleTopic sanityTopic;
nt::DoubleArrayTopic curPoseTopic;
nt::DoubleArraySubscriber poseSub;
nt::DoubleEntry sanityEntry;
nt::DoubleArrayEntry curPoseEntry;
nt::DoubleTopic polePixelTopic;
nt::DoubleEntry polePixelEntry;
nt::DoubleArrayTopic coneTopic;
nt::DoubleArraySubscriber coneEntry;

// To track time for slew rate and pid controll
frc::Timer timer;
double lastTime = 0;
double lastFwdSpeed = 0;
double lastStrafeSpeed = 0;
double lastTurnSpeed = 0;

// Values to Set with ShuffleBoard
double MAX_DRIVE_SPEED = 0.4;
double MAX_SPIN_SPEED = 0.4;
double ELEVATOR_SPEED = 0.1;

// Cringe Auto Values S**FF
double splineSection = 0;

double lastSanity = 0;

void Robot::RobotInit()
{
  // Set all Values from Shuffleboard (Smartdashboard but cooler)
  frc::SmartDashboard::PutNumber("MAX DRIVE SPEED", 0.4);
  frc::SmartDashboard::PutNumber("ELEVATOR_SPEED", 0.1);

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
  visionTable = inst.GetTable("vision");
  limelightTable = inst.GetTable("limelight");
  poseTopic = visionTable->GetDoubleArrayTopic("poseArray");
  sanityTopic = visionTable->GetDoubleTopic("sanitycheck");
  curPoseTopic = visionTable->GetDoubleArrayTopic("curPose");
  polePixelTopic = limelightTable->GetDoubleTopic("polePixel");
  coneTopic = visionTable->GetDoubleArrayTopic("conePos");
  poseSub = poseTopic.Subscribe({});
  sanityEntry = sanityTopic.GetEntry(10000);
  curPoseEntry = curPoseTopic.GetEntry({});
  polePixelEntry = polePixelTopic.GetEntry(1000);
  coneEntry = coneTopic.Subscribe({});

  // Initializing things
  timer = Timer();

  // Initializing Subsystems
  swerveDrive = new SwerveDrive(&driveFL, &swerveFL, &FLMagEnc, &driveFR, &swerveFR, &FRMagEnc, &driveBR, &swerveBR, &BRMagEnc, &driveBL,
                                &swerveBL, &BLMagEnc, &_pigeon);

  elevator = new Elevator(&winchL, &winchR, &TOFSensor);
  limelight = new Limelight(limelightTable);
  claw = new Claw(&wrist, &clawM1);
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
  /*m_autoSelected = m_chooser.GetSelected();
  m_autoSelected = SmartDashboard::GetString("Auto Selector",
                                             kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom)
  {
    swerveDrive->ResetOdometry(Pose2d(4.74_m, 1.89_m, Rotation2d(3.14_rad)));
    swerveDrive->ResetTrajectoryList();
    swerveDrive->InitializeTrajectory("RedRight3GamePiece1");
    swerveDrive->InitializeTrajectory("RedRight3GamePiece2");
    swerveDrive->InitializeTrajectory("RedRight3GamePiece3");
    swerveDrive->InitializeTrajectory("RedRight3GamePiece4");
    swerveDrive->SetNextTrajectory();

    splineSection = 0;
  }
  else
  {
    splineSection = 0;
  }*/

  elevator->ResetElevatorEncoder();
  claw->ResetClawEncoder();
  swerveDrive->ResetOdometry(Pose2d(5.22_m, 1.78_m, Rotation2d(3.14_rad)));
  swerveDrive->ResetTrajectoryList();
  swerveDrive->InitializeTrajectory("RedRight3GamePiece1");
  swerveDrive->InitializeTrajectory("RedRight3GamePiece2");
  swerveDrive->InitializeTrajectory("RedRight3GamePiece3");
  swerveDrive->InitializeTrajectory("RedRight3GamePiece4");
  swerveDrive->SetNextTrajectory();
  splineSection = 0;

  // Start our match timer and reset our odometry to the robot's starting position
  lastTime = 0;
  timer.Reset();
  timer.Start();
}

void Robot::AutonomousPeriodic()
{
  double elapsedTime = timer.Get().value() - lastTime;
  swerveDrive->UpdateOdometry(timer.Get());
  for (auto array : poseSub.ReadQueue())
  {
    Translation2d poseEst = Translation2d(units::meter_t{array.value[0]}, units::meter_t{array.value[1]});
    frc::SmartDashboard::PutNumber("Vision X", poseEst.X().value());
    frc::SmartDashboard::PutNumber("Vision Y", poseEst.Y().value());
    swerveDrive->AddPositionEstimate(poseEst, units::microsecond_t{array.time - array.value[4]});
  }

  if (splineSection == 0)
  {
    timer.Reset();
    lastTime = 0;
    elevator->StartPIDLoop();
    splineSection = 0.25;
  }

  if (splineSection == 0.25)
  {
    claw->PIDWrist(0.6, elapsedTime);
    bool elevatorDone = false;
    if (claw->MagEncoderReading() > 0.4)
      elevatorDone = elevator->SetElevatorHeightPID(76, elapsedTime);
    if (elevatorDone)
    {
      elevator->StopElevator();
      splineSection = 0.5;
    }
  }

  if (splineSection == 0.5)
  {
    double wristDone = claw->PIDWrist(M_PI / 2 - 0.5, elapsedTime);
    if (wristDone)
    {
      splineSection = 0.75;
    }
  }

  if (splineSection == 0.75)
  {
    claw->PIDWrist(M_PI / 2 - 0.5, elapsedTime);
    claw->OpenClaw(elapsedTime);
    if (claw->ClawEncoderReading() < -5)
    {
      claw->OpenClaw(elapsedTime);
      splineSection = 0.9;
    }
  }

  if (splineSection == 0.9)
  {
    claw->PIDWrist(1, elapsedTime);
    claw->OpenClaw(elapsedTime);
    elevator->SetElevatorHeightPID(0, elapsedTime);
    if (elevator->winchEncoderReading() < 30)
    {
      timer.Reset();
      lastTime = 0;
      splineSection = 1;
    }
  }

  if (splineSection == 1)
  {
    if (swerveDrive->GetPose().Y() > 5_m)
    {
      for (auto array : coneEntry.ReadQueue())
      {
        double angle = -1 * swerveDrive->GetPose().Rotation().Radians().value();
        if (array.value[0] != 0 || array.value[1] != 0)
        {
          double fieldOrientedX = -1 * (array.value[0] * cos(angle) - array.value[1] * sin(angle));
          double fieldOrientedY = -1 * (array.value[0] * sin(angle) + array.value[1] * cos(angle));
          Translation2d transEst = Translation2d(4.56_m + units::meter_t{fieldOrientedX}, 7_m + units::meter_t{fieldOrientedY});
          frc::SmartDashboard::PutNumber("Cone X Final", transEst.X().value());
          frc::SmartDashboard::PutNumber("Cone Y Final", transEst.Y().value());
          swerveDrive->AddPositionEstimate(transEst, units::microsecond_t{array.time - array.value[2]});
        }
      }
    }
    else
    {
      coneEntry.ReadQueue();
    }

    elevator->SetElevatorHeightPID(0, elapsedTime);
    claw->OpenClaw(elapsedTime);
    claw->PIDWrist(2.2, elapsedTime);
    bool splineDone = swerveDrive->FollowTrajectory(timer.Get(), elapsedTime);
    if (splineDone)
    {
      splineSection = 1.5;
      swerveDrive->SetNextTrajectory();
      timer.Reset();
      lastTime = 0;
    }
  }

  if (splineSection == 1.5)
  {
    claw->PIDWrist(2.2, elapsedTime);

    elevator->SetElevatorHeightPID(0, elapsedTime);
    bool clawDone = claw->PIDClaw(-2, elapsedTime);
    swerveDrive->DriveSwervePercent(0, 0, 0);

    if (clawDone)
    {
      splineSection = 2;
      timer.Reset();
      lastTime = 0;
    }
  }

  if (splineSection == 2)
  {
    if (swerveDrive->GetPose().Y() < 3_m)
      elevator->SetElevatorHeightPID(40, elapsedTime);
    else
      elevator->SetElevatorHeightPID(0, elapsedTime);

    claw->PIDClaw(-2, elapsedTime);
    claw->PIDWrist(0.6, elapsedTime);
    bool splineDone = swerveDrive->FollowTrajectory(timer.Get(), elapsedTime);
    if (splineDone)
    {
      splineSection = 2.1;
      swerveDrive->SetNextTrajectory();
    }
  }

  if (splineSection == 2.1)
  {
    bool elevatorDone = elevator->SetElevatorHeightPID(40, elapsedTime);
    if (elevatorDone)
    {
      splineSection = 2.2;
    }
  }

  if (splineSection == 2.2)
  {
    double wristDone = claw->PIDWrist(M_PI / 2 - 0.5, elapsedTime);
    if (wristDone)
    {
      splineSection = 2.3;
    }
  }

  if (splineSection == 2.3)
  {
    claw->PIDWrist(M_PI / 2 - 0.5, elapsedTime);
    claw->OpenClaw(elapsedTime);
    if (claw->ClawEncoderReading() < -5)
    {
      claw->OpenClaw(elapsedTime);
      splineSection = 2.4;
    }
  }

  if (splineSection == 2.4)
  {
    claw->PIDWrist(1, elapsedTime);
    claw->OpenClaw(elapsedTime);
    elevator->SetElevatorHeightPID(0, elapsedTime);
    /*if (elevator->winchEncoderReading() < 30)
    {
      timer.Reset();
      lastTime = 0;
      splineSection = 3;
    } */
  }

  /*
  if (splineSection == 3)
  {
    if (swerveDrive->GetPose().Y() > 5_m)
    {
      for (auto array : coneEntry.ReadQueue())
      {
        double angle = -1 * swerveDrive->GetPose().Rotation().Radians().value();
        if (array.value[0] != 0 || array.value[1] != 0)
        {
          double fieldOrientedX = -1 * (array.value[0] * cos(angle) - array.value[1] * sin(angle));
          double fieldOrientedY = -1 * (array.value[0] * sin(angle) + array.value[1] * cos(angle));
          Translation2d transEst = Translation2d(4.56_m + units::meter_t{fieldOrientedX}, 7_m + units::meter_t{fieldOrientedY});
          frc::SmartDashboard::PutNumber("Cone X Final", transEst.X().value());
          frc::SmartDashboard::PutNumber("Cone Y Final", transEst.Y().value());
          swerveDrive->AddPositionEstimate(transEst, units::microsecond_t{array.time - array.value[2]});
        }
      }
    }
    else
    {
      coneEntry.ReadQueue();
    }

    elevator->SetElevatorHeightPID(0, elapsedTime);
    claw->OpenClaw(elapsedTime);
    claw->PIDWrist(2.2, elapsedTime);
    bool splineDone = swerveDrive->FollowTrajectory(timer.Get(), elapsedTime);
    if (splineDone)
    {
      splineSection = 3.5;
      swerveDrive->SetNextTrajectory();
    }
  }

  if (splineSection == 3.5)
  {
    claw->PIDWrist(2.2, elapsedTime);

    elevator->SetElevatorHeightPID(0, elapsedTime);
    bool clawDone = claw->PIDClaw(-2, elapsedTime);
    swerveDrive->DriveSwervePercent(0, 0, 0);

    if (clawDone)
    {
      splineSection = 2;
      timer.Reset();
      lastTime = 0;
    }
  }

  if (splineSection == 4)
  {
    claw->PIDClaw(-2, elapsedTime);
    claw->PIDWrist(0.6, elapsedTime);
    bool splineDone = swerveDrive->FollowTrajectory(timer.Get(), elapsedTime);
    if (splineDone)
    {
      splineSection = 4.1;
    }
  }

  if (splineSection == 4.1)
  {
    claw->OpenClaw(elapsedTime);
    claw->PIDWrist(0.6, elapsedTime);
    elevator->SetElevatorHeightPID(0, elapsedTime);
  }*/

  lastTime = timer.Get().value();
}

void Robot::TeleopInit()
{
  // Prepare swerve drive odometry
  pigeon_initial = fmod(_pigeon.GetYaw() + STARTING_DRIVE_HEADING, 360);
  swerveDrive->pigeon_initial = pigeon_initial;
  swerveDrive->ResetOdometry(Pose2d(5.22_m, 1.78_m, Rotation2d(3.14_rad)));
  elevator->ResetElevatorEncoder();
  claw->ResetClawEncoder();

  // Reset all our values throughout the code
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

void Robot::TeleopPeriodic()
{
  // Take values from Smartdashboard
  MAX_DRIVE_SPEED = frc::SmartDashboard::GetNumber("MAX DRIVE SPEED", 0.4);
  ELEVATOR_SPEED = frc::SmartDashboard::GetNumber("ELEVATOR_SPEED", 0.1);

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

  // Scale our joystick inputs to our intended max drive speeds
  double FWD_Drive_Speed = joy_lStick_Y * MAX_DRIVE_SPEED;
  double STRAFE_Drive_Speed = joy_lStick_X * MAX_DRIVE_SPEED;
  double Turn_Speed = joy_rStick_X * MAX_SPIN_SPEED;

  // update our timer
  double time = timer.Get().value();
  double elapsedTime = time - lastTime;
  lastTime = time;

  // Slew rate limiting driver input
  lastFwdSpeed += std::clamp(FWD_Drive_Speed - lastFwdSpeed, -1 * MAX_DRIVE_ACCELERATION * elapsedTime,
                             MAX_DRIVE_ACCELERATION * elapsedTime);
  lastStrafeSpeed += std::clamp(STRAFE_Drive_Speed - lastStrafeSpeed, -1 * MAX_DRIVE_ACCELERATION * elapsedTime,
                                MAX_DRIVE_ACCELERATION * elapsedTime);
  lastTurnSpeed += std::clamp(Turn_Speed - lastTurnSpeed, -1 * MAX_SPIN_ACCELERATION * elapsedTime,
                              MAX_SPIN_ACCELERATION * elapsedTime);

  swerveDrive->DriveSwervePercent(lastStrafeSpeed, lastFwdSpeed, lastTurnSpeed);

  // Update our odometry
  double microsecondTime = (double)RobotController::GetFPGATime();
  swerveDrive->UpdateOdometry(units::microsecond_t{microsecondTime});

  // Update our odometry position based on cone data

  for (auto array : coneEntry.ReadQueue())
  {
    double angle = -swerveDrive->GetPose().Rotation().Radians().value();
    if (array.value[0] != 0 || array.value[1] != 0)
    {
      double fieldOrientedX = -1 * (array.value[0] * cos(angle) - array.value[1] * sin(angle));
      double fieldOrientedY = -1 * (array.value[0] * sin(angle) + array.value[1] * cos(angle));
      Translation2d transEst = Translation2d(0_m + units::meter_t{fieldOrientedX}, 1.5_m + units::meter_t{fieldOrientedY});
      frc::SmartDashboard::PutNumber("Cone X Vision", transEst.X().value());
      frc::SmartDashboard::PutNumber("Cone Y Vision", transEst.Y().value());
      //swerveDrive->AddPositionEstimate(transEst, units::microsecond_t{array.time - array.value[2]});
    }
  }
  for (auto array : poseSub.ReadQueue())
  {
    Translation2d poseEst = Translation2d(units::meter_t{array.value[0]}, units::meter_t{array.value[1]});
    frc::SmartDashboard::PutNumber("Vision X", poseEst.X().value());
    frc::SmartDashboard::PutNumber("Vision Y", poseEst.Y().value());
    //swerveDrive->AddPositionEstimate(poseEst, units::microsecond_t{array.time - array.value[4]});
  }

  Pose2d pose = swerveDrive->GetPose();
  double poseArray[] = {pose.X().value(), pose.Y().value(), 0.75, pose.Rotation().Radians().value(), 0};
  curPoseEntry.Set(poseArray);

  // DEBUG INFO
  frc::SmartDashboard::PutNumber("Odometry X", pose.X().value());
  frc::SmartDashboard::PutNumber("Odometry Y", pose.Y().value());
  frc::SmartDashboard::PutNumber("Odometry Theta", pose.Rotation().Degrees().value());
  SmartDashboard::PutNumber("lift encoder", elevator->winchEncoderReading());
  SmartDashboard::PutNumber("wrist encoder", claw->MagEncoderReading());
  SmartDashboard::PutNumber("claw encoder", claw->ClawEncoderReading());

  // LIMELIGHT CODE
  // if (xbox_Drive->GetRightBumper())
  // {
  //  // double offset = coneXEntry.Get(); for cones
  //   double offset = limelight->getTargetX();
  //   bool thing = swerveDrive->StrafeToPole(offset, elapsedTime);
  //   SmartDashboard::PutNumber("thing", thing);
  // }

  // BASIC ELEVATOR CODE
  if (xbox_Drive->GetBButtonPressed())
    elevator->StartPIDLoop();

  if (xbox_Drive->GetYButton())
    elevator->MoveElevatorPercent(0.2);
  else if (xbox_Drive->GetAButton())
    elevator->MoveElevatorPercent(-0.2);
  else if (xbox_Drive->GetBButton())
    elevator->SetElevatorHeightPID(76, elapsedTime);
  else if (xbox_Drive->GetXButton())
    elevator->MoveElevatorPercent(0.05);
  else
    elevator->MoveElevatorPercent(0);

  if (xbox_Drive->GetRightBumper())
    claw->MoveClawPercent(-0.7);
  else if (xbox_Drive->GetRightTriggerAxis() > 0.5)
    claw->MoveClawPercent(0.7);
  else if (xbox_Drive->GetStartButton())
    claw->OpenClaw(elapsedTime);
  else
    claw->MoveClawPercent(0);

  if (xbox_Drive->GetLeftBumper())
    claw->MoveWristPercent(0.8);
  else if (xbox_Drive->GetLeftTriggerAxis() > 0.5)
    claw->MoveWristPercent(-0.6);
  else if (xbox_Drive->GetBackButton())
    claw->PIDWrist(M_PI / 2, elapsedTime);
  else
    claw->MoveWristPercent(0.0);

  /*
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
