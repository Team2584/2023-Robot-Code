// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Setup.h"

#include "Swerve.cpp"
#include "Elevator.cpp"
#include "Limelight.cpp"
#include "Claw.cpp"

#include <fmt/core.h>
#include <frc/livewindow/LiveWindow.h>

#include <frc/smartdashboard/SmartDashboard.h>

double pigeon_initial;
// Our future subsystem objects
SwerveDrive *swerveDrive;
ElevatorLift *elevatorLift;
Limelight *limelight;
Claw *claw;

// To find values from cameras
nt::NetworkTableInstance inst;
shared_ptr<nt::NetworkTable> visionTable;
shared_ptr<nt::NetworkTable> limelightTable;
nt::DoubleArrayTopic poseTopic;
nt::DoubleTopic sanityTopic;
nt::BooleanTopic connectedTopic;
nt::DoubleArrayTopic curPoseTopic;
nt::DoubleArraySubscriber poseSub;
nt::DoubleEntry sanityEntry;
nt::DoubleArrayEntry curPoseEntry;
nt::DoubleTopic polePixelTopic;
nt::DoubleEntry polePixelEntry;
nt::BooleanEntry connectedEntry;
nt::DoubleArrayTopic coneTopic;
nt::DoubleArraySubscriber coneEntry;
nt::BooleanTopic hasConeTopic;
nt::BooleanEntry hasConeEntry;
double currentConeX;
double currentConeY;

// To track time for slew rate and pid controll
frc::Timer timer;
double lastTime = 0;
double lastFwdSpeed = 0;
double lastStrafeSpeed = 0;
double lastTurnSpeed = 0;
double lastElevatorSpeed = 0;
double lastWristSpeed = 0;

// Driver Control Variables
int currentDriverSection = 0;
enum DriverSection
{
  BEGINDRIVING = -2, 
  RESUMEDRIVING = -1,
  DRIVING = 0,
  ZEROING = 1,
  GROUNDPREPAREDTOGRAB = 2,
  GROUNDINTAKING = 3,
  SUBSTATIONINTAKING = 4,
  SCORINGCONE = 5,
  SCORINGCUBE = 6,
  UNTIPCONE = 7
};

// Values to Set with ShuffleBoard
double MAX_DRIVE_SPEED = 0.4;
double MAX_SPIN_SPEED = 0.4;
double ELEVATOR_SPEED = 0.1;

// Cringe Auto Values S**FF
double splineSection = 0;
Rotation2d goalConeGrabAngle;
bool turnt;
bool doneWithPoleAlignment;
double conePlaceXLimelightGoal = 0.19; 
double conePlaceYLimelightGoal = -0.0288;
double conePlaceElevatorGoal = 44;
bool queuePlacingCone = true;
bool autoControllingRobot = false;
bool dpadAutoControl = false;

double startingSanity = 0;

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
  connectedTopic = visionTable->GetBooleanTopic("connected");
  curPoseTopic = visionTable->GetDoubleArrayTopic("curPose");
  polePixelTopic = limelightTable->GetDoubleTopic("polePixel");
  coneTopic = visionTable->GetDoubleArrayTopic("conePos");
  hasConeTopic = visionTable->GetBooleanTopic("hasCone");
  poseSub = poseTopic.Subscribe({});
  sanityEntry = sanityTopic.GetEntry(-1);
  connectedEntry = connectedTopic.GetEntry(false);
  curPoseEntry = curPoseTopic.GetEntry({});
  polePixelEntry = polePixelTopic.GetEntry(1000);
  coneEntry = coneTopic.Subscribe({});
  hasConeEntry = hasConeTopic.GetEntry(false);

  connectedEntry.Set(true);
  startingSanity = sanityEntry.Get();

  // Initializing things
  timer = Timer();

  // Initializing Subsystems
  swerveDrive = new SwerveDrive(&driveFL, &swerveFL, &FLMagEnc, FL_WHEEL_OFFSET, &driveFR, &swerveFR, &FRMagEnc,
                                FR_WHEEL_OFFSET, &driveBR, &swerveBR, &BRMagEnc, BR_WHEEL_OFFSET, &driveBL,
                                &swerveBL, &BLMagEnc, BL_WHEEL_OFFSET, &_pigeon, STARTING_DRIVE_HEADING);

  elevatorLift = new ElevatorLift(&winchL, &winchR, &TOFSensor);
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
  connectedEntry.Set(true);
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

  elevatorLift->ResetElevatorEncoder();
  claw->ResetClawEncoder();
  //swerveDrive->ResetOdometry(Pose2d(2.42_m, 1.85_m, Rotation2d(3.14_rad)));
  swerveDrive->ResetOdometry(Pose2d(4.98_m, 1.85_m, Rotation2d(3.14_rad)));
  swerveDrive->ResetTrajectoryList();
  swerveDrive->InitializeTrajectory("RedRight2GamePiece1");
  swerveDrive->InitializeTrajectory("RedRight2GamePiece2");
  //swerveDrive->InitializeTrajectory("Red1GamePieceBalance");
  swerveDrive->SetNextTrajectory();  
  splineSection = 0;

  // Start our match timer and reset our odometry to the robot's starting position
  lastTime = 0;
  timer.Reset();
  timer.Start();
}

void Robot::AutonomousPeriodic()
{
  /*
  double elapsedTime = timer.Get().value() - lastTime;
  swerveDrive->UpdateOdometry(timer.Get());

  //Reset Timer and prepare to lift elevator
  if (splineSection == 0)
  {
    timer.Reset();
    lastTime = 0;
    splineSection = 1;
  }

  //Drive onto the platform using a spline (basically a path to drive) in pathplanner
  // If you want to edit the spline go into pathplanner.exe on the driver station's desktop or download it yourself from github
  // https://github.com/mjansen4857/pathplanner
  if (splineSection == 1)
  {
    bool splineDone = swerveDrive->FollowTrajectory(timer.Get(), elapsedTime);
    if (splineDone)
    {
      splineSection = 1.5;
      timer.Reset();
      lastTime = 0;
    }
  }

  // When you are done with the spline, balance using your code
  if (splineSection == 1.5)
  {
    elevatorLift->SetElevatorHeightPID(0, elapsedTime);
    claw->PIDWrist(1, elapsedTime);
    claw->OpenClaw(elapsedTime);
    bool balanced = swerveDrive->BalanceOnCharger(elapsedTime);
    // Uncomment code below if you think you want it, otherwise your robot will continue to balance until autonomous ends
    /*
    if (balanced)
    {
      //When you are balanced, stop driving  
      //    If you need your wheels to spin to face left and right here to make sure the robot stays parked we I'll add the code when I show up for the lunch break
      swerveDrive->DriveSwervePercent(0,0,0);
      splineSection = 100;
      timer.Reset();
      lastTime = 0;
    }
  }*/

  SmartDashboard::PutNumber("Spline Section", splineSection);

  double elapsedTime = timer.Get().value() - lastTime;
  swerveDrive->UpdateOdometry(timer.Get());
  swerveDrive->UpdateConeOdometry();
  for (auto array : coneEntry.ReadQueue())
  {
    if ((array.value[0] != 0 || array.value[1] != 0) && array.value[1] > 1)
    {
      double fieldOrientedX = -1 * array.value[0];
      double fieldOrientedY = -1 * array.value[1];
      Translation2d transEst = Translation2d(units::meter_t{fieldOrientedX}, units::meter_t{fieldOrientedY});
      frc::SmartDashboard::PutNumber("Cone X Vision", transEst.X().value());
      frc::SmartDashboard::PutNumber("Cone Y Vision", transEst.Y().value());
      swerveDrive->ResetConeOdometry(Pose2d(transEst, swerveDrive->GetPose().Rotation()));
      currentConeX = -1 * array.value[0];
      currentConeY = -1 * array.value[1];
    }
  }

  if (splineSection == 0)
  {
    doneWithPoleAlignment = false;
    turnt = false;
    swerveDrive->BeginPIDLoop();
    splineSection = 0.5; 
  }

  if (splineSection == 0.5)
  {
    if (!doneWithPoleAlignment)
    {
      claw->PIDWrist(0.9, elapsedTime);
    }
    bool lifted = elevatorLift->SetElevatorHeightPID(78, elapsedTime); 
    bool centered = false;
    if (!turnt)
      turnt = swerveDrive->DriveToPose(Pose2d(swerveDrive->GetPose().Translation(), Rotation2d(180_deg)), elapsedTime);
    if (turnt && elevatorLift->winchEncoderReading() > 30)
    {
      double offsetX = limelight->getTargetX();
      double offsetY = limelight->getTargetY();
      centered = swerveDrive->StrafeToPole(offsetX, offsetY, 0.19, -0.04, elapsedTime);  //0.27, 0.149
    }
    if (centered && lifted)
      doneWithPoleAlignment = true;
    if (doneWithPoleAlignment)
    {
      claw->PIDWrist(M_PI / 2, elapsedTime);
      if (claw->MagEncoderReading() > M_PI / 2 - 0.05)
        claw->OpenClaw(elapsedTime);
      if (claw->ClawEncoderReading() > 5)
        splineSection = 0.9;
    }
  }

  if (splineSection == 0.9)
  {
    claw->PIDWrist(0.5, elapsedTime);
    claw->OpenClaw(elapsedTime);
    if (claw->MagEncoderReading() < 0.75)
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
    if (elevatorLift->winchEncoderReading() < 40)
    {
      timer.Reset();
      lastTime = 0;
      splineSection = 1;
    }
  }

  if (splineSection == 1)
  {
    elevatorLift->SetElevatorHeightPID(0, elapsedTime);
    claw->OpenClaw(elapsedTime);
    claw->PIDWrist(2.1, elapsedTime);
    bool splineDone = swerveDrive->FollowTrajectory(timer.Get(), elapsedTime);
    if (splineDone)
    {
      splineSection = 1.5;
      swerveDrive->SetNextTrajectory();
      swerveDrive->DriveSwervePercent(0,0,0);
      swerveDrive->ResetConeOdometry(Pose2d(0_m, -1_m, Rotation2d(0_deg)));
      turnt = false;
    }
  }

  if (splineSection == 1.5)
  {
    elevatorLift->SetElevatorHeightPID(0, elapsedTime);
    claw->PIDWrist(2.1, elapsedTime);
    if (!turnt)
    {
      double angleGoal = atan2(-currentConeX, -currentConeY);
      turnt = swerveDrive->TurnToPixelCone(angleGoal, elapsedTime);
      goalConeGrabAngle = swerveDrive->GetPose().Rotation();
    }
    SmartDashboard::PutBoolean("turnt", turnt);
    bool atCone = hasConeEntry.Get();
    if (turnt && !atCone)
      atCone = swerveDrive->DriveToPoseConeOdometry(Pose2d(0_m, -0.58_m, goalConeGrabAngle), elapsedTime);
    SmartDashboard::PutBoolean("atCone", atCone);

    if (atCone) 
      claw->CloseClaw(elapsedTime);
    else
      claw->OpenClaw(elapsedTime);

    if (claw->ClawEncoderReading() < 2)
    {
      splineSection = 2; 
      timer.Reset();
      lastTime = 0;
    }
  }

  if (splineSection == 2)
  {
    //claw->CloseClaw(elapsedTime);
    claw->PIDWrist(0.6, elapsedTime);
    bool splineDone = swerveDrive->FollowTrajectory(timer.Get(), elapsedTime);
    if (splineDone)
    {
      splineSection = 2.5;
      doneWithPoleAlignment = false;
      turnt = false;
      swerveDrive->BeginPIDLoop();
    }
  }

  if (splineSection == 2.5)
  {
    if (!doneWithPoleAlignment)
    {
      claw->PIDWrist(0.9, elapsedTime);
    }
    bool lifted = elevatorLift->SetElevatorHeightPID(47, elapsedTime); // 40
    bool centered = false;
    if (!turnt)
      turnt = swerveDrive->DriveToPose(Pose2d(swerveDrive->GetPose().Translation(), Rotation2d(180_deg)), elapsedTime);
    if (turnt && elevatorLift->winchEncoderReading() > 30)
    {
      double offsetX = limelight->getTargetX();
      double offsetY = limelight->getTargetY();
      centered = swerveDrive->StrafeToPole(offsetX, offsetY, 0.19, -0.04, elapsedTime);  //0.27, 0.149
    }
    if (centered && lifted)
      doneWithPoleAlignment = true;
    if (doneWithPoleAlignment)
    {
      claw->PIDWrist(M_PI / 2, elapsedTime);
      if (claw->MagEncoderReading() > M_PI / 2 - 0.05)
        claw->OpenClaw(elapsedTime);
      if (claw->ClawEncoderReading() > 5)
      {
        splineSection = 100;
        swerveDrive->DriveSwervePercent(0,0,0);
        claw->MoveClawPercent(0);
        elevatorLift->MoveElevatorPercent(0);
        claw->MoveWristPercent(0);
      }
    }
  }

  lastTime = timer.Get().value();
}

void Robot::TeleopInit()
{
  // LOWEST POSSIBLE WRIST ALLOWED: M_PI 
  // LOWEST WRIST ALLOWED WHEN ELEVATOR IS DOWN:2.17
  // HIHGEST WRIST ALLOWED: -0.005
  // HIGHEST CLAW : maybe not a great idea
  // LOWEST CLAW: maybe not a great idea
  // Slew rate limiting for drive: maybe not a great idea

  // Prepare swerve drive odometry
  pigeon_initial = fmod(_pigeon.GetYaw() + STARTING_DRIVE_HEADING, 360);
  swerveDrive->pigeon_initial = pigeon_initial;
  swerveDrive->ResetOdometry(Pose2d(0_m, 0_m, Rotation2d(180_deg)));
  elevatorLift->ResetElevatorEncoder();
  claw->ResetClawEncoder();

  // Reset all our values throughout the code
  timer.Reset();
  timer.Start();
  lastTime = 0;

  lastFwdSpeed = 0;
  lastStrafeSpeed = 0;
  lastTurnSpeed = 0;
  lastElevatorSpeed = 0;
  lastWristSpeed = 0;

  currentDriverSection = 0;

  /*
    orchestra.LoadMusic("CHIRP");
    orchestra.AddInstrument(swerveBL);
    orchestra.AddInstrument(driveBL);
    orchestra.Play();
  */
}

void Robot::TeleopPeriodic()
{
  //distanceSensor.SetOversampleBits(8);
  ///SmartDashboard::PutNumber("real distance value", distanceSensor.GetValue());
  // SmartDashboard::PutNumber("average distance value", distanceSensor.GetAverageValue() / 8);

  // SmartDashboard::PutNumber("FL Mag", swerveDrive->FLModule->magEncoder->GetAbsolutePosition());
  // SmartDashboard::PutNumber("FR Mag", swerveDrive->FRModule->magEncoder->GetAbsolutePosition());
  // SmartDashboard::PutNumber("BL Mag", swerveDrive->BLModule->magEncoder->GetAbsolutePosition());
  // SmartDashboard::PutNumber("BR Mag", swerveDrive->BRModule->magEncoder->GetAbsolutePosition());
  // SmartDashboard::PutNumber("Pigeon", _pigeon.GetYaw());

  // update our timer
  double time = timer.Get().value();
  double elapsedTime = time - lastTime;
  lastTime = time;

  // Update our odometry
  double microsecondTime = (double)RobotController::GetFPGATime();
  swerveDrive->UpdateOdometry(units::microsecond_t{microsecondTime});
  swerveDrive->UpdateConeOdometry();
  swerveDrive->UpdateTagOdometry();

  for (auto array : coneEntry.ReadQueue())
  {
    double angle = -swerveDrive->GetPose().Rotation().Radians().value();
    if ((array.value[0] != 0 || array.value[1] != 0) && array.value[1] > 1)
    {
      double fieldOrientedX = -1 * array.value[0];
      double fieldOrientedY = -1 * array.value[1];
      Translation2d transEst = Translation2d(units::meter_t{fieldOrientedX}, units::meter_t{fieldOrientedY});
      frc::SmartDashboard::PutNumber("Cone X Vision", transEst.X().value());
      frc::SmartDashboard::PutNumber("Cone Y Vision", transEst.Y().value());
      swerveDrive->ResetConeOdometry(Pose2d(transEst, swerveDrive->GetPose().Rotation()));
      currentConeX = -1 * array.value[0];
      currentConeY = -1 * array.value[1];
    }
  }
  for (auto array : poseSub.ReadQueue())
  {
    Translation2d poseEst = Translation2d(units::meter_t{array.value[0]}, units::meter_t{array.value[1]});
    frc::SmartDashboard::PutNumber("Tag Vision X", poseEst.X().value());
    frc::SmartDashboard::PutNumber("Tag Vision Y", poseEst.Y().value());
    swerveDrive->ResetTagOdometry(Pose2d(poseEst, swerveDrive->GetPose().Rotation()));
  }

  // Update our odometry position based on cone data

  /*
    for (auto array : coneEntry.ReadQueue())
    {
      double angle = -swerveDrive->GetPose().Rotation().Radians().value();
      if (array.value[0] != 0 || array.value[1] != 0)
      {
        frc::SmartDashboard::PutNumber("Pre Rot Cone X Vision", array.value[0]);
        frc::SmartDashboard::PutNumber("Pre Rot Cone Y Vision", array.value[1]);
        double fieldOrientedX = -1 * (array.value[0] * cos(angle) - array.value[1] * sin(angle));
        double fieldOrientedY = -1 * (array.value[0] * sin(angle) + array.value[1] * cos(angle));
        frc::SmartDashboard::PutNumber("Almost Cone X Vision", fieldOrientedX);
        frc::SmartDashboard::PutNumber("Almost Cone Y Vision", fieldOrientedY);
        Translation2d transEst = Translation2d(4.56_m + units::meter_t{fieldOrientedX}, 7_m + units::meter_t{fieldOrientedY});
        frc::SmartDashboard::PutNumber("Cone X Vision", transEst.X().value());
        frc::SmartDashboard::PutNumber("Cone Y Vision", transEst.Y().value());
        swerveDrive->AddPositionEstimate(transEst, units::microsecond_t{array.time - array.value[2]});
      }
    }
    for (auto array : poseSub.ReadQueue())
    {
      Translation2d poseEst = Translation2d(units::meter_t{array.value[0]}, units::meter_t{array.value[1]});
      frc::SmartDashboard::PutNumber("Vision X", poseEst.X().value());
      frc::SmartDashboard::PutNumber("Vision Y", poseEst.Y().value());
      SmartDashboard::PutNumber("time array offest", array.value[4]);
      SmartDashboard::PutNumber("Time of mesurement", array.time);
      SmartDashboard::PutNumber("system Time", microsecondTime);
      SmartDashboard::PutNumber("possible system time", Timer::GetFPGATimestamp().value());
      SmartDashboard::PutNumber("position estimate time", units::microsecond_t{array.time - array.value[4]}.value());
      swerveDrive->AddPositionEstimate(poseEst, units::microsecond_t{array.time - array.value[4]});
    }*/



  // DEBUG INFO
  Pose2d pose = swerveDrive->GetPose();
  double poseArray[] = {pose.X().value(), pose.Y().value(), 0.75, pose.Rotation().Radians().value(), 0};
  curPoseEntry.Set(poseArray);
  frc::SmartDashboard::PutNumber("Odometry X", pose.X().value());
  frc::SmartDashboard::PutNumber("Odometry Y", pose.Y().value());
  frc::SmartDashboard::PutNumber("Odometry Theta", swerveDrive->GetPose().Rotation().Degrees().value());
  SmartDashboard::PutNumber("lift encoder", elevatorLift->winchEncoderReading());
  SmartDashboard::PutNumber("wrist encoder", claw->MagEncoderReading());
  SmartDashboard::PutNumber("claw encoder", claw->ClawEncoderReading());
  SmartDashboard::PutNumber("Driver Section", currentDriverSection);

  double joy_lStick_Y, joy_lStick_X, joy_rStick_X;
  // Find controller input
  joy_lStick_Y = xbox_Drive->GetLeftY();
  joy_lStick_X = xbox_Drive->GetLeftX();
  joy_rStick_X = xbox_Drive->GetRightX();
  joy_lStick_Y *= -1;

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

  // Speed up or Slow Down the Drive
  if (xbox_Drive->GetRightBumper() && elevatorLift->winchEncoderReading() < 30)
  {
    MAX_DRIVE_SPEED = 0.9;
    MAX_SPIN_SPEED = 0.9;
  }
  else if (xbox_Drive->GetLeftBumper())
  {
    MAX_DRIVE_SPEED = 0.2;
    MAX_SPIN_SPEED = 0.2;
  }
  else
  {
    MAX_DRIVE_SPEED = 0.4;
    MAX_SPIN_SPEED = 0.4;
  }

  // Scale our joystick inputs to our intended max drive speeds
  double FWD_Drive_Speed = joy_lStick_Y * MAX_DRIVE_SPEED;
  double STRAFE_Drive_Speed = joy_lStick_X * MAX_DRIVE_SPEED;
  double Turn_Speed = joy_rStick_X * MAX_SPIN_SPEED;


  // Slew rate limiting driver input
  lastFwdSpeed += std::clamp(FWD_Drive_Speed - lastFwdSpeed, -1 * MAX_DRIVE_ACCELERATION * elapsedTime,
                            MAX_DRIVE_ACCELERATION * elapsedTime);
  lastStrafeSpeed += std::clamp(STRAFE_Drive_Speed - lastStrafeSpeed, -1 * MAX_DRIVE_ACCELERATION * elapsedTime,
                                MAX_DRIVE_ACCELERATION * elapsedTime);
  lastTurnSpeed += std::clamp(Turn_Speed - lastTurnSpeed, -1 * MAX_SPIN_ACCELERATION * elapsedTime,
                              MAX_SPIN_ACCELERATION * elapsedTime);


  switch(currentDriverSection)
  {
    case BEGINDRIVING:
      lastFwdSpeed = 0;
      lastStrafeSpeed = 0;
      lastTurnSpeed = 0;
    case RESUMEDRIVING:
      lastElevatorSpeed = 0;
      lastWristSpeed = 0;
      currentDriverSection = DRIVING;
    case DRIVING:
    {
      swerveDrive->DriveSwervePercent(lastStrafeSpeed, lastFwdSpeed, lastTurnSpeed);

      double elevSpeed = 0;
      if (xbox_Drive2->GetLeftY() > 0.3)
        elevSpeed = -0.2;
      else if (xbox_Drive2->GetLeftY() < -0.3)
        elevSpeed = 0.3;
      else
        elevSpeed = 0;
        
      lastElevatorSpeed += std::clamp(elevSpeed - lastElevatorSpeed, -1 * MAX_ELEV_ACCELERATION * elapsedTime,
                                      MAX_ELEV_ACCELERATION * elapsedTime);

      bool elevatorPIDWrist = false;
      if (elevatorLift->winchEncoderReading() < 20 && lastElevatorSpeed > 0 && claw->MagEncoderReading() < 0.3)
      {
        lastElevatorSpeed = 0;
        elevatorPIDWrist = true;
        claw->PIDWrist(0.4, elapsedTime);
      }    
      else if (elevatorLift->winchEncoderReading() < 30 && lastElevatorSpeed < 0 && claw->MagEncoderReading() < 0.3)
      {
        lastElevatorSpeed = 0;
        elevatorPIDWrist = true;
        claw->PIDWrist(0.4, elapsedTime);
      }
      else if (elevatorLift->winchEncoderReading() < 30 && lastElevatorSpeed < 0 && claw->MagEncoderReading() > 2.2)
      {
        lastElevatorSpeed = 0;
        elevatorPIDWrist = true;
        claw->PIDWrist(2.1, elapsedTime);
      }
      else if (elevatorLift->winchEncoderReading() > 82.5 && lastElevatorSpeed > 0)
      {
        lastElevatorSpeed = 0;
      }
      else if (elevatorLift->winchEncoderReading() < -1 && lastElevatorSpeed < 0)
      {
        //lastElevatorSpeed = 0;
      }
      elevatorLift->MoveElevatorPercent(lastElevatorSpeed);


      double wristSpeed = 0;
      if (xbox_Drive2->GetRightBumper())
        wristSpeed = 0.2;
      else if (xbox_Drive2->GetLeftBumper())
        wristSpeed = -0.2;
      else
        wristSpeed = 0;

      lastWristSpeed += std::clamp(wristSpeed - lastWristSpeed, -1 * MAX_WRIST_ACCELERATION * elapsedTime,
                                      MAX_WRIST_ACCELERATION * elapsedTime);
    

      if (claw->MagEncoderReading() < 0.3 && lastWristSpeed > 0)
        lastWristSpeed = 0;
      else if (elevatorLift->winchEncoderReading() < 20 && claw->MagEncoderReading() > 2.17 && lastWristSpeed < 0)
        lastWristSpeed = 0;
      else if (claw->MagEncoderReading() > 2.75 && lastWristSpeed < 0)
        lastWristSpeed = 0;
      
      if (!elevatorPIDWrist)
        claw->MoveWristPercent(lastWristSpeed); 

      if (xbox_Drive2->GetLeftTriggerAxis() > 0.5)
        claw->MoveClawPercent(0.5);  
      else if (xbox_Drive2->GetRightTriggerAxis() > 0.5)
        claw->MoveClawPercent(-0.5);
      else
        claw->MoveClawPercent(0);

      //Reset All Encoder and Gyro Values
      if (xbox_Drive->GetStartButton() && xbox_Drive->GetBackButton())
      {
        swerveDrive->ResetOdometry(Pose2d(0_m, 0_m, Rotation2d(0_deg)));
        elevatorLift->ResetElevatorEncoder();
        claw->ResetClawEncoder();
      }

      // Trigger Autonomous Commands
      if (xbox_Drive2->GetBButtonPressed())
      {
        // Low Post
        conePlaceXLimelightGoal = 0.19; 
        conePlaceYLimelightGoal = -0.04;
        conePlaceElevatorGoal = 44;  
        doneWithPoleAlignment = false;
        turnt = false;
        swerveDrive->BeginPIDLoop();
        currentDriverSection = SCORINGCONE;
      } 
      else if (xbox_Drive2->GetYButtonPressed())
      {
        //High Post
        conePlaceXLimelightGoal = 0.19; 
        conePlaceYLimelightGoal = -0.04;
        conePlaceElevatorGoal = 78;  
        doneWithPoleAlignment = false;
        turnt = false;
        swerveDrive->BeginPIDLoop();
        currentDriverSection = SCORINGCONE;
      }
      else if (xbox_Drive2->GetXButtonPressed())
      {
        //Low Cube
        conePlaceYLimelightGoal= 0.91;
        conePlaceXLimelightGoal = 0.075;
        conePlaceElevatorGoal = 44;  
        doneWithPoleAlignment = false;
        turnt = false;
        swerveDrive->BeginPIDLoop();
        currentDriverSection = SCORINGCUBE;
      }
      else if (xbox_Drive2->GetAButtonPressed())
      {
        //High Cube
        conePlaceYLimelightGoal= 0.91;
        conePlaceXLimelightGoal = 0.075;
        conePlaceElevatorGoal = 78;  
        doneWithPoleAlignment = false;
        turnt = false;
        swerveDrive->BeginPIDLoop();
        currentDriverSection = SCORINGCUBE;
      }
      else if (xbox_Drive2->GetBackButtonPressed())
      {
        currentDriverSection = ZEROING;
      }
      else if (xbox_Drive2->GetPOV() == 0 || xbox_Drive2->GetPOV() == 45 || xbox_Drive2->GetPOV() == 315)
      {
        currentDriverSection = SUBSTATIONINTAKING;
      }
      else if (xbox_Drive2->GetPOV() == 180 || xbox_Drive2->GetPOV() == 225 || xbox_Drive2->GetPOV() == 135)
      {
        // nothing yet
      }
      else if (xbox_Drive->GetRightTriggerAxis() > 0.5)
      {
        currentDriverSection = GROUNDINTAKING;
      }
      else if (xbox_Drive->GetLeftTriggerAxis() > 0.5)
      {
        currentDriverSection = GROUNDPREPAREDTOGRAB;
      }
      else if (xbox_Drive2->GetRightStickButtonPressed())
      {
        currentDriverSection = UNTIPCONE;
      }
      break;
    }

    case SCORINGCONE:
    {
      claw->MoveClawPercent(0);

      if (!doneWithPoleAlignment)
      {
        claw->PIDWrist(0.9, elapsedTime);
      }
      bool lifted = elevatorLift->SetElevatorHeightPID(conePlaceElevatorGoal, elapsedTime);
      bool centered = false;

      if (!turnt)
        turnt = swerveDrive->DriveToPose(Pose2d(swerveDrive->GetPose().Translation(), Rotation2d(180_deg)), elapsedTime);

      if (turnt && elevatorLift->winchEncoderReading() > 15)
      {
        double offsetX = limelight->getTargetX();
        double offsetY = limelight->getTargetY();
        centered = swerveDrive->StrafeToPole(offsetX, offsetY, conePlaceXLimelightGoal, conePlaceYLimelightGoal, elapsedTime);  //0.27, 0.149
      }
      if (centered && lifted)
        doneWithPoleAlignment = true;
      if (doneWithPoleAlignment)
      {
        claw->PIDWrist(M_PI / 2, elapsedTime);
      }

      if (xbox_Drive2->GetYButtonReleased() || xbox_Drive2->GetBButtonReleased())
      {
        currentDriverSection = BEGINDRIVING;
      }
      break;
    }

    case SCORINGCUBE:
    {
      claw->MoveClawPercent(0);

      if (!doneWithPoleAlignment)
      {
        claw->PIDWrist(0.9, elapsedTime);
      }
      bool lifted = elevatorLift->SetElevatorHeightPID(conePlaceElevatorGoal, elapsedTime);
      bool centered = false;
      if (!turnt)
        turnt = swerveDrive->DriveToPose(Pose2d(swerveDrive->GetPose().Translation(), Rotation2d(180_deg)), elapsedTime);
      if (turnt && elevatorLift->winchEncoderReading() > 15)
      {
        centered = swerveDrive->DriveToPoseTag(Pose2d(units::meter_t{conePlaceXLimelightGoal}, units::meter_t{conePlaceYLimelightGoal}, Rotation2d(180_deg)), elapsedTime); 
      }
      if (centered && lifted)
        doneWithPoleAlignment = true;
      if (doneWithPoleAlignment)
      {
        claw->PIDWrist(M_PI / 2, elapsedTime);
      }

      if (xbox_Drive2->GetAButtonReleased() || xbox_Drive2->GetXButtonReleased())
      {
        currentDriverSection = BEGINDRIVING;
      }
      break;
    }

    case ZEROING:
    {
      swerveDrive->DriveSwervePercent(lastStrafeSpeed, lastFwdSpeed, lastTurnSpeed);
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      claw->MoveClawPercent(0);
      if (elevatorLift->winchEncoderReading() < 5)
        claw->PIDWrist(0.3, elapsedTime);
      else
        claw->PIDWrist(1, elapsedTime);

      if (xbox_Drive2->GetBackButtonReleased())
         currentDriverSection = RESUMEDRIVING;
      break;
    }

    case GROUNDPREPAREDTOGRAB:
    {
      swerveDrive->DriveSwervePercent(lastStrafeSpeed, lastFwdSpeed, lastTurnSpeed);
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      claw->PIDWrist(2.1, elapsedTime);
      claw->OpenClaw(elapsedTime);
      
      if (xbox_Drive->GetLeftTriggerAxis() < 0.2)
        currentDriverSection = RESUMEDRIVING;
      break;
    }

    case GROUNDINTAKING:
    {
      swerveDrive->DriveSwervePercent(lastStrafeSpeed, lastFwdSpeed, lastTurnSpeed);
      bool closed = claw->CloseClaw(elapsedTime);
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      if (closed && elevatorLift->winchEncoderReading() < 5)
        claw->PIDWrist(0.3, elapsedTime);
      else
        claw->PIDWrist(2.1, elapsedTime);

      if (xbox_Drive->GetRightTriggerAxis() < 0.2)
        currentDriverSection = RESUMEDRIVING; 
      break;
    }

    case SUBSTATIONINTAKING:
    {
      swerveDrive->DriveSwervePercent(lastStrafeSpeed, lastFwdSpeed, lastTurnSpeed);
      elevatorLift->SetElevatorHeightPID(50, elapsedTime);
      claw->PIDWrist(M_PI / 2, elapsedTime);
      claw->OpenClaw(elapsedTime);
      
      if (xbox_Drive2->GetPOV() == 0)
        currentDriverSection = RESUMEDRIVING; // BEGIN DRIVING once using april tags to align
      break;
    }

    case UNTIPCONE:
    {
      swerveDrive->DriveSwervePercent(lastStrafeSpeed, lastFwdSpeed, lastTurnSpeed);
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      claw->PIDWrist(1.866, elapsedTime);
      claw->OpenClaw(elapsedTime);
      if (xbox_Drive2->GetRightStickButtonReleased())
        currentDriverSection = RESUMEDRIVING;  
      break;
    }
  } 

  /*
  if (xbox_Drive->GetXButtonPressed())
  {
    turnt = false;
  }
  if (xbox_Drive->GetXButton())
  {
    if (!turnt)
    {
      double angleGoal = atan2(-currentConeX, -currentConeY);
      SmartDashboard::PutNumber("cone angle", angleGoal);
      turnt = swerveDrive->TurnToPixelCone(angleGoal, elapsedTime);
      goalConeGrabAngle = swerveDrive->GetPose().Rotation();
    }
    bool atCone = hasConeEntry.Get();
    if (turnt && !atCone)
      atCone = swerveDrive->DriveToPoseConeOdometry(Pose2d(0_m, -0.58_m, goalConeGrabAngle), elapsedTime);
    if (atCone)
      claw->CloseClaw(elapsedTime);
  }*/

  limelight->getTargetX();
  limelight->getTargetY();
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
