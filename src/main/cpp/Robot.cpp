// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Setup.h"

#include "Swerve.cpp"
#include "Elevator.cpp"
#include "Limelight.cpp"
#include "Claw.cpp"
#include "LEDLights.cpp"

#include <fmt/core.h>
#include <frc/livewindow/LiveWindow.h>

#include <frc/smartdashboard/SmartDashboard.h>

double pigeon_initial;
// Our future subsystem objects
SwerveDrive *swerveDrive;
ElevatorLift *elevatorLift;
Limelight *limelight;
Claw *claw;
LEDLights *lights;

// To find values from cameras
nt::NetworkTableInstance inst;
shared_ptr<nt::NetworkTable> visionTable;
shared_ptr<nt::NetworkTable> limelightTable;
nt::DoubleArrayTopic cubeTagTopic;
nt::DoubleArraySubscriber cubeTagSub;
nt::DoubleArrayTopic substationTagTopic;
nt::DoubleArraySubscriber substationTagSub;
nt::DoubleTopic sanityTopic;
nt::BooleanTopic connectedTopic;
nt::DoubleEntry sanityEntry;
nt::DoubleTopic polePixelTopic;
nt::DoubleEntry polePixelEntry;
nt::BooleanEntry connectedEntry;
nt::DoubleArrayTopic coneTopic;
nt::DoubleArraySubscriber coneEntry;
nt::BooleanTopic seeConesTopic;
nt::BooleanEntry seeConesEntry;
nt::BooleanTopic seeCubesTopic;
nt::BooleanEntry seeCubesEntry;
nt::BooleanTopic seeCubeTagsTopic;
nt::BooleanEntry seeCubeTagsEntry;
nt::BooleanTopic seeSubstationTagsTopic;
nt::BooleanEntry seeSubstationTagsEntry;
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
  UNTIPCONE = 7,
  BALANCE = 8
};

// Values to Set with ShuffleBoard
double MAX_DRIVE_SPEED = 0.4;
double MAX_SPIN_SPEED = 0.4;
double ELEVATOR_SPEED = 0.1;
double MAX_DRIVE_ACCELERATION = 1;    //max change in percent per second
double MAX_SPIN_ACCELERATION = 1;

// Cringe Auto Values S**FF
double splineSection = 0;
Rotation2d goalConeGrabAngle;
bool turnt;
bool doneWithPoleAlignment;
double conePlaceXLimelightGoal = 0.19; 
double conePlaceYLimelightGoal = -0.0288;
double conePlaceElevatorGoal = 44;
bool placingHigh = false;
bool queuePlacingCone = true;
bool centeredOnSubstation = false;
bool coneInClaw = false;
bool leftSubstation = false;
bool clawFinishedOpening = false;
bool canSeeCubeTag = false;

double startingSanity = 0;

void Robot::RobotInit()
{
  // Set all Values from Shuffleboard (Smartdashboard but cooler)
  frc::SmartDashboard::PutNumber("MAX DRIVE SPEED", 0.4);
  frc::SmartDashboard::PutNumber("ELEVATOR_SPEED", 0.1);

  // Autonomous Choosing
  m_chooser.SetDefaultOption(kAutoRR2GO, kAutoRR2GO);
  m_chooser.AddOption(kAutoRL2GO, kAutoRL2GO);
    m_chooser.AddOption(kAutoRR1GOB, kAutoRR1GOB);
  m_chooser.AddOption(kAutoBL2GO, kAutoBL2GO);
  m_chooser.AddOption(kAutoBR2GO, kAutoBR2GO);
  m_chooser.AddOption(kAutoBL1GOB, kAutoBL1GOB);
  m_chooser.AddOption(kAutoCB, kAutoCB);
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
  cubeTagTopic = visionTable->GetDoubleArrayTopic("cubeTag");
  substationTagTopic = visionTable->GetDoubleArrayTopic("substationTag");
  sanityTopic = visionTable->GetDoubleTopic("sanitycheck");
  connectedTopic = visionTable->GetBooleanTopic("connected");
  polePixelTopic = limelightTable->GetDoubleTopic("polePixel");
  coneTopic = visionTable->GetDoubleArrayTopic("conePos");
  seeConesTopic = visionTable->GetBooleanTopic("seeCones");
  seeCubesTopic = visionTable->GetBooleanTopic("seeCubes");
  seeCubeTagsTopic = visionTable->GetBooleanTopic("seeCubeTags");
  seeSubstationTagsTopic = visionTable->GetBooleanTopic("seeSubstationTags");
  cubeTagSub = cubeTagTopic.Subscribe({});
  substationTagSub = substationTagTopic.Subscribe({});
  sanityEntry = sanityTopic.GetEntry(-1);
  connectedEntry = connectedTopic.GetEntry(false);
  polePixelEntry = polePixelTopic.GetEntry(1000);
  coneEntry = coneTopic.Subscribe({});
  seeCubesEntry = seeCubesTopic.GetEntry(true);
  seeConesEntry = seeConesTopic.GetEntry(true);
  seeCubeTagsEntry = seeCubeTagsTopic.GetEntry(true);
  seeSubstationTagsEntry = seeSubstationTagsTopic.GetEntry(true);

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
  lights  = new LEDLights(&lightController);
  lights->SetLED();
  limelight->TurnOffLimelight();
  SmartDashboard::PutBoolean("code pushed", true);
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
  m_autoSelected = m_chooser.GetSelected();
  fmt::print("Auto selected: {}\n", m_autoSelected);

  swerveDrive->ResetTrajectoryList();
  elevatorLift->ResetElevatorEncoder();
  claw->ResetClawEncoder();
  splineSection = 0;

  if (m_autoSelected == kAutoRR2GO)
  {
    swerveDrive->SetAllianceColorRed();
    swerveDrive->ResetOdometry(Pose2d(4.98_m, 1.85_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("RedRight2GamePiece1");
    swerveDrive->InitializeTrajectory("RedRight2GamePiece2");
    swerveDrive->SetNextTrajectory();  
  }
  else if (m_autoSelected == kAutoRL2GO)
  {
    swerveDrive->SetAllianceColorRed();
    swerveDrive->ResetOdometry(Pose2d(0.63_m, 1.79_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("RedLeft2GamePiece1");
    swerveDrive->InitializeTrajectory("RedLeft2GamePiece2");
    swerveDrive->SetNextTrajectory();  
  }
  else if (m_autoSelected == kAutoBR2GO)
  {
    swerveDrive->SetAllianceColorBlue();
    swerveDrive->ResetOdometry(Pose2d(7.56_m, 1.83_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("BlueRight2GamePiece1");
    swerveDrive->InitializeTrajectory("BlueRight2GamePiece2");
    swerveDrive->SetNextTrajectory();      
  }
  else if (m_autoSelected == kAutoBL2GO)
  {
    swerveDrive->SetAllianceColorBlue();
    swerveDrive->ResetOdometry(Pose2d(3.25_m, 1.84_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("BlueLeft2GamePiece1");
    swerveDrive->InitializeTrajectory("BlueLeft2GamePiece2");
    swerveDrive->SetNextTrajectory();      
  }
  else if (m_autoSelected == kAutoRR1GOB)
  {
    swerveDrive->SetAllianceColorRed();
    swerveDrive->ResetOdometry(Pose2d(4.98_m, 1.85_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("RedRight2GamePiece1");
    swerveDrive->InitializeTrajectory("RedRight1GamePieceBalance2");
    swerveDrive->SetNextTrajectory();  
  }
  else if (m_autoSelected == kAutoBL1GOB)
  {
    swerveDrive->SetAllianceColorBlue();
    swerveDrive->ResetOdometry(Pose2d(3.25_m, 1.84_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("BlueLeft2GamePiece1");
    swerveDrive->InitializeTrajectory("BlueLeft1GamePieceBalance2");
    swerveDrive->SetNextTrajectory();      
  }
  else if (m_autoSelected == kAutoCB)
  {
    swerveDrive->SetAllianceColorRed();  
    swerveDrive->ResetOdometry(Pose2d(2.42_m, 1.85_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("Place and Balance");
    swerveDrive->SetNextTrajectory();      
  }


  // Start our match timer and reset our odometry to the robot's starting position
  lastTime = 0;
  timer.Reset();
  timer.Start();

  seeConesEntry.Set(true);
  seeCubesEntry.Set(false);
  seeCubeTagsEntry.Set(false);
  seeSubstationTagsEntry.Set(false);
}

void Robot::AutonomousPeriodic()
{
  SmartDashboard::PutNumber("Spline Section", splineSection);

  if (claw->ConeInClaw() && claw->ClawEncoderReading() > 8)
    lights->SetLED("green");
  else
    lights->SetLED();

  if (m_autoSelected == kAutoRR2GO || m_autoSelected == kAutoRL2GO || m_autoSelected == kAutoBL2GO || m_autoSelected == kAutoBR2GO)
  {
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
      limelight->TurnOnLimelight();
    }

    if (splineSection == 0.5)
    {
      if (!doneWithPoleAlignment)
      {
        claw->PIDWrist(0.9, elapsedTime);
      }
      bool lifted = elevatorLift->SetElevatorHeightPID(80, elapsedTime); 
      bool centered = false;
      if (!turnt)
        turnt = swerveDrive->DriveToPose(Pose2d(swerveDrive->GetPose().Translation(), Rotation2d(180_deg)), elapsedTime);
      if (turnt && elevatorLift->winchEncoderReading() > 30)
      {
        double offsetX = limelight->getTargetX();
        double offsetY = limelight->getTargetY();
        centered = swerveDrive->StrafeToPole(offsetX, offsetY, 0.19, -0.04, elapsedTime);  //0.27, 0.149
      }
      SmartDashboard::PutBoolean("cenetered", centered);
      SmartDashboard::PutBoolean("lifted", lifted);
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
        coneInClaw = false;
      }
    }

    if (splineSection == 1.5)
    {
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      claw->PIDWrist(2.1, elapsedTime);

      if (swerveDrive->GetConeOdometryPose().Y().value() < 1.2)
        seeConesEntry.Set(false);

      if (!turnt)
      {
        double angleGoal = atan2(-currentConeX, -currentConeY);
        turnt = swerveDrive->TurnToPixelCone(angleGoal, elapsedTime);
        goalConeGrabAngle = swerveDrive->GetPose().Rotation();
      }
      SmartDashboard::PutBoolean("turnt", turnt);
      if (turnt)
        coneInClaw = claw->ConeInClaw();
      
      if (turnt && !coneInClaw)
        coneInClaw = swerveDrive->DriveToPoseConeOdometry(Pose2d(0_m, -0.58_m, goalConeGrabAngle), elapsedTime);
      else if (turnt)
        swerveDrive->DriveSwervePercent(0,0,0);

      SmartDashboard::PutBoolean("atCone", coneInClaw);

      bool clawClosed = false;
      if (coneInClaw) 
        clawClosed = claw->CloseClaw(elapsedTime);
      else
        claw->OpenClaw(elapsedTime);

      if (clawClosed)
      {
        splineSection = 2; 
        timer.Reset();
        lastTime = 0;
      }
    }

    if (splineSection == 2)
    {
      claw->MoveClawPercent(0);
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
      bool lifted = elevatorLift->SetElevatorHeightPID(47, elapsedTime); 
      bool centered = false;
      if (!turnt)
        turnt = swerveDrive->DriveToPose(Pose2d(swerveDrive->GetPose().Translation(), Rotation2d(180_deg)), elapsedTime);
      if (turnt && elevatorLift->winchEncoderReading() > 30)
      {
        double offsetX = limelight->getTargetX();
        double offsetY = limelight->getTargetY();
        centered = swerveDrive->StrafeToPole(offsetX, offsetY, 0.19, -0.04, elapsedTime); 
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
  }
  else if (m_autoSelected == kAutoBL1GOB || m_autoSelected == kAutoRR1GOB)
  {
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
      limelight->TurnOnLimelight();
      splineSection = 0.5; 
    }

    if (splineSection == 0.5)
    {
      if (!doneWithPoleAlignment)
      {
        claw->PIDWrist(0.9, elapsedTime);
      }
      bool lifted = elevatorLift->SetElevatorHeightPID(80, elapsedTime); 
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
        claw->BeginClawPID();
        turnt = false;
        coneInClaw = false;
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
      if (turnt)
        coneInClaw = claw->ConeInClaw();
      
      if (turnt && !coneInClaw)
        coneInClaw = swerveDrive->DriveToPoseConeOdometry(Pose2d(0_m, -0.58_m, goalConeGrabAngle), elapsedTime);
      else if (turnt)
        swerveDrive->DriveSwervePercent(0,0,0);
        
      SmartDashboard::PutBoolean("atCone", coneInClaw);

      bool clawClosed = false;
      if (coneInClaw) 
        clawClosed = claw->CloseClaw(elapsedTime);
      else
        claw->OpenClaw(elapsedTime);

      if (clawClosed)
      {
        splineSection = 2; 
        timer.Reset();
        lastTime = 0;
      }
    }

    if (splineSection == 2)
    {
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      claw->MoveClawPercent(0);
      claw->PIDWrist(0.6, elapsedTime);
      bool splineDone = swerveDrive->FollowTrajectory(timer.Get(), elapsedTime);
      if (splineDone)
      {
        splineSection = 2.5;
        swerveDrive->DriveSwervePercent(0,0,0);
      }
    } 

    if (splineSection == 2.5)
    {
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      claw->PIDWrist(0.6, elapsedTime);
      claw->MoveClawPercent(0);
      swerveDrive->BalanceOnCharger(elapsedTime);
    }
  }
  else if (m_autoSelected == kAutoCB)
  {
    double elapsedTime = timer.Get().value() - lastTime;
    swerveDrive->UpdateOdometry(timer.Get());

    if (splineSection == 0)
    {
      limelight->TurnOnLimelight();
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
      bool lifted = elevatorLift->SetElevatorHeightPID(80, elapsedTime); 
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
      if (elevatorLift->winchEncoderReading() < 5)
      {
        timer.Reset();
        lastTime = 0;
        splineSection = 1;
      }
    }

    //Drive onto the platform using a spline (basically a path to drive) in pathplanner
    // If you want to edit the spline go into pathplanner.exe on the driver station's desktop or download it yourself from github
    // https://github.com/mjansen4857/pathplanner
    if (splineSection == 1)
    {
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      claw->PIDWrist(0.6, elapsedTime);
      claw->OpenClaw(elapsedTime);
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
      claw->PIDWrist(0.6, elapsedTime);
      claw->OpenClaw(elapsedTime);
      swerveDrive->BalanceOnCharger(elapsedTime);
    }
  }

  lastTime = timer.Get().value();
}

void Robot::TeleopInit()
{
  // Prepare swerve drive odometry
  // pigeon_initial = fmod(_pigeon.GetYaw() + STARTING_DRIVE_HEADING, 360);
  // swerveDrive->pigeon_initial = pigeon_initial;
  // swerveDrive->ResetOdometry(Pose2d(0_m, 0_m, Rotation2d(180_deg)));
  // elevatorLift->ResetElevatorEncoder();
  // claw->ResetClawEncoder();

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

  seeConesEntry.Set(true);
  seeCubesEntry.Set(false);
  seeCubeTagsEntry.Set(true);
  seeSubstationTagsEntry.Set(false);

  SmartDashboard::PutBoolean("code pushed", true);

  /*
    orchestra.LoadMusic("CHIRP");
    orchestra.AddInstrument(swerveBL);
    orchestra.AddInstrument(driveBL);
    orchestra.Play();
  */
}

void Robot::TeleopPeriodic()
{
  // SmartDashboard::PutNumber("FL Mag", swerveDrive->FLModule->magEncoder->GetAbsolutePosition());
  // SmartDashboard::PutNumber("FR Mag", swerveDrive->FRModule->magEncoder->GetAbsolutePosition());
  // SmartDashboard::PutNumber("BL Mag", swerveDrive->BLModule->magEncoder->GetAbsolutePosition());
  // SmartDashboard::PutNumber("BR Mag", swerveDrive->BRModule->magEncoder->GetAbsolutePosition());
  // SmartDashboard::PutNumber("Pigeon", _pigeon.GetYaw());

  if (claw->ClawEncoderReading() > 11 && !claw->ConeInClaw())
    lights->SetLED("blue");
  else if (claw->ClawEncoderReading() > 11 && claw->ConeInClaw())
    lights->SetLED("yellow");
  else if (claw->ClawEncoderReading() < 5 && claw->ConeInClaw())
    lights->SetLED("green");
  else if (claw->ClawEncoderReading() < 5 && !claw->ConeInClaw())
    lights->SetLED("red");
  else
    lights->SetLED();
  SmartDashboard::PutBoolean("cone in claw", claw->ConeInClaw());

  // update our timer
  double time = timer.Get().value();
  double elapsedTime = time - lastTime;
  lastTime = time;

  // Update our odometry
  double microsecondTime = (double)RobotController::GetFPGATime();
  swerveDrive->UpdateOdometry(units::microsecond_t{microsecondTime});
  swerveDrive->UpdateConeOdometry();
  swerveDrive->UpdateTagOdometry();

  /*for (auto array : coneEntry.ReadQueue())
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
  }*/

  auto cubeTagArrays = cubeTagSub.ReadQueue();
  canSeeCubeTag = cubeTagArrays.size() >= 1;
  for (auto array : cubeTagArrays)
  {
    Translation2d poseEst = Translation2d(units::meter_t{array.value[0]}, units::meter_t{array.value[1]});
    frc::SmartDashboard::PutNumber("Tag Vision X", poseEst.X().value());
    frc::SmartDashboard::PutNumber("Tag Vision Y", poseEst.Y().value());
    swerveDrive->ResetTagOdometry(Pose2d(poseEst, swerveDrive->GetPose().Rotation()));
  }
  for (auto array : substationTagSub.ReadQueue())
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
    MAX_DRIVE_ACCELERATION = 4;
    MAX_SPIN_ACCELERATION = 4;
  }
  else if (xbox_Drive->GetLeftBumper() || xbox_Drive->GetXButton())
  {
    MAX_DRIVE_SPEED = 0.2;
    MAX_SPIN_SPEED = 0.2;
    MAX_DRIVE_ACCELERATION = 1;
    MAX_SPIN_ACCELERATION = 1;
  }
  else
  {
    MAX_DRIVE_SPEED = 0.4;
    MAX_SPIN_SPEED = 0.4;
    MAX_DRIVE_ACCELERATION = 2;
    MAX_SPIN_ACCELERATION = 2;
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

  if (xbox_Drive->GetXButton())
    lastFwdSpeed = 0;


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
        elevSpeed = -0.3;
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
      else if (xbox_Drive2->GetRightY() < -0.3)
        wristSpeed = 0.2;
      else if (xbox_Drive2->GetRightY() > 0.3)
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
        claw->MoveClawPercent(0.7);  
      else if (xbox_Drive2->GetRightTriggerAxis() > 0.5)
        claw->MoveClawPercent(-0.8);
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
        limelight->TurnOnLimelight();
        conePlaceXLimelightGoal = 0.19; 
        conePlaceYLimelightGoal = -0.04;
        conePlaceElevatorGoal = 47;  
        placingHigh = false;
        doneWithPoleAlignment = false;
        turnt = false;
        swerveDrive->BeginPIDLoop();
        currentDriverSection = SCORINGCONE;
      } 
      else if (xbox_Drive2->GetYButtonPressed())
      {
        //High Post
        limelight->TurnOnLimelight();
        conePlaceXLimelightGoal = 0.19; 
        conePlaceYLimelightGoal = -0.04;
        conePlaceElevatorGoal = 82;  
        placingHigh = true;
        doneWithPoleAlignment = false;
        turnt = false;
        swerveDrive->BeginPIDLoop();
        currentDriverSection = SCORINGCONE;
      }
      else if (xbox_Drive2->GetXButtonPressed())
      {
        
        //High Cube
        seeCubesEntry.Set(true);
        conePlaceYLimelightGoal= 0.91;
        conePlaceXLimelightGoal = 0.075;
        conePlaceElevatorGoal = 82;  
        placingHigh = true;
        doneWithPoleAlignment = false;
        turnt = false;
        swerveDrive->BeginPIDLoop();
        currentDriverSection = SCORINGCUBE;
      }
      else if (xbox_Drive2->GetAButtonPressed())
      {
        //Low Cube
        seeCubesEntry.Set(true);
        conePlaceYLimelightGoal= 0.91;
        conePlaceXLimelightGoal = 0.075;
        conePlaceElevatorGoal = 47;  
        placingHigh = false;
        doneWithPoleAlignment = false;
        turnt = false;
        swerveDrive->BeginPIDLoop();
        currentDriverSection = SCORINGCUBE;
      }
      else if (xbox_Drive2->GetBackButtonPressed())
      {
        currentDriverSection = ZEROING;
      }
      else if (xbox_Drive2->GetPOV() == 90 || xbox_Drive2->GetPOV() == 45 || xbox_Drive2->GetPOV() == 135)
      {
        seeSubstationTagsEntry.Set(true);
        turnt = false;
        centeredOnSubstation = false;
        coneInClaw = false;
        clawFinishedOpening = false;
        leftSubstation = false;
        currentDriverSection = SUBSTATIONINTAKING;
      }
      else if (xbox_Drive2->GetPOV() == 315 || xbox_Drive2->GetPOV() == 225 || xbox_Drive2->GetPOV() == 270)
      {
        seeSubstationTagsEntry.Set(true);
        turnt = false;
        centeredOnSubstation = false;
        coneInClaw = false;
        clawFinishedOpening = false;
        leftSubstation = true;
        currentDriverSection = SUBSTATIONINTAKING;
      }
      else if (xbox_Drive->GetRightTriggerAxis() > 0.5)
      {
        claw->PIDWrist(0.3, elapsedTime);
      }
      else if (xbox_Drive->GetLeftTriggerAxis() > 0.5)
      {
        coneInClaw = false;
        currentDriverSection = GROUNDPREPAREDTOGRAB;
      }
      else if (xbox_Drive->GetLeftTriggerAxis() > 0.5)
      {
        //currentDriverSection = UNTIPCONE;
      }
      else if (xbox_Drive->GetYButtonPressed())
      {
        currentDriverSection = BALANCE;
      }
      break;
    }

    case SCORINGCONE:
    {
      if (xbox_Drive2->GetLeftTriggerAxis() > 0.5)
        claw->MoveClawPercent(0.5);  
      else if (xbox_Drive2->GetRightTriggerAxis() > 0.5)
        claw->MoveClawPercent(-0.8);
      else
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
        if (limelight->getTargetArea() != 0 && limelight->getTargetArea() < 100000000) // tune
          centered = swerveDrive->StrafeToPole(offsetX, offsetY, conePlaceXLimelightGoal, conePlaceYLimelightGoal, elapsedTime);  
      }
      if (centered && lifted)
        doneWithPoleAlignment = true;
      if (doneWithPoleAlignment)
      {
        claw->PIDWrist(M_PI / 2, elapsedTime);
      }

      if ((!xbox_Drive2->GetYButton() && placingHigh) || (!xbox_Drive2->GetBButton() && !placingHigh))
      {
        limelight->TurnOffLimelight();
        currentDriverSection = BEGINDRIVING;
      }
      break;
    }

    case SCORINGCUBE:
    {
      if (xbox_Drive2->GetLeftTriggerAxis() > 0.5)
        claw->MoveClawPercent(0.5);  
      else if (xbox_Drive2->GetRightTriggerAxis() > 0.5)
        claw->MoveClawPercent(-0.8);
      else
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
        bool wristDown = claw->PIDWrist(M_PI / 2, elapsedTime);
        if (wristDown && canSeeCubeTag)
          claw->OpenClaw(elapsedTime);
      }

      if ((!xbox_Drive2->GetXButton() && placingHigh) || (!xbox_Drive2->GetAButton() && !placingHigh))
      {
        seeCubeTagsEntry.Set(false);
        currentDriverSection = BEGINDRIVING;
      }
      break;
    }

    case ZEROING:
    {
      swerveDrive->DriveSwervePercent(lastStrafeSpeed, lastFwdSpeed, lastTurnSpeed);
      if (xbox_Drive2->GetLeftTriggerAxis() > 0.5)
        claw->MoveClawPercent(0.5);  
      else if (xbox_Drive2->GetRightTriggerAxis() > 0.5)
        claw->MoveClawPercent(-0.8);
      else
        claw->MoveClawPercent(0);

      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      if (elevatorLift->winchEncoderReading() < 5)
        claw->PIDWrist(0.2, elapsedTime);
      else
        claw->PIDWrist(1, elapsedTime);

      if (!xbox_Drive2->GetBackButton())
         currentDriverSection = RESUMEDRIVING;
      break;
    }

    case GROUNDPREPAREDTOGRAB:
    {
      swerveDrive->DriveSwervePercent(lastStrafeSpeed, lastFwdSpeed, lastTurnSpeed);
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      claw->PIDWrist(2.15, elapsedTime);
      
      if ((claw->ConeInClaw() && claw->ClawEncoderReading() > 5) || coneInClaw)
      {
        claw->CloseClaw(elapsedTime);
        coneInClaw = true;
      }
      else 
      {
        claw->OpenClaw(elapsedTime);
      }

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
        claw->PIDWrist(2.15, elapsedTime);

      if (xbox_Drive->GetRightTriggerAxis() < 0.2)
        currentDriverSection = RESUMEDRIVING; 
      break;
    }

    case SUBSTATIONINTAKING:
    {
      if (leftSubstation)
      {
        bool lifted = elevatorLift->SetElevatorHeightPID(66, elapsedTime);
        claw->PIDWrist(M_PI / 2, elapsedTime);
        if (!turnt)
          turnt = swerveDrive->DriveToPose(Pose2d(swerveDrive->GetPose().Translation(), Rotation2d(0_deg)), elapsedTime);
        if (turnt && !centeredOnSubstation)
        {
          centeredOnSubstation = swerveDrive->DriveToPoseTag(Pose2d(-0.65_m, -1.5_m, Rotation2d(0_deg)), elapsedTime); 
        }
        if (centeredOnSubstation && lifted && !coneInClaw)
        {
          coneInClaw = swerveDrive->DriveToPoseTag(Pose2d(-0.65_m, -0.98_m, Rotation2d(0_deg)), elapsedTime); 
        }
        if (centeredOnSubstation && lifted && (claw->ConeInClaw() || coneInClaw))
        {
          swerveDrive->DriveSwervePercent(0,0,0);
          coneInClaw = true;
        }
        if (coneInClaw)
          claw->CloseClaw(elapsedTime);
        else
          claw->OpenClaw(elapsedTime);
        SmartDashboard::PutBoolean("centered", centeredOnSubstation);
        SmartDashboard::PutBoolean("lifted", lifted);
        SmartDashboard::PutBoolean("done", coneInClaw);
        SmartDashboard::PutBoolean("cone in claw", claw->ConeInClaw());
      }
      else
      {
        swerveDrive->DriveSwervePercent(lastStrafeSpeed, lastFwdSpeed, lastTurnSpeed);
        elevatorLift->SetElevatorHeightPID(72, elapsedTime);
        claw->PIDWrist(M_PI / 2, elapsedTime);
        if (!clawFinishedOpening)
          clawFinishedOpening = claw->OpenClaw(elapsedTime);
        else 
        {
           if (xbox_Drive2->GetLeftTriggerAxis() > 0.5)
            claw->MoveClawPercent(0.5);  
          else if (xbox_Drive2->GetRightTriggerAxis() > 0.5)
            claw->MoveClawPercent(-0.8);
          else
            claw->MoveClawPercent(0);     
        }
      }

      if (xbox_Drive2->GetPOV() == -1)
      {
        seeSubstationTagsEntry.Set(false);
        if (leftSubstation)
          currentDriverSection = BEGINDRIVING;
        else
          currentDriverSection = RESUMEDRIVING; 
      }
      break;
    }

    case UNTIPCONE:
    {
      swerveDrive->DriveSwervePercent(lastStrafeSpeed, lastFwdSpeed, lastTurnSpeed);
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      claw->PIDWrist(1.866, elapsedTime);
      claw->OpenClaw(elapsedTime);
      if (xbox_Drive->GetLeftTriggerAxis() < 0.2)
        currentDriverSection = RESUMEDRIVING;  
      break;
    }

    case BALANCE:
    {
      swerveDrive->BalanceOnCharger(elapsedTime);
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      claw->PIDWrist(0.6, elapsedTime);
      claw->OpenClaw(elapsedTime);

      if(!xbox_Drive->GetYButton())
        currentDriverSection = BEGINDRIVING;
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
