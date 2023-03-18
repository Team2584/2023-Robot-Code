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
// Suggestion: It might be safer to have these as shared pointers instead of raw pointers.
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
double lastConeX;
double lastConeY;

// To track time for slew rate and pid controll
frc::Timer timer;
frc::Timer masterTimer;
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
  BALANCE = 8,
  AUTOGRABBINGCONE = 9,
  ANGLEDSUBSTATION = 10,
  AUTOCLOSE = 11,
  LIFTINGTOPLACEOBJECT = 12
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
bool turnedOffCone = false;
double preBalanceXGoal = 0;
double preBalanceYGoal = 0;

#define HIGHCONELIFTHEIGHT 80.0
#define HIGHCONEX 0.19
#define HIGHCONEY -0.04

double startingSanity = 0;

void Robot::RobotInit()
{
  // Set all Values from Shuffleboard (Smartdashboard but cooler)
  frc::SmartDashboard::PutNumber("MAX DRIVE SPEED", 0.4);
  frc::SmartDashboard::PutNumber("ELEVATOR_SPEED", 0.1);

  // Autonomous Choosing
  m_chooser.SetDefaultOption(kAuto1GO, kAuto1GO);
  m_chooser.AddOption(kAutoRL2GO, kAutoRL2GO);
  m_chooser.AddOption(kAutoRL1GOB, kAutoRL1GOB);
  m_chooser.AddOption(kAutoRLCubeCone, kAutoRLCubeCone);
  m_chooser.AddOption(kAutoRConeB, kAutoRConeB);
  m_chooser.AddOption(kAutoRCubeB, kAutoRCubeB);
  m_chooser.AddOption(kAutoRConeDriveL, kAutoRConeDriveL);
  m_chooser.AddOption(kAutoRConeDriveR, kAutoRConeDriveR);
  m_chooser.AddOption(kAutoRR2GO, kAutoRR2GO);
  m_chooser.AddOption(kAutoRR1GOB, kAutoRR1GOB);
  m_chooser.AddOption(kAutoRRCubeCone, kAutoRRCubeCone);
  m_chooser.AddOption(kAutoBL2GO, kAutoBL2GO);
  m_chooser.AddOption(kAutoBL1GOB, kAutoBL1GOB);
  m_chooser.AddOption(kAutoBLCubeCone, kAutoBLCubeCone);
  m_chooser.AddOption(kAutoBConeB, kAutoBConeB);
  m_chooser.AddOption(kAutoBCubeB, kAutoBCubeB);
  m_chooser.AddOption(kAutoBConeDriveL, kAutoBConeDriveL);
  m_chooser.AddOption(kAutoBConeDriveR, kAutoBConeDriveR);
  m_chooser.AddOption(kAutoBR2GO, kAutoBR2GO);
  m_chooser.AddOption(kAutoBR1GOB, kAutoBR1GOB);
  m_chooser.AddOption(kAutoBRCubeCone, kAutoBRCubeCone);
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
  masterTimer = Timer();

  // Initializing Subsystems
  swerveDrive = new SwerveDrive(&driveFL, &swerveFL, &FLMagEnc, FL_WHEEL_OFFSET, &driveFR, &swerveFR, &FRMagEnc,
                                FR_WHEEL_OFFSET, &driveBR, &swerveBR, &BRMagEnc, BR_WHEEL_OFFSET, &driveBL,
                                &swerveBL, &BLMagEnc, BL_WHEEL_OFFSET, &_pigeon, STARTING_DRIVE_HEADING);
  elevatorLift = new ElevatorLift(&winchL, &winchR, &TOFSensor);
  limelight = new Limelight(limelightTable);
  claw = new Claw(&wrist);
  lights  = new LEDLights(&lightController);

  lights->SetLED();
  //limelight->TurnOffLimelight();
  limelight->TurnOnLimelight();

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
  // limelight->TurnOnLimelight();
  // limelight->getTargetX();
  // limelight->getTargetY();
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

  //Suggestion: It would be cleaner and more efficient to put these if statements in one big switch
  //Suggestion: Changing m_autoSelected to an enum would save space and remove extra calculations
     
  /* Advanced Suggestion:
  To make adding or modifying autonomous programs quicker, you could create an "AutonomousFunctionClass".
  AutonomousFunctionClass could have a constructor like: AutonomousFunctionClass(bool AllianceColor, Pose2d startingOdomoetry, List<string> trajectoryNames).
  AutonomousFunctionClass could then have an Initialize function that takes in a SwerveDrive* and calls SetAllianceColor, ResetOdometry, InitializeTrajectory, etc... with the values from its constructor.
    
  // Advanced Suggestion Part 2:
  Instead of your m_chooser being a SendableChooser<string> you can make it a SendableChooser<AutonomousFunctionClass>.
  To add options to the m_chooser, you could do something like m_Chooser.AddOption(autonName, AutonomousFunctionClass(*correct parameters for the auton*));
  Finally, m_chooser.GetSelected would return the proper AutonomousFunctionClass, and all you need to do is call that class's initialize function -- zero if statements!!
  
  // Fixing issues this suggestion might cause:
  I understand that some other parts of Robot.cpp rely on m_autoSelected, which this suggestion removes.
  There are ways to abstract all the code that uses m_autoSelected into the AutonomousFunctionClass, which we can work toward in the future.
  For now, I recommend first adding an extra string parameter to the AutonomousFunctionClass constructor call AutonName.
  You could then replace m_autonSelected with selectedAutonomousFunctionClass.AutonName in the other parts of Robot.cpp!
  */
  if (m_autoSelected == kAutoRR2GO)
  {
    swerveDrive->SetAllianceColorRed();
    swerveDrive->ResetOdometry(Pose2d(4_m, 1.85_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("RedRight2GamePiece1");
    swerveDrive->InitializeTrajectory("RedRight2GamePiece2");
    swerveDrive->SetNextTrajectory();  
  }
  else if (m_autoSelected == kAutoRRCubeCone)
  {
    claw->ResetClawEncoder(6.6);
    swerveDrive->SetAllianceColorRed();
    swerveDrive->ResetOdometry(Pose2d(4.5_m, 1.85_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("RedRightCube2GamePiece1", 3_mps, 3_mps_sq);
    swerveDrive->InitializeTrajectory("RedRight2GamePiece2", 3_mps, 3_mps_sq);
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
  else if (m_autoSelected == kAutoRLCubeCone)
  {
    claw->ResetClawEncoder(6.6);
    swerveDrive->SetAllianceColorRed();
    swerveDrive->ResetOdometry(Pose2d(1.18_m, 1.79_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("RedLeftCube2GamePiece1", 3_mps, 3_mps_sq);
    swerveDrive->InitializeTrajectory("RedLeft2GamePiece2", 3_mps, 3_mps_sq);
    swerveDrive->SetNextTrajectory();  
  }
  else if (m_autoSelected == kAutoBR2GO)
  {
    swerveDrive->SetAllianceColorBlue();
    swerveDrive->ResetOdometry(Pose2d(6.54_m, 1.85_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("BlueRight2GamePiece1");
    swerveDrive->InitializeTrajectory("BlueRight2GamePiece2");
    swerveDrive->SetNextTrajectory();      
  }
  else if (m_autoSelected == kAutoBRCubeCone)
  {
    claw->ResetClawEncoder(6.6);
    swerveDrive->SetAllianceColorBlue();
    swerveDrive->ResetOdometry(Pose2d(7.0_m, 1.85_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("BlueRightCube2GamePiece1", 3_mps, 3_mps_sq);
    swerveDrive->InitializeTrajectory("BlueRight2GamePiece2", 3_mps, 3_mps_sq);
    swerveDrive->SetNextTrajectory();      
  }
  else if (m_autoSelected == kAutoBL2GO)
  {
    swerveDrive->SetAllianceColorBlue();
    swerveDrive->ResetOdometry(Pose2d(3.2_m, 1.85_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("BlueLeft2GamePiece1");
    swerveDrive->InitializeTrajectory("BlueLeft2GamePiece2");
    swerveDrive->SetNextTrajectory();      
  }
  else if (m_autoSelected == kAutoBLCubeCone)
  {
    claw->ResetClawEncoder(6.6);
    swerveDrive->SetAllianceColorBlue();
    swerveDrive->ResetOdometry(Pose2d(3.69_m, 1.85_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("BlueLeftCube2GamePiece1", 3_mps, 3_mps_sq);
    swerveDrive->InitializeTrajectory("BlueLeft2GamePiece2", 3_mps, 3_mps_sq);
    swerveDrive->SetNextTrajectory();      
  }
  else if (m_autoSelected == kAutoRR1GOB)
  {
    swerveDrive->SetAllianceColorRed();
    swerveDrive->ResetOdometry(Pose2d(4_m, 1.85_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("RedRight2GamePiece1");
    swerveDrive->InitializeTrajectory("RedRight1GamePieceBalance2", 2.7_mps, 7_mps_sq);
    swerveDrive->SetNextTrajectory();  
  }
  else if (m_autoSelected == kAutoBL1GOB)
  {
    swerveDrive->SetAllianceColorBlue();
    swerveDrive->ResetOdometry(Pose2d(3.25_m, 1.84_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("BlueLeft2GamePiece1");
    swerveDrive->InitializeTrajectory("BlueLeft1GamePieceBalance2", 2.7_mps, 7_mps_sq);
    swerveDrive->SetNextTrajectory();      
  }
  else if (m_autoSelected == kAutoRL1GOB)
  {
    swerveDrive->SetAllianceColorRed();
    swerveDrive->ResetOdometry(Pose2d(4_m, 1.85_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("RedLeft2GamePiece1");
    swerveDrive->InitializeTrajectory("RedLeft1GamePieceBalance2", 2.7_mps, 7_mps_sq);
    swerveDrive->SetNextTrajectory();  
  }
  else if (m_autoSelected == kAutoBR1GOB)
  {
    swerveDrive->SetAllianceColorBlue();
    swerveDrive->ResetOdometry(Pose2d(3.25_m, 1.84_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("BlueRight2GamePiece1");
    swerveDrive->InitializeTrajectory("BlueRight1GamePieceBalance2", 2.7_mps, 7_mps_sq);
    swerveDrive->SetNextTrajectory();      
  }
  else if (m_autoSelected == kAutoRConeB)
  {
    swerveDrive->SetAllianceColorRed();  
    swerveDrive->ResetOdometry(Pose2d(2.35_m, 1.85_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("RedCenterConeBalance1");
    swerveDrive->InitializeTrajectory("RedCenterBalance2", 2.7_mps, 7_mps_sq);
    swerveDrive->SetNextTrajectory();      
  }
  else if (m_autoSelected == kAutoBConeB)
  {
    swerveDrive->SetAllianceColorBlue();  
    swerveDrive->ResetOdometry(Pose2d(4.86_m, 1.91_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("BlueCenterConeBalance1");
    swerveDrive->InitializeTrajectory("BlueCenterBalance2", 2.7_mps, 7_mps_sq);
    swerveDrive->SetNextTrajectory();      
  }
  else if (m_autoSelected == kAutoRCubeB)
  {
    claw->ResetClawEncoder(6.6);
    swerveDrive->SetAllianceColorRed();  
    swerveDrive->ResetOdometry(Pose2d(2.89_m, 1.85_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("RedCenterCubeBalance1");
    swerveDrive->InitializeTrajectory("RedCenterBalance2", 2.7_mps, 7_mps_sq);
    swerveDrive->SetNextTrajectory();      
  }
  else if (m_autoSelected == kAutoBCubeB)
  {
    claw->ResetClawEncoder(6.6);
    swerveDrive->SetAllianceColorBlue();  
    swerveDrive->ResetOdometry(Pose2d(5.19_m, 1.85_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("BlueCenterCubeBalance1");
    swerveDrive->InitializeTrajectory("BlueCenterBalance2", 2.7_mps, 7_mps_sq);
    swerveDrive->SetNextTrajectory();      
  }
  else if (m_autoSelected == kAutoRConeDriveL)
  {
    swerveDrive->SetAllianceColorRed();  
    swerveDrive->ResetOdometry(Pose2d(2.35_m, 1.85_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("RedCenterConeDriveOutLeft");
    swerveDrive->SetNextTrajectory();      
  }
  else if (m_autoSelected == kAutoRConeDriveR)
  {
    swerveDrive->SetAllianceColorRed();  
    swerveDrive->ResetOdometry(Pose2d(2.35_m, 1.85_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("RedCenterConeDriveOutRight");
    swerveDrive->SetNextTrajectory();     
  }
  else if (m_autoSelected == kAutoBConeDriveL)
  {
    swerveDrive->SetAllianceColorBlue();  
    swerveDrive->ResetOdometry(Pose2d(4.86_m, 1.91_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("BlueCenterConeDriveOutLeft");
    swerveDrive->SetNextTrajectory();      
  }
  else if (m_autoSelected == kAutoBConeDriveR)
  {
    swerveDrive->SetAllianceColorBlue();  
    swerveDrive->ResetOdometry(Pose2d(4.86_m, 1.91_m, Rotation2d(180_deg)));
    swerveDrive->InitializeTrajectory("BlueCenterConeDriveOutRight");
    swerveDrive->SetNextTrajectory();      
  }
  else if (m_autoSelected == kAuto1GO)
    swerveDrive->ResetOdometry(Pose2d(5.19_m, 1.85_m, Rotation2d(180_deg)));



  // else if (m_autoSelected == kAutoRR2GONV)
  // {
  //   swerveDrive->SetAllianceColorRed();
  //   swerveDrive->ResetOdometry(Pose2d(4_m, 1.85_m, Rotation2d(180_deg)));
  //   swerveDrive->InitializeTrajectory("RedRight2GamePiece");
  //   swerveDrive->SetNextTrajectory();  
  // }
  // else if (m_autoSelected == kAutoRL2GONV)
  // {
  //   swerveDrive->SetAllianceColorRed();
  //   swerveDrive->ResetOdometry(Pose2d(0.63_m, 1.85_m, Rotation2d(180_deg)));
  //   swerveDrive->InitializeTrajectory("RedLeft2GamePiece");
  //   swerveDrive->SetNextTrajectory();  
  // }
  // else if (m_autoSelected == kAutoBR2GONV)
  // {
  //   swerveDrive->SetAllianceColorBlue();
  //   swerveDrive->ResetOdometry(Pose2d(6.57_m, 1.89_m, Rotation2d(180_deg)));
  //   swerveDrive->InitializeTrajectory("BlueRight2GamePiece");
  //   swerveDrive->SetNextTrajectory();      
  // }
  // else if (m_autoSelected == kAutoBL2GONV)
  // {
  //   swerveDrive->SetAllianceColorBlue();
  //   swerveDrive->ResetOdometry(Pose2d(3.3_m, 1.91_m, Rotation2d(180_deg)));
  //   swerveDrive->InitializeTrajectory("BlueLeft2GamePiece");
  //   swerveDrive->SetNextTrajectory();      
  // }


  // Start our match timer and reset our odometry to the robot's starting position
  lastTime = 0;
  timer.Reset();
  timer.Start();
  masterTimer.Reset();
  masterTimer.Start();

  seeConesEntry.Set(true);
  seeCubesEntry.Set(false);
  seeCubeTagsEntry.Set(false);
  seeSubstationTagsEntry.Set(false);
}

void Robot::AutonomousPeriodic()
{
  SmartDashboard::PutNumber("Spline Section", splineSection);
  SmartDashboard::PutNumber("claw encoder", claw->ClawEncoderReading());
  SmartDashboard::PutNumber("master auto timer", masterTimer.Get().value());
  lights->SetLED();







  

  if (m_autoSelected == kAutoRR2GO || m_autoSelected == kAutoRL2GO || m_autoSelected == kAutoBL2GO || m_autoSelected == kAutoBR2GO)
  {
    double elapsedTime = timer.Get().value() - lastTime;
    swerveDrive->UpdateOdometry(timer.Get());
    swerveDrive->UpdateConeOdometry();
    for (auto array : coneEntry.ReadQueue())
    {
      //Suggestion: Whenever array.value[1] > 0.75 the condition is true, and whenever array.value[1] <= .75 the condition is false.
      //Because it does not affect the results of the if statement, I recommend either changing or removing (array.value[0] != 0 || array.value[1] != 0).
      if ((array.value[0] != 0 || array.value[1] != 0) && array.value[1] > 0.75)
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
      timer.Reset();
      turnedOffCone = false;
      lastTime = 0;
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
      bool lifted = false;
      if (claw->MagEncoderReading() > 0.25)
        lifted = elevatorLift->SetElevatorHeightPID(HIGHCONELIFTHEIGHT, elapsedTime);
      else 
        elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      bool centered = false;
      if (!turnt)
        turnt = swerveDrive->DriveToPose(Pose2d(swerveDrive->GetPose().Translation(), Rotation2d(180_deg)), elapsedTime);
      if (turnt && elevatorLift->winchEncoderReading() > 30)
      {
        double offsetX = limelight->getTargetX();
        double offsetY = limelight->getTargetY();
        centered = swerveDrive->StrafeToPole(offsetX, offsetY, HIGHCONEX, HIGHCONEY, elapsedTime);  
      }
      SmartDashboard::PutBoolean("cenetered", centered);
      SmartDashboard::PutBoolean("lifted", lifted);
      if ((centered && lifted) || timer.Get() > 4_s)
      {
        doneWithPoleAlignment = true;
      }
      if (doneWithPoleAlignment)
      {
        claw->PIDWrist(M_PI / 2, elapsedTime);
        if (claw->MagEncoderReading() > M_PI / 2 - 0.05 || timer.Get() > 5_s)
          claw->OpenClaw(elapsedTime);
        if (claw->ClawEncoderReading() > 5)
        {
          splineSection = 0.9;
          if (timer.Get() < 5_s)
          {
            if (m_autoSelected == kAutoBR2GO)
              swerveDrive->ResetOdometry(Pose2d(6.46_m, 1.91_m, swerveDrive->GetPose().Rotation()));
            else if (m_autoSelected == kAutoBL2GO)
              swerveDrive->ResetOdometry(Pose2d(3.2_m, 1.91_m, swerveDrive->GetPose().Rotation()));
            else if (m_autoSelected == kAutoRR2GO)
              swerveDrive->ResetOdometry(Pose2d(4_m, 1.91_m, swerveDrive->GetPose().Rotation()));
            else if (m_autoSelected == kAutoRL2GO)
              swerveDrive->ResetOdometry(Pose2d(0.63_m, 1.91_m, swerveDrive->GetPose().Rotation()));
          }
        }
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
      if (swerveDrive->GetPose().Y() > 4.75_m)
        claw->PIDWristDown(elapsedTime);
      else
        claw->PIDWristUp(elapsedTime);
      bool splineDone = swerveDrive->FollowTrajectory(timer.Get(), elapsedTime);
      if (splineDone)
      {
        splineSection = 1.5;
        swerveDrive->SetNextTrajectory();
        swerveDrive->DriveSwervePercent(0,0,0);
        swerveDrive->ResetConeOdometry(Pose2d(0_m, -1_m, Rotation2d(0_deg)));
        turnt = false;
        coneInClaw = false;
        claw->BeginClawPID();
      }
    }

    if (splineSection == 1.5)
    {
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      claw->PIDWristDown(elapsedTime);

      if (swerveDrive->GetConeOdometryPose().Y().value() > 3 && turnt && !turnedOffCone) 
      {
        swerveDrive->ResetConeOdometry(Pose2d(units::meter_t{lastConeX}, units::meter_t{lastConeY}, Rotation2d(0_deg)));
        seeConesEntry.Set(false);
        turnedOffCone = true;
        SmartDashboard::PutBoolean("caught bad cone", true);
      }

      turnt = true; // maybe remove this later

      if (!turnt)
      {
        double angleGoal = atan2(-currentConeX, -currentConeY);
        turnt = swerveDrive->TurnToPixelCone(angleGoal, elapsedTime);
        goalConeGrabAngle = swerveDrive->GetPose().Rotation();
      }
      SmartDashboard::PutBoolean("turnt", turnt);
      if (turnt && !coneInClaw)
        coneInClaw = claw->ObjectInClaw();
      
      if (turnt && !coneInClaw)
        coneInClaw = swerveDrive->DriveToPoseConeOdometry(Pose2d(0_m, -0.58_m, goalConeGrabAngle), elapsedTime);
      else if (turnt)
        swerveDrive->DriveSwervePercent(0,0,0);

      if (!coneInClaw && swerveDrive->GetPose().Y() > 7_m)
      {
        swerveDrive->DriveSwervePercent(0,0,0);
        coneInClaw = true;
      }

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
        centered = swerveDrive->StrafeToPole(offsetX, offsetY, 0.19, -0.055, elapsedTime); 
      }
      if (centered && lifted)
        doneWithPoleAlignment = true;
      if (doneWithPoleAlignment || masterTimer.Get() > 14.5_s)
      {
        claw->PIDWrist(M_PI / 2, elapsedTime);
        if (claw->MagEncoderReading() > M_PI / 2 - 0.05)
          claw->OpenClaw(elapsedTime);
        if (claw->ClawEncoderReading() > 8)
        {
          splineSection = 2.75;
          swerveDrive->DriveSwervePercent(0,0,0);
          claw->MoveClawPercent(0);
          elevatorLift->MoveElevatorPercent(0);
          claw->MoveWristPercent(0);
        }
      }
    }

    if (splineSection == 2.75)
    {
      swerveDrive->DriveSwervePercent(0,0,0);
      claw->MoveClawPercent(0);
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      if (elevatorLift->winchEncoderReading() < 5)
        claw->PIDWristUp(elapsedTime);
      else
        claw->PIDWrist(1, elapsedTime);
    }
  }










  else if (m_autoSelected == kAutoBLCubeCone || m_autoSelected == kAutoBRCubeCone || m_autoSelected == kAutoRRCubeCone || m_autoSelected == kAutoRLCubeCone)
  {
    double elapsedTime = timer.Get().value() - lastTime;
    swerveDrive->UpdateOdometry(timer.Get());
    swerveDrive->UpdateConeOdometry();
    swerveDrive->UpdateTagOdometry();
    for (auto array : coneEntry.ReadQueue())
    {
      if ((array.value[0] != 0 || array.value[1] != 0) && array.value[1] > 0.75)
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
    for (auto array :cubeTagSub.ReadQueue())
    {
      Translation2d poseEst = Translation2d(units::meter_t{array.value[0]}, units::meter_t{array.value[1]});
      frc::SmartDashboard::PutNumber("Tag Vision X", poseEst.X().value());
      frc::SmartDashboard::PutNumber("Tag Vision Y", poseEst.Y().value());
      swerveDrive->ResetTagOdometry(Pose2d(poseEst, swerveDrive->GetPose().Rotation()));
    }
  
    if (splineSection == 0)
    {
      limelight->TurnOffLimelight();  
      seeCubeTagsEntry.Set(true);
      splineSection = 0.5; 
      conePlaceYLimelightGoal= 0.85;
      conePlaceXLimelightGoal = 0.075;
      conePlaceElevatorGoal = 70;  
      placingHigh = true;
      doneWithPoleAlignment = false;
      turnt = false;
      swerveDrive->ResetTagOdometry(Pose2d(0.075_m, 0.91_m, Rotation2d(180_deg)));
      swerveDrive->BeginPIDLoop();
      timer.Reset();
    }


    if (splineSection == 0.5)
    {   
      if (!doneWithPoleAlignment)
      {
        claw->PIDWrist(1.1, elapsedTime);
      }
      bool lifted = false;
      if (claw->MagEncoderReading() > 0.25)
        lifted = elevatorLift->SetElevatorHeightPID(conePlaceElevatorGoal, elapsedTime);
      else 
        elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      bool centered = false;
      SmartDashboard::PutBoolean("lifted", lifted);
      if (lifted)
      {
        centered = swerveDrive->DriveToPoseTag(Pose2d(units::meter_t{conePlaceXLimelightGoal}, units::meter_t{conePlaceYLimelightGoal}, Rotation2d(180_deg)), elapsedTime); 
      }
      SmartDashboard::PutBoolean("centered", centered);
      if ((centered && lifted) || timer.Get() > 4_s)
        doneWithPoleAlignment = true;
      
      if (doneWithPoleAlignment)
      {
        claw->PIDWrist(1.1, elapsedTime);
        claw->OpenClaw(elapsedTime);
      }

      if (claw->ClawEncoderReading() > 9)
      {
        splineSection = 0.9;
        if (m_autoSelected == kAutoBRCubeCone)
          swerveDrive->ResetOdometry(Pose2d(7.0_m, 1.85_m, swerveDrive->GetPose().Rotation()));
        else if (m_autoSelected == kAutoBLCubeCone)
          swerveDrive->ResetOdometry(Pose2d(3.69_m, 1.85_m, swerveDrive->GetPose().Rotation()));
        else if (m_autoSelected == kAutoRRCubeCone)
          swerveDrive->ResetOdometry(Pose2d(4.5_m, 1.85_m, swerveDrive->GetPose().Rotation()));
        else if (m_autoSelected == kAutoRLCubeCone)
          swerveDrive->ResetOdometry(Pose2d(1.18_m, 1.85_m, swerveDrive->GetPose().Rotation()));
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
      if (swerveDrive->GetPose().Y() > 4.75_m)
        claw->PIDWristDown(elapsedTime);
      else
        claw->PIDWristUp(elapsedTime);
      bool splineDone = swerveDrive->FollowTrajectory(timer.Get(), elapsedTime);
      if (splineDone)
      {
        splineSection = 1.5;
        swerveDrive->SetNextTrajectory();
        swerveDrive->DriveSwervePercent(0,0,0);
        swerveDrive->ResetConeOdometry(Pose2d(0_m, -1_m, Rotation2d(0_deg)));
        turnt = false;
        coneInClaw = false;
        claw->BeginClawPID();
      }
    }

    if (splineSection == 1.5)
    {
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      claw->PIDWristDown(elapsedTime);

      if (swerveDrive->GetConeOdometryPose().Y().value() > 3 && turnt && !turnedOffCone) 
      {
        swerveDrive->ResetConeOdometry(Pose2d(units::meter_t{lastConeX}, units::meter_t{lastConeY}, Rotation2d(0_deg)));
        seeConesEntry.Set(false);
        turnedOffCone = true;
        SmartDashboard::PutBoolean("caught bad cone", true);
      }

      turnt = true; // maybe remove this later

      if (!turnt)
      {
        double angleGoal = atan2(-currentConeX, -currentConeY);
        turnt = swerveDrive->TurnToPixelCone(angleGoal, elapsedTime);
        goalConeGrabAngle = swerveDrive->GetPose().Rotation();
      }
      SmartDashboard::PutBoolean("turnt", turnt);
      if (turnt && !coneInClaw)
        coneInClaw = claw->ObjectInClaw();
      
      if (turnt && !coneInClaw)
        coneInClaw = swerveDrive->DriveToPoseConeOdometry(Pose2d(0_m, -0.58_m, goalConeGrabAngle), elapsedTime);
      else if (turnt)
        swerveDrive->DriveSwervePercent(0,0,0);

      if (!coneInClaw && swerveDrive->GetPose().Y() > 7_m)
      {
        swerveDrive->DriveSwervePercent(0,0,0);
        coneInClaw = true;
      }

      SmartDashboard::PutBoolean("atCone", coneInClaw);

      bool clawClosed = false;
      if (coneInClaw) 
      {
        clawClosed = claw->CloseClaw(elapsedTime);
        swerveDrive->DriveSwervePercent(0,0,0);
      }
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
        limelight->TurnOnLimelight();
        swerveDrive->BeginPIDLoop();
      }
    }

    if (splineSection == 2.5)
    {
      if (!doneWithPoleAlignment)
      {
        claw->PIDWrist(0.9, elapsedTime);
      }
      bool lifted = elevatorLift->SetElevatorHeightPID(HIGHCONELIFTHEIGHT, elapsedTime); 
      bool centered = false;
      if (!turnt)
        turnt = swerveDrive->DriveToPose(Pose2d(swerveDrive->GetPose().Translation(), Rotation2d(180_deg)), elapsedTime);
      if (turnt && elevatorLift->winchEncoderReading() > 40)
      {
        double offsetX = limelight->getTargetX();
        double offsetY = limelight->getTargetY();
        centered = swerveDrive->StrafeToPole2(offsetX, offsetY, HIGHCONEX, -0.07, elapsedTime); 
      }
      if ((centered && lifted))
        doneWithPoleAlignment = true;
      if (doneWithPoleAlignment)
      {
        claw->PIDWrist(M_PI / 2, elapsedTime);
        if (claw->MagEncoderReading() > M_PI / 2 - 0.05)
          claw->OpenClaw(elapsedTime);
        if (claw->ClawEncoderReading() > 8)
        {
          splineSection = 2.75;
          swerveDrive->DriveSwervePercent(0,0,0);
          claw->MoveClawPercent(0);
          elevatorLift->MoveElevatorPercent(0);
          claw->MoveWristPercent(0);
        }
      }
    }

    if (splineSection == 2.75)
    {
      swerveDrive->DriveSwervePercent(0,0,0);
      claw->MoveClawPercent(0);
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      if (elevatorLift->winchEncoderReading() < 5)
        claw->PIDWristUp(elapsedTime);
      else
        claw->PIDWrist(1, elapsedTime);
    }
  }



/*
  else if (m_autoSelected == kAutoBL2GONV || m_autoSelected == kAutoBR2GONV || m_autoSelected == kAutoRR2GONV || m_autoSelected == kAutoRL2GONV)
  {
    double elapsedTime = timer.Get().value() - lastTime;
    swerveDrive->UpdateOdometry(timer.Get());
    swerveDrive->UpdateConeOdometry();
    for (auto array : coneEntry.ReadQueue())
    {
      if ((array.value[0] != 0 || array.value[1] != 0) && array.value[1] > 0.75)
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
      timer.Reset();
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
      {
        doneWithPoleAlignment = true;
        timer.Reset();
        lastTime = 0;
      }
      if (doneWithPoleAlignment || timer.Get() > 2_s)
      {
        claw->PIDWrist(M_PI / 2, elapsedTime);
        if (claw->MagEncoderReading() > M_PI / 2 - 0.05 || timer.Get() > 1_s)
          claw->OpenClaw(elapsedTime);
        if (claw->ClawEncoderReading() > 5)
          splineSection = 0.9;
      }
    }

    if (splineSection == 0.9)
    {
      claw->PIDWrist(0.6, elapsedTime);
      claw->OpenClaw(elapsedTime);
      if (claw->MagEncoderReading() < 0.75)
        elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      if (elevatorLift->winchEncoderReading() < 40)
      {
        swerveDrive->ResetOdometry(Pose2d(3.3_m, 1.91_m, swerveDrive->GetPose().Rotation()));
        timer.Reset();
        lastTime = 0;
        splineSection = 1;
        claw->BeginClawPID();
        coneInClaw = false;
      }
    }

    if (splineSection == 1)
    {
      bool splineDone = swerveDrive->FollowTrajectory(timer.Get(), elapsedTime);
      bool clawClosed = false;
      if (!coneInClaw && swerveDrive->GetPose().Y().value() > 5.92)
        coneInClaw = claw->ConeInClaw();      

      if (coneInClaw || swerveDrive->GetPose().Y().value() > 7.55)
        clawClosed = claw->CloseClaw(elapsedTime);
      else
        claw->OpenClaw(elapsedTime);
      
      if (clawClosed || timer.Get() > 4_s)
        claw->PIDWrist(0.6, elapsedTime);
      else
        claw->PIDWristDown(elapsedTime);

      if (swerveDrive->GetPose().Y().value() < 3.5 && coneInClaw)
        elevatorLift->SetElevatorHeightPID(47, elapsedTime);
      else
        elevatorLift->SetElevatorHeightPID(0, elapsedTime);

      if (splineDone)
      {
        splineSection = 1.5;
        swerveDrive->DriveSwervePercent(0,0,0);
        doneWithPoleAlignment = false;
        turnt = false;
        swerveDrive->BeginPIDLoop();
      }
    }

    if (splineSection == 1.5)
    {
      if (!doneWithPoleAlignment)
      {
        claw->PIDWrist(0.9, elapsedTime);
      }
      bool lifted = elevatorLift->SetElevatorHeightPID(52, elapsedTime); 
      bool centered = false;
      if (!turnt)
        turnt = swerveDrive->DriveToPose(Pose2d(swerveDrive->GetPose().Translation(), Rotation2d(180_deg)), elapsedTime);
      if (turnt && elevatorLift->winchEncoderReading() > 30)
      {
        double offsetX = limelight->getTargetX();
        double offsetY = limelight->getTargetY();
        centered = swerveDrive->StrafeToPole(offsetX, offsetY, 0.19, -0.0555, elapsedTime); 
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
          splineSection = 1.9;
          swerveDrive->DriveSwervePercent(0,0,0);
          claw->MoveClawPercent(0);
          elevatorLift->MoveElevatorPercent(0);
          claw->MoveWristPercent(0);
        }
      }
    }

    if (splineSection == 1.9)
    {
      claw->PIDWrist(0.6, elapsedTime);
      claw->OpenClaw(elapsedTime);
      if (claw->MagEncoderReading() < 0.75)
        elevatorLift->SetElevatorHeightPID(0, elapsedTime);
    }
  }
*/








  else if (m_autoSelected == kAutoBL1GOB || m_autoSelected == kAutoRR1GOB || m_autoSelected == kAutoBR1GOB || m_autoSelected == kAutoRL1GOB)
  {
    double elapsedTime = timer.Get().value() - lastTime;
    swerveDrive->UpdateOdometry(timer.Get());
    swerveDrive->UpdateConeOdometry();
    for (auto array : coneEntry.ReadQueue())
    {
      if ((array.value[0] != 0 || array.value[1] != 0) && array.value[1] > 0.75)
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
      timer.Reset();
      lastTime = 0;
      doneWithPoleAlignment = false;
      turnt = false;
      swerveDrive->BeginPIDLoop();
      splineSection = 0.5; 
      limelight->TurnOnLimelight();
      turnedOffCone = false;
    }

    if (splineSection == 0.5)
    {
      if (!doneWithPoleAlignment)
      {
        claw->PIDWrist(0.9, elapsedTime);
      }
      bool lifted = false;
      if (claw->MagEncoderReading() > 0.25)
        lifted = elevatorLift->SetElevatorHeightPID(HIGHCONELIFTHEIGHT, elapsedTime);
      else 
        elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      bool centered = false;
      if (!turnt)
        turnt = swerveDrive->DriveToPose(Pose2d(swerveDrive->GetPose().Translation(), Rotation2d(180_deg)), elapsedTime);
      if (turnt && elevatorLift->winchEncoderReading() > 30)
      {
        double offsetX = limelight->getTargetX();
        double offsetY = limelight->getTargetY();
        centered = swerveDrive->StrafeToPole(offsetX, offsetY, HIGHCONEX, HIGHCONEY, elapsedTime);  
      }
      SmartDashboard::PutBoolean("cenetered", centered);
      SmartDashboard::PutBoolean("lifted", lifted);
      if ((centered && lifted) || timer.Get() > 3.5_s)
      {
        doneWithPoleAlignment = true;
      }
      if (doneWithPoleAlignment)
      {
        claw->PIDWrist(M_PI / 2, elapsedTime);
        if (claw->MagEncoderReading() > M_PI / 2 - 0.05 || timer.Get() > 4.25_s)
          claw->OpenClaw(elapsedTime);
        if (claw->ClawEncoderReading() > 5)
          splineSection = 0.9;
      }
    }

    if (splineSection == 0.9)
    {
      if (m_autoSelected == kAutoBR1GOB)
        swerveDrive->ResetOdometry(Pose2d(6.5_m, 1.91_m, swerveDrive->GetPose().Rotation()));
      else if (m_autoSelected == kAutoBL1GOB)
        swerveDrive->ResetOdometry(Pose2d(3.25_m, 1.91_m, swerveDrive->GetPose().Rotation()));
      else if (m_autoSelected == kAutoRR1GOB)
        swerveDrive->ResetOdometry(Pose2d(4_m, 1.91_m, swerveDrive->GetPose().Rotation()));
      else if (m_autoSelected == kAutoRL1GOB)
        swerveDrive->ResetOdometry(Pose2d(0.63_m, 1.91_m, swerveDrive->GetPose().Rotation()));
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
      if (swerveDrive->GetPose().Y() > 4.75_m)
        claw->PIDWristDown(elapsedTime);
      else
        claw->PIDWristUp(elapsedTime);
      bool splineDone = swerveDrive->FollowTrajectory(timer.Get(), elapsedTime);
      if (splineDone)
      {
        splineSection = 1.5;
        swerveDrive->SetNextTrajectory();
        swerveDrive->DriveSwervePercent(0,0,0);
        swerveDrive->ResetConeOdometry(Pose2d(0_m, -1_m, Rotation2d(0_deg)));
        turnt = false;
        coneInClaw = false;
        lastConeX = 0;
        lastConeY = -1;
        claw->BeginClawPID();
      }
    }

    if (splineSection == 1.5)
    {
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      claw->PIDWristDown(elapsedTime);

      if (swerveDrive->GetConeOdometryPose().Y().value() > 3 && turnt && !turnedOffCone) 
      {
        swerveDrive->ResetConeOdometry(Pose2d(units::meter_t{lastConeX}, units::meter_t{lastConeY}, Rotation2d(0_deg)));
        seeConesEntry.Set(false);
        turnedOffCone = true;
        SmartDashboard::PutBoolean("caught bad cone", true);
      }

      turnt = true; // maybe remove this later

      if (!turnt)
      {
        double angleGoal = atan2(-currentConeX, -currentConeY);
        turnt = swerveDrive->TurnToPixelCone(angleGoal, elapsedTime);
        goalConeGrabAngle = swerveDrive->GetPose().Rotation();
      }
      SmartDashboard::PutBoolean("turnt", turnt);
      if (turnt && !coneInClaw)
        coneInClaw = claw->ObjectInClaw();
      
      if (turnt && !coneInClaw)
        coneInClaw = swerveDrive->DriveToPoseConeOdometry(Pose2d(0_m, -0.58_m, goalConeGrabAngle), elapsedTime);
      else if (turnt)
        swerveDrive->DriveSwervePercent(0,0,0);

      if (!coneInClaw && swerveDrive->GetPose().Y() > 7_m)
      {
        swerveDrive->DriveSwervePercent(0,0,0);
        coneInClaw = true;
      }

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
        swerveDrive->StartBalance();
        swerveDrive->BeginPIDLoop();
      }
    }

    if (splineSection == 2.5)
    {
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      claw->PIDWrist(0.6, elapsedTime);
      claw->MoveClawPercent(0);
      if (masterTimer.Get() < 14.8_s)
        swerveDrive->BalanceOnCharger(elapsedTime);
      else
        swerveDrive->StrafeLock();
    }
  }




  
  else if (m_autoSelected == kAutoBConeB || m_autoSelected == kAutoRConeB)
  {
    double elapsedTime = timer.Get().value() - lastTime;
    swerveDrive->UpdateOdometry(timer.Get());
    swerveDrive->UpdateConeOdometry();

    if (splineSection == 0)
    {
      timer.Reset();
      lastTime = 0;
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
      bool lifted = elevatorLift->SetElevatorHeightPID(HIGHCONELIFTHEIGHT, elapsedTime); 
      bool centered = false;
      if (!turnt)
        turnt = swerveDrive->DriveToPose(Pose2d(swerveDrive->GetPose().Translation(), Rotation2d(180_deg)), elapsedTime);
      if (turnt && elevatorLift->winchEncoderReading() > 30)
      {
        double offsetX = limelight->getTargetX();
        double offsetY = limelight->getTargetY();
        centered = swerveDrive->StrafeToPole(offsetX, offsetY, HIGHCONEX, HIGHCONEY, elapsedTime);  
      }
      SmartDashboard::PutBoolean("cenetered", centered);
      SmartDashboard::PutBoolean("lifted", lifted);
      if ((centered && lifted) || timer.Get() > 4_s)
      {
        doneWithPoleAlignment = true;
      }
      if (doneWithPoleAlignment)
      {
        claw->PIDWrist(M_PI / 2, elapsedTime);
        if (claw->MagEncoderReading() > M_PI / 2 - 0.05 || timer.Get() > 5_s)
          claw->OpenClaw(elapsedTime);
        if (claw->ClawEncoderReading() > 5)
        {
          if (timer.Get() < 5_s)
          {
            if (m_autoSelected == kAutoBConeB)
              swerveDrive->ResetOdometry(Pose2d(4.86_m, 1.91_m, swerveDrive->GetPose().Rotation()));
            else if (m_autoSelected == kAutoRConeB)
              swerveDrive->ResetOdometry(Pose2d(2.35_m, 1.91_m, swerveDrive->GetPose().Rotation()));
          }
          splineSection = 0.9;
        }
      }
    }

    if (splineSection == 0.9)
    {
      claw->PIDWrist(0.5, elapsedTime);
      claw->OpenClaw(elapsedTime);
      if (claw->MagEncoderReading() < 0.75)
        elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      if (elevatorLift->winchEncoderReading() < 10)
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
      claw->PIDWrist(0.6, elapsedTime);
      bool splineDone = swerveDrive->FollowTrajectory(timer.Get(), elapsedTime);
      if (splineDone)
      {
        splineSection = 1.5;
        swerveDrive->SetNextTrajectory();
      }
    }

    if (splineSection == 1.5)
    {
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      claw->OpenClaw(elapsedTime);
      claw->PIDWrist(0.6, elapsedTime);
      swerveDrive->DriveSwervePercent(0, 0.4, 0);
      if (fabs(swerveDrive->GetIMURoll()) < 5 || swerveDrive->GetPose().Y() > 8.5_m)
      {
        preBalanceXGoal = swerveDrive->GetPose().X().value();
        preBalanceYGoal = swerveDrive->GetPose().Y().value() + 0.7;
        splineSection = 1.7;
      }
    }

    if (splineSection == 1.7)
    {
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      claw->OpenClaw(elapsedTime);
      claw->PIDWrist(0.6, elapsedTime);
      bool done = swerveDrive->DriveToPose(Pose2d(units::meter_t{preBalanceXGoal}, units::meter_t{preBalanceYGoal}, Rotation2d(180_deg)), elapsedTime);
      if (done)
      {
        splineSection = 1.9;
        timer.Reset();
        lastTime = 0;
      }
    }

    if (splineSection == 1.9)
    {
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      claw->OpenClaw(elapsedTime);
      claw->PIDWrist(0.6, elapsedTime);
      swerveDrive->DriveSwervePercent(0,0,0);
      if (timer.Get() > 0.5_s)
      {
        splineSection = 2;
        timer.Reset();
        lastTime = 0;
      }
    }

    if (splineSection == 2)
    {
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      claw->OpenClaw(elapsedTime);
      claw->PIDWrist(0.6, elapsedTime);
      bool splineDone = swerveDrive->FollowTrajectory(timer.Get(), elapsedTime);
      if (splineDone)
      {
        splineSection = 2.5;
        swerveDrive->StartBalance();
      }
    }

    // When you are done with the spline, balance using your code
    if (splineSection == 2.5)
    {
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      claw->PIDWrist(0.6, elapsedTime);
      claw->OpenClaw(elapsedTime);
      if (masterTimer.Get() < 14.8_s)
        swerveDrive->BalanceOnCharger(elapsedTime);
      else
        swerveDrive->StrafeLock();    
    }
  }






  else if (m_autoSelected == kAutoBCubeB || m_autoSelected == kAutoRCubeB)
  {
    double elapsedTime = timer.Get().value() - lastTime;
    swerveDrive->UpdateOdometry(timer.Get());
    swerveDrive->UpdateConeOdometry();
    swerveDrive->UpdateTagOdometry();

    for (auto array :cubeTagSub.ReadQueue())
    {
      Translation2d poseEst = Translation2d(units::meter_t{array.value[0]}, units::meter_t{array.value[1]});
      frc::SmartDashboard::PutNumber("Tag Vision X", poseEst.X().value());
      frc::SmartDashboard::PutNumber("Tag Vision Y", poseEst.Y().value());
      swerveDrive->ResetTagOdometry(Pose2d(poseEst, swerveDrive->GetPose().Rotation()));
    }
  
    if (splineSection == 0)
    {
      limelight->TurnOffLimelight();  
      seeCubeTagsEntry.Set(true);
      splineSection = 0.5; 
      conePlaceYLimelightGoal= 0.91;
      conePlaceXLimelightGoal = 0.075;
      conePlaceElevatorGoal = 72;  
      placingHigh = true;
      doneWithPoleAlignment = false;
      turnt = false;
      swerveDrive->ResetTagOdometry(Pose2d(0.075_m, 0.91_m, Rotation2d(180_deg)));
      swerveDrive->BeginPIDLoop();
    }

    if (splineSection == 0.5)
    {   
      if (!doneWithPoleAlignment)
      {
        claw->PIDWrist(0.9, elapsedTime);
      }
      bool lifted = elevatorLift->SetElevatorHeightPID(conePlaceElevatorGoal, elapsedTime);
      bool centered = false;
      SmartDashboard::PutBoolean("lifted", lifted);
      if (lifted)
      {
        centered = swerveDrive->DriveToPoseTag(Pose2d(units::meter_t{conePlaceXLimelightGoal}, units::meter_t{conePlaceYLimelightGoal}, Rotation2d(180_deg)), elapsedTime); 
      }
      SmartDashboard::PutBoolean("centered", centered);
      if (centered && lifted)
        doneWithPoleAlignment = true;
      if (doneWithPoleAlignment)
      {
        bool wristDown = claw->PIDWrist(M_PI / 2, elapsedTime);
        if (wristDown)
          claw->OpenClaw(elapsedTime);
      }

      if (claw->ClawEncoderReading() > 9)
      {
        splineSection = 0.9;
      }
    }

    if (splineSection == 0.9)
    {
      claw->PIDWrist(0.6, elapsedTime);
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
      claw->PIDWrist(0.6, elapsedTime);
      bool splineDone = swerveDrive->FollowTrajectory(timer.Get(), elapsedTime);
      if (splineDone)
      {
        splineSection = 1.5;
        swerveDrive->SetNextTrajectory();
      }
    }

    if (splineSection == 1.5)
    {
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      claw->OpenClaw(elapsedTime);
      claw->PIDWrist(0.6, elapsedTime);
      swerveDrive->DriveSwervePercent(0, 0.4, 0);
      if (fabs(swerveDrive->GetIMURoll()) < 5 || swerveDrive->GetPose().Y() > 8.5_m)
      {
        preBalanceXGoal = swerveDrive->GetPose().X().value();
        preBalanceYGoal = swerveDrive->GetPose().Y().value() + 0.7;
        splineSection = 1.7;
      }
    }

    if (splineSection == 1.7)
    {
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      claw->OpenClaw(elapsedTime);
      claw->PIDWrist(0.6, elapsedTime);
      bool done = swerveDrive->DriveToPose(Pose2d(units::meter_t{preBalanceXGoal}, units::meter_t{preBalanceYGoal}, Rotation2d(180_deg)), elapsedTime);
      if (done)
      {
        splineSection = 1.9;
        timer.Reset();
        lastTime = 0;
      }
    }

    if (splineSection == 1.9)
    {
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      claw->OpenClaw(elapsedTime);
      claw->PIDWrist(0.6, elapsedTime);
      swerveDrive->DriveSwervePercent(0,0,0);
      if (timer.Get() > 0.5_s)
      {
        splineSection = 2;
        timer.Reset();
        lastTime = 0;
      }
    }

    if (splineSection == 2)
    {
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      claw->OpenClaw(elapsedTime);
      claw->PIDWrist(0.6, elapsedTime);
      bool splineDone = swerveDrive->FollowTrajectory(timer.Get(), elapsedTime);
      if (splineDone)
      {
        splineSection = 2.5;
        swerveDrive->StartBalance();
      }
    }

    // When you are done with the spline, balance using your code
    if (splineSection == 2.5)
    {
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      claw->PIDWrist(0.6, elapsedTime);
      claw->OpenClaw(elapsedTime);
      if (masterTimer.Get() < 14.8_s)
        swerveDrive->BalanceOnCharger(elapsedTime);
      else
        swerveDrive->StrafeLock();
    }
  }





  else if (m_autoSelected == kAutoBConeDriveL || m_autoSelected == kAutoBConeDriveR || m_autoSelected == kAutoRConeDriveL || m_autoSelected == kAutoRConeDriveR)
  {
    double elapsedTime = timer.Get().value() - lastTime;
    swerveDrive->UpdateOdometry(timer.Get());
    swerveDrive->UpdateConeOdometry();
    for (auto array : coneEntry.ReadQueue())
    {
      if ((array.value[0] != 0 || array.value[1] != 0) && array.value[1] > 0.75)
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
      timer.Reset();
      turnedOffCone = false;
      lastTime = 0;
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
      bool lifted = elevatorLift->SetElevatorHeightPID(HIGHCONELIFTHEIGHT, elapsedTime); 
      bool centered = false;
      if (!turnt)
        turnt = swerveDrive->DriveToPose(Pose2d(swerveDrive->GetPose().Translation(), Rotation2d(180_deg)), elapsedTime);
      if (turnt && elevatorLift->winchEncoderReading() > 30)
      {
        double offsetX = limelight->getTargetX();
        double offsetY = limelight->getTargetY();
        centered = swerveDrive->StrafeToPole(offsetX, offsetY, HIGHCONEX, HIGHCONEY, elapsedTime);  
      }
      SmartDashboard::PutBoolean("cenetered", centered);
      SmartDashboard::PutBoolean("lifted", lifted);
      if ((centered && lifted) || timer.Get() > 4_s)
      {
        doneWithPoleAlignment = true;
      }
      if (doneWithPoleAlignment)
      {
        claw->PIDWrist(M_PI / 2, elapsedTime);
        if (claw->MagEncoderReading() > M_PI / 2 - 0.05 || timer.Get() > 5_s)
          claw->OpenClaw(elapsedTime);
        if (claw->ClawEncoderReading() > 5)
        {
          splineSection = 0.9;
          if (timer.Get() < 5_s)
          {
            if (m_autoSelected == kAutoBConeDriveL || m_autoSelected == kAutoBConeDriveR)
              swerveDrive->ResetOdometry(Pose2d(4.86_m, 1.91_m, swerveDrive->GetPose().Rotation()));
            else if (m_autoSelected == kAutoRConeDriveL || m_autoSelected == kAutoRConeDriveR)
              swerveDrive->ResetOdometry(Pose2d(2.35_m, 1.91_m, swerveDrive->GetPose().Rotation()));
          }
        }
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
      claw->PIDWristUp(elapsedTime);
      bool splineDone = swerveDrive->FollowTrajectory(timer.Get(), elapsedTime);
      if (splineDone)
      {
        splineSection = 1.5;
        swerveDrive->DriveSwervePercent(0,0,0);
        claw->MoveClawPercent(0);
        claw->MoveWristPercent(0);
        elevatorLift->MoveElevatorPercent(0);
      }
    }
  }










  else if (m_autoSelected == kAuto1GO)
  {
    double elapsedTime = timer.Get().value() - lastTime;
    swerveDrive->UpdateOdometry(timer.Get());
    swerveDrive->UpdateConeOdometry();
    for (auto array : coneEntry.ReadQueue())
    {
      if ((array.value[0] != 0 || array.value[1] != 0) && array.value[1] > 0.75)
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
      timer.Reset();
      turnedOffCone = false;
      lastTime = 0;
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
      bool lifted = elevatorLift->SetElevatorHeightPID(HIGHCONELIFTHEIGHT, elapsedTime); 
      bool centered = false;
      if (!turnt)
        turnt = swerveDrive->DriveToPose(Pose2d(swerveDrive->GetPose().Translation(), Rotation2d(180_deg)), elapsedTime);
      if (turnt && elevatorLift->winchEncoderReading() > 30)
      {
        double offsetX = limelight->getTargetX();
        double offsetY = limelight->getTargetY();
        centered = swerveDrive->StrafeToPole(offsetX, offsetY, HIGHCONEX, HIGHCONEY, elapsedTime);  
      }
      SmartDashboard::PutBoolean("cenetered", centered);
      SmartDashboard::PutBoolean("lifted", lifted);
      if ((centered && lifted) || timer.Get() > 4_s)
      {
        doneWithPoleAlignment = true;
      }
      if (doneWithPoleAlignment)
      {
        claw->PIDWrist(M_PI / 2, elapsedTime);
        if (claw->MagEncoderReading() > M_PI / 2 - 0.05 || timer.Get() > 5_s)
          claw->OpenClaw(elapsedTime);
        if (claw->ClawEncoderReading() > 5)
        {
          splineSection = 0.9;
        }
      }
    }

    if (splineSection == 0.9)
    {
      claw->PIDWrist(0.5, elapsedTime);
      swerveDrive->DriveSwervePercent(0,0,0);
      claw->OpenClaw(elapsedTime);
      if (claw->MagEncoderReading() < 0.75)
        elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      if (elevatorLift->winchEncoderReading() < 3)
      {
        claw->MoveClawPercent(0);
        claw->MoveWristPercent(0);
        elevatorLift->MoveElevatorPercent(0);
        timer.Reset();
        lastTime = 0;
        splineSection = 1;
      }
    } 
  }






  lastConeX = currentConeX;
  lastConeY = currentConeY;
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

  limelight->TurnOffLimelight();
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
//  limelight->TurnOnLimelight();

  // SmartDashboard::PutNumber("FL Mag", swerveDrive->FLModule->magEncoder->GetAbsolutePosition());
  // SmartDashboard::PutNumber("FR Mag", swerveDrive->FRModule->magEncoder->GetAbsolutePosition());
  // SmartDashboard::PutNumber("BL Mag", swerveDrive->BLModule->magEncoder->GetAbsolutePosition());
  // SmartDashboard::PutNumber("BR Mag", swerveDrive->BRModule->magEncoder->GetAbsolutePosition());
  // SmartDashboard::PutNumber("Pigeon", _pigeon.GetYaw());

  if (claw->ObjectInClaw() || claw->ObjectInClawSubstation())
    lights->SetLED("green");
  else if (claw->ClawEncoderReading() > 11)
    lights->SetLED("blue");
  else if (claw->ClawEncoderReading() < 5 && !claw->ObjectInClaw())
    lights->SetLED("red");
  else
    lights->SetLED();
  SmartDashboard::PutBoolean("cone in claw", claw->ObjectInClaw() || claw->ObjectInClawSubstation());
  SmartDashboard::PutBoolean("object is cone", claw->IsObjectCone());

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
  SmartDashboard::PutNumber("wrist backup encoder", claw->WristEncoderReading());
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
    MAX_DRIVE_SPEED = 0.98;
    MAX_SPIN_SPEED = 0.98;
    MAX_DRIVE_ACCELERATION = 4;
    MAX_SPIN_ACCELERATION = 4;
    if (elevatorLift->winchEncoderReading() < 10)
    {
      MAX_DRIVE_ACCELERATION = 8;
      MAX_SPIN_ACCELERATION = 8; 
    }
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

  // strafe lock
  if (xbox_Drive->GetXButton())
    lastFwdSpeed = 0;

    //Reset All Encoder and Gyro Values
  if (xbox_Drive->GetStartButton() && xbox_Drive->GetBackButton())
  {
    swerveDrive->ResetOdometry(Pose2d(0_m, 0_m, Rotation2d(0_deg)));
    elevatorLift->ResetElevatorEncoder();
  }

  if ((xbox_Drive2->GetLeftStickButtonPressed() && xbox_Drive2->GetRightStickButton()) || (xbox_Drive2->GetLeftStickButton() && xbox_Drive2->GetRightStickButtonPressed()))
  {
    claw->SetUsingBeamBreaks(!claw->GetUsingBeamBreaks());
    SmartDashboard::PutBoolean("using beam breaks", claw->GetUsingBeamBreaks());
  }


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
      if (xbox_Drive->GetXButton() && lastStrafeSpeed == 0 && lastTurnSpeed == 0 && lastFwdSpeed == 0)
        swerveDrive->StrafeLock();
      else
        swerveDrive->DriveSwervePercent(lastStrafeSpeed, lastFwdSpeed, lastTurnSpeed);

      double elevSpeed = 0;
      if (xbox_Drive2->GetLeftY() > 0.3)
        elevSpeed = -((xbox_Drive2->GetLeftY() - 0.3) * 4 / 7 + 0.3); 
      else if (xbox_Drive2->GetLeftY() < -0.3)
        elevSpeed = -((xbox_Drive2->GetLeftY() + 0.3) * 4 / 7 - 0.3); 
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
        claw->PIDWristDown(elapsedTime);
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
      else if (elevatorLift->winchEncoderReading() < 20 && claw->MagEncoderReading() > 2.1 && lastWristSpeed < 0)
        lastWristSpeed = 0;
      else if (claw->MagEncoderReading() > 2.75 && lastWristSpeed < 0)
        lastWristSpeed = 0;
      
      if (!elevatorPIDWrist)
        claw->MoveWristPercent(lastWristSpeed); 

      if (xbox_Drive2->GetLeftTriggerAxis() > 0.5)
        claw->MoveClawPercent(0.7);  
      else if (xbox_Drive2->GetRightTriggerAxis() > 0.5)
        claw->MoveClawPercent(-0.9);
      else
        claw->MoveClawPercent(0);

      // Trigger Autonomous Commands
      if (!claw->IsObjectCone())
      {
        //CUBESSSSSSS
        if (xbox_Drive2->GetYButtonPressed())
        {
          //High Cube
          seeCubesEntry.Set(true);
          conePlaceYLimelightGoal= 0.91;
          conePlaceXLimelightGoal = 0.075;
          conePlaceElevatorGoal = 72;  
          placingHigh = true;
          doneWithPoleAlignment = false;
          turnt = false;
          swerveDrive->BeginPIDLoop();
          currentDriverSection = SCORINGCUBE;
        }
        else if (xbox_Drive2->GetBButtonPressed())
        {
          //Mid Cube
          seeCubesEntry.Set(true);
          conePlaceYLimelightGoal= 0.91;
          conePlaceXLimelightGoal = 0.075;
          conePlaceElevatorGoal = 48;  
          placingHigh = false;
          doneWithPoleAlignment = false;
          turnt = false;
          swerveDrive->BeginPIDLoop();
          currentDriverSection = SCORINGCUBE;
        }
        else if (xbox_Drive2->GetXButtonPressed())
        {
          //Prepare High
          placingHigh = true;
          currentDriverSection = LIFTINGTOPLACEOBJECT;
          conePlaceElevatorGoal = 72;  
        }
        else if (xbox_Drive2->GetAButtonPressed())
        {
          //Prepare Mid
          placingHigh = false;
          conePlaceElevatorGoal = 48;  
          currentDriverSection = LIFTINGTOPLACEOBJECT;
        }
      }
      else
      {
        if (xbox_Drive2->GetBButtonPressed())
        {
          // Mid Post
          limelight->TurnOnLimelight();
          if (claw->ObjectInClawSubstation())
          {
            conePlaceXLimelightGoal = 0.19; 
            conePlaceYLimelightGoal = -0.07;
            conePlaceElevatorGoal = 47;  
          }
          else
          {
            conePlaceXLimelightGoal = 0.19; 
            conePlaceYLimelightGoal = -0.055;
            conePlaceElevatorGoal = 47;  
          }
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
          if (claw->ObjectInClawSubstation())
          {
            conePlaceXLimelightGoal = 0.19; 
            conePlaceYLimelightGoal = -0.07;
            conePlaceElevatorGoal = 82;  
          }
          else
          {
            conePlaceXLimelightGoal = 0.19; 
            conePlaceYLimelightGoal = -0.055;
            conePlaceElevatorGoal = 82;  
          }
          placingHigh = true;
          doneWithPoleAlignment = false;
          turnt = false;
          swerveDrive->BeginPIDLoop();
          currentDriverSection = SCORINGCONE;
        }
        else if (xbox_Drive2->GetXButtonPressed())
        {
          //Prepare High
          placingHigh = true;
          conePlaceElevatorGoal = 82;  
          currentDriverSection = LIFTINGTOPLACEOBJECT;
        }
        else if (xbox_Drive2->GetAButtonPressed())
        {
          //Prepare Mid
          placingHigh = false;
          conePlaceElevatorGoal = 47;  
          currentDriverSection = LIFTINGTOPLACEOBJECT;
        }
      }


      if (xbox_Drive2->GetBackButtonPressed())
      {
        currentDriverSection = ZEROING;
      }
      else if (xbox_Drive2->GetPOV() == 90 || xbox_Drive2->GetPOV() == 45 || xbox_Drive2->GetPOV() == 135)
      {
        clawFinishedOpening = false;
        coneInClaw = false;
        currentDriverSection = ANGLEDSUBSTATION;
      }
      else if (xbox_Drive2->GetPOV() == 315 || xbox_Drive2->GetPOV() == 225 || xbox_Drive2->GetPOV() == 270)
      {
        seeSubstationTagsEntry.Set(true);
        seeCubeTagsEntry.Set(false);
        turnt = false;
        centeredOnSubstation = false;
        claw->BeginClawPID();
        coneInClaw = false;
        clawFinishedOpening = false;
        leftSubstation = true;
        currentDriverSection = SUBSTATIONINTAKING;
      }
      else if (xbox_Drive2->GetStartButtonPressed())
      {
        coneInClaw = false;
        claw->BeginClawPID();
        currentDriverSection = GROUNDINTAKING;      
      }
      else if (xbox_Drive->GetRightTriggerAxis() > 0.5)
      {
        currentDriverSection = UNTIPCONE;
      }
      else if (xbox_Drive->GetLeftTriggerAxis() > 0.5)
      {
        //coneInClaw = false;
        //claw->BeginClawPID();
        //currentDriverSection = GROUNDPREPAREDTOGRAB;
      }
      else if (xbox_Drive->GetBButtonPressed())
      {
        turnt = false;
        coneInClaw = false;
        claw->BeginClawPID();
        currentDriverSection = AUTOGRABBINGCONE;
      }
      else if (xbox_Drive->GetYButtonPressed())
      {
        swerveDrive->StartBalance();
        currentDriverSection = BALANCE;
      }
      break;
    }

    case LIFTINGTOPLACEOBJECT:
    {
      swerveDrive->DriveSwervePercent(lastStrafeSpeed, lastFwdSpeed, lastTurnSpeed);
      if (xbox_Drive2->GetLeftTriggerAxis() > 0.5)
        claw->MoveClawPercent(0.5);  
      else if (xbox_Drive2->GetRightTriggerAxis() > 0.5)
        claw->MoveClawPercent(-0.8);
      else
        claw->MoveClawPercent(0);

      elevatorLift->SetElevatorHeightPID(conePlaceElevatorGoal, elapsedTime);
      claw->PIDWrist(0.9, elapsedTime);

      if ((!xbox_Drive2->GetAButton() && !placingHigh) || (!xbox_Drive2->GetXButton() && placingHigh))
         currentDriverSection = RESUMEDRIVING;
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

      if ((!xbox_Drive2->GetYButton() && placingHigh) || (!xbox_Drive2->GetBButton() && !placingHigh))
      {
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
        claw->PIDWristUp(elapsedTime);
      else if (elevatorLift->winchEncoderReading() > 65)
        claw->PIDWrist(0.6, elapsedTime);
      else
        claw->PIDWrist(1, elapsedTime);

      if (!xbox_Drive2->GetBackButton())
         currentDriverSection = RESUMEDRIVING;
      break;
    }

    case GROUNDINTAKING:
    {
      swerveDrive->DriveSwervePercent(lastStrafeSpeed, lastFwdSpeed, lastTurnSpeed);
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      
      bool clawClosed = false;
      if ((claw->ObjectInClaw() && claw->ClawEncoderReading() > 5) || coneInClaw)
      {
        clawClosed = claw->CloseClaw(elapsedTime);
        coneInClaw = true;
      }
      else 
      {
        claw->OpenClaw(elapsedTime);
      }

      if (clawClosed)
        claw->PIDWristUp(elapsedTime);
      else
        claw->PIDWristDown(elapsedTime);

      if (!xbox_Drive2->GetStartButton())
        currentDriverSection = RESUMEDRIVING;
      break;
    }

    case SUBSTATIONINTAKING:
    {
      /*if (leftSubstation)
      {
        bool lifted = elevatorLift->SetElevatorHeightPID(81, elapsedTime);
        claw->PIDWristDown(elapsedTime);
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
      {*/
        swerveDrive->DriveSwervePercent(lastStrafeSpeed, lastFwdSpeed, lastTurnSpeed);

        if (claw->MagEncoderReading() < 0.3)
          elevatorLift->MoveElevatorPercent(0);
        else
          elevatorLift->SetElevatorHeightPID(69, elapsedTime);

        if (!coneInClaw && elevatorLift->winchEncoderReading() > 65)
          coneInClaw = claw->ObjectInClaw() || claw->ObjectInClawSubstation();
//1.37
        claw->PIDWrist(1.37, elapsedTime);
        if (!clawFinishedOpening)
          clawFinishedOpening = claw->OpenClaw(elapsedTime);
        else if (coneInClaw)
          claw->CloseClaw(elapsedTime);
        else 
        {
           if (xbox_Drive2->GetLeftTriggerAxis() > 0.5)
            claw->MoveClawPercent(0.5);  
          else if (xbox_Drive2->GetRightTriggerAxis() > 0.5)
            claw->MoveClawPercent(-0.8);
          else
            claw->MoveClawPercent(0);     
        }
    //  }

      if (xbox_Drive2->GetPOV() == -1)
      {
        seeSubstationTagsEntry.Set(false);
        seeCubeTagsEntry.Set(true);
     /* if (leftSubstation)
          currentDriverSection = BEGINDRIVING;
        else*/
          currentDriverSection = RESUMEDRIVING; 
      }
      break;
    }

    case ANGLEDSUBSTATION:
    {
      swerveDrive->DriveSwervePercent(lastStrafeSpeed, lastFwdSpeed, lastTurnSpeed);
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      
      bool clawClosed = false;
      if (((claw->ObjectInClaw() || claw->ObjectInClawSubstation()) && claw->ClawEncoderReading() > 5) || coneInClaw)
      {
        clawClosed = claw->CloseClaw(elapsedTime);
        coneInClaw = true;
      }
      else 
      {
        claw->OpenClaw(elapsedTime);
      }

      if (clawClosed)
        claw->PIDWristUp(elapsedTime);
      else
        claw->PIDWristDown(elapsedTime);

      if (xbox_Drive2->GetPOV() == -1)
        currentDriverSection = RESUMEDRIVING;
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
      bool done = swerveDrive->BalanceOnCharger(elapsedTime);
      SmartDashboard::PutBoolean("done with balance", done);
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      if (elevatorLift->winchEncoderReading() < 5)
        claw->PIDWristUp(elapsedTime);
      else
        claw->PIDWrist(1, elapsedTime);
      claw->OpenClaw(elapsedTime);
      if(!xbox_Drive->GetYButton())
        currentDriverSection = BEGINDRIVING;
      break;
    }

    case AUTOGRABBINGCONE:
      elevatorLift->SetElevatorHeightPID(0, elapsedTime);
      claw->PIDWristDown(elapsedTime);
      turnt = true;
      if (!turnt)
      {
        double angleGoal = atan2(-currentConeX, -currentConeY);
        SmartDashboard::PutNumber("cone angle", angleGoal);
        turnt = swerveDrive->TurnToPixelCone(angleGoal, elapsedTime);
        goalConeGrabAngle = swerveDrive->GetPose().Rotation();
      }
      coneInClaw = claw->ObjectInClaw();
      if (turnt && !coneInClaw)
        coneInClaw = swerveDrive->DriveToPoseConeOdometry(Pose2d(0_m, -0.58_m, goalConeGrabAngle), elapsedTime);
      if (coneInClaw)
        claw->CloseClaw(elapsedTime);
      else
        claw->OpenClaw(elapsedTime);

      if (!xbox_Drive->GetBButton())
        currentDriverSection = BEGINDRIVING;
      break;
  } 

  limelight->getTargetX();
  limelight->getTargetY();
  SmartDashboard::PutBoolean("object in claw regular", claw->ObjectInClaw());
  SmartDashboard::PutBoolean("object in claw substation", claw->ObjectInClawSubstation());
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
