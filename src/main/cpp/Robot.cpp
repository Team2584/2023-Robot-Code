// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Setup.h"

#include "Swerve.cpp"

#include <fmt/core.h>
#include <frc/livewindow/LiveWindow.h>

#include <frc/smartdashboard/SmartDashboard.h>

double pigeon_initial;
// Our future subsystem objects
// Suggestion: It might be safer to have these as shared pointers instead of raw pointers.
SwerveDrive *swerveDrive;

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

  // Setting motor breaktypes
  driveFL.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  driveBL.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  driveFR.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  driveBR.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  // Initializing things
  timer = Timer();
  masterTimer = Timer();

  // Initializing Subsystems
  swerveDrive = new SwerveDrive(&driveFL, &swerveFL, &FLMagEnc, FL_WHEEL_OFFSET, &driveFR, &swerveFR, &FRMagEnc,
                                FR_WHEEL_OFFSET, &driveBR, &swerveBR, &BRMagEnc, BR_WHEEL_OFFSET, &driveBL,
                                &swerveBL, &BLMagEnc, BL_WHEEL_OFFSET, &_pigeon, STARTING_DRIVE_HEADING);

  //limelight->TurnOffLimelight();
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
  }

void Robot::AutonomousPeriodic()
{
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

   /*SmartDashboard::PutNumber("FL Mag", swerveDrive->FLModule->magEncoder->GetAbsolutePosition());
   SmartDashboard::PutNumber("FR Mag", swerveDrive->FRModule->magEncoder->GetAbsolutePosition());
   SmartDashboard::PutNumber("BL Mag", swerveDrive->BLModule->magEncoder->GetAbsolutePosition());
   SmartDashboard::PutNumber("BR Mag", swerveDrive->BRModule->magEncoder->GetAbsolutePosition());
   SmartDashboard::PutNumber("Pigeon", _pigeon.GetYaw());*/

  // update our timer
  double time = timer.Get().value();
  double elapsedTime = time - lastTime;
  lastTime = time;

  // Update our odometry
  double microsecondTime = (double)RobotController::GetFPGATime();
  swerveDrive->UpdateOdometry(units::microsecond_t{microsecondTime});

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
  if (xbox_Drive->GetRightBumper())
  {
    MAX_DRIVE_SPEED = 0.98;
    MAX_SPIN_SPEED = 0.98;
    MAX_DRIVE_ACCELERATION = 4;
    MAX_SPIN_ACCELERATION = 4;
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

  SmartDashboard::PutNumber("FWD Drive", FWD_Drive_Speed);
  SmartDashboard::PutNumber("STRAFE Drive", STRAFE_Drive_Speed);
  SmartDashboard::PutNumber("Turn Drive", Turn_Speed);

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
  }

  swerveDrive->DriveSwervePercent(lastStrafeSpeed, lastFwdSpeed, lastTurnSpeed);
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
