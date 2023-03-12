#pragma once

#include "ctre/Phoenix.h"
#include <frc/TimedRobot.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "rev/CANSparkMax.h"
#include "rev/AbsoluteEncoder.h"
#include "rev/SparkMaxLimitSwitch.h"

#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"
#include "ctre/phoenix/sensors/Pigeon2.h"
#include "ctre/phoenix/music/Orchestra.h"

#include <fmt/core.h>

#include <frc/DriverStation.h>
#include <frc/PS4Controller.h>
#include <frc/Timer.h>
#include <frc/fmt/Units.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/encoder.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/Filesystem.h>
#include <wpi/fs.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/XboxController.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/PWM.h>
#include <frc/AnalogInput.h>

#include <pathplanner/lib/PathPlanner.h>

#include "cameraserver/CameraServer.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/DoubleArrayTopic.h"
#include "networktables/DoubleTopic.h"
#include "networktables/StringTopic.h"
#include "networktables/BooleanTopic.h"
#include "networktables/IntegerTopic.h"
#include <networktables/RawTopic.h>
#include <networktables/NetworkTableValue.h>
#include "networktables/NetworkTableInstance.h"
#include <networktables/NetworkTableEntry.h>

#include <wpi/future.h>
#include <wpi/sendable/SendableRegistry.h>

#include <TimeOfFlight.h>

#include <string>
#include <string_view>
#include <cmath>
#include <iostream>
#include <math.h>
#include <thread>
#include <queue>

using namespace std;
using namespace frc;
using namespace frc2;

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoRR2GO = "RED Right Place 2 Cones";
  const std::string kAutoRL2GO = "RED Left Place 2 Cones";
  const std::string kAutoRConeB = "RED Center Place Cone -> Balance";
  const std::string kAutoRCubeB = "RED Center Place Cube -> Balance";
  const std::string kAutoBConeB = "BLUE Center Place Cone -> Balance";
  const std::string kAutoBCubeB = "BLUE Center Place Cube -> Balance";
  const std::string kAutoBR2GO = "BLUE Right Place 2 Cones";
  const std::string kAutoBL2GO = "BLUE Left Place 2 Cones";
  const std::string kAutoRR1GOB = "RED Right Place Cone -> Grab Cone -> Balance";
  const std::string kAutoBL1GOB = "BLUE Left Place Cone -> Grab Cone -> Balance";  
  const std::string kAutoRL1GOB = "RED Left Place Cone -> Grab Cone -> Balance";
  const std::string kAutoBR1GOB = "BLUE Right Place Cone -> Grab Cone -> Balance";
  const std::string kAuto1GO = "Just Place Cone";
  const std::string kAutoRRCubeCone = "RED Right Place Cube -> Place Cone";
  const std::string kAutoRLCubeCone = "RED Left Place Cube -> Place Cone";
  const std::string kAutoBRCubeCone = "BLUE Right Place Cube -> Place Cone";
  const std::string kAutoBLCubeCone = "BLUE Left Place Cube -> Place Cone";
  const std::string kAutoRRCubeConeCone = "RED Right Place Cube -> Place Cone -> Grab Cone";
  const std::string kAutoRLCubeConeCone = "RED Left Place Cube -> Place Cone -> Grab Cone";
  const std::string kAutoBRCubeConeCone = "BLUE Right Place Cube -> Place Cone -> Grab Cone";
  const std::string kAutoBLCubeConeCone = "BLUE Left Place Cube -> Place Cone -> Grab Cone";
  const std::string kAutoRConeDriveR = "RED Center Place Cone -> Drive out Right";
  const std::string kAutoRConeDriveL = "RED Center Place Cone -> Drive out Left";
  const std::string kAutoBConeDriveR = "BLUE Center Place Cone -> Drive out Right";
  const std::string kAutoBConeDriveL = "BLUE Center Place Cone -> Drive out Left";
  std::string m_autoSelected;
};
