#pragma once

#include <string>

#include "ctre/Phoenix.h"
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
//#include "rev/CANEncoder.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"
#include "ctre/phoenix/sensors/Pigeon2.h"
#include "ctre/phoenix/music/Orchestra.h"

#include <fmt/core.h>

#include <frc/DriverStation.h>
#include <frc/PS4Controller.h>
#include <frc/TimedRobot.h>
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
#include <frc/filter/SlewRateLimiter.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/XboxController.h>

#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>

#include "cameraserver/CameraServer.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <frc/DutyCycleEncoder.h>
#include <frc/PWM.h>
#include <networktables/NetworkTableEntry.h>
#include <wpi/future.h>
#include <wpi/sendable/SendableRegistry.h>

#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/area.h>
#include <units/capacitance.h>
#include <units/charge.h>
#include <units/concentration.h>
#include <units/conductance.h>
#include <units/current.h>
#include <units/curvature.h>
#include <units/data.h>
#include <units/data_transfer_rate.h>
#include <units/density.h>
#include <units/dimensionless.h>
#include <units/energy.h>
#include <units/force.h>
#include <units/frequency.h>
#include <units/illuminance.h>
#include <units/impedance.h>
#include <units/inductance.h>
#include <units/length.h>
#include <units/luminous_flux.h>
#include <units/luminous_intensity.h>
#include <units/magnetic_field_strength.h>
#include <units/magnetic_flux.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>
#include <units/power.h>
#include <units/pressure.h>
#include <units/radiation.h>
#include <units/solid_angle.h>
#include <units/substance.h>
#include <units/temperature.h>
#include <units/time.h>
#include <units/torque.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/volume.h>

#include <cmath>
#include <iostream>
#include <math.h>
#include <thread>

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
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
