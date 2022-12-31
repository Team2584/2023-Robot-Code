#include "SwerveConstants.h"
#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>

class SwerveModule
{
private:
  // Instance Variables for each swerve module
  ctre::phoenix::motorcontrol::can::TalonFX *driveMotor, *spinMotor;
  frc::DutyCycleEncoder *magEncoder;
  frc::PIDController *spinPIDController;
  double encoderOffset;
  double driveEncoderInitial;
  double spinEncoderInitialHeading;
  double spinEncoderInitialValue;

public:
  // Constructor for swerve module, setting all instance variables
  SwerveModule(ctre::phoenix::motorcontrol::can::TalonFX *driveMotor_,
               ctre::phoenix::motorcontrol::can::TalonFX *spinMotor_, frc::DutyCycleEncoder *magEncoder_,
               double encoderOffset_)
  {
    driveMotor = driveMotor_;
    spinMotor = spinMotor_;
    magEncoder = magEncoder_;
    encoderOffset = encoderOffset_;
    ResetEncoders();
  }

  // Converts Mag-Encoder Reading to a radian 0 - 2pi
  double GetMagEncoderReading()
  {
    double encoderReading = magEncoder->GetAbsolutePosition();
    // subtract the encoder offset to make 0 degrees forward
    encoderReading -= encoderOffset;
    if (encoderReading < 0)
      encoderReading += 1;
    // Flip the degrees to make clockwise positive
    encoderReading = 1 - encoderReading;
    // Convert from 0-1 to degrees
    encoderReading *= 2 * M_PI;
    return encoderReading;
  }

  void ResetEncoders()
  {
    driveEncoderInitial = driveMotor->GetSelectedSensorPosition();
    spinEncoderInitialHeading = GetMagEncoderReading();
    spinEncoderInitialValue = -1 * spinMotor->GetSelectedSensorPosition();
  }

  // Converts Talon Drive Encoder to Meters
  double GetDriveEncoderMeters()
  {
    return (driveMotor->GetSelectedSensorPosition() - driveEncoderInitial) / 2048 / DRIVE_MOTOR_GEAR_RATIO * DRIVE_MOTOR_CIRCUMFERENCE;
  } 

  // Finds Drive Motor Velocity in Meters per Second
  double GetDriveVelocity()
  {
    return driveMotor->GetSelectedSensorVelocity() / 2048 / 6.54 * 0.10322 * M_PI * 10;
  }

  //Finds Spin Encoder Rotation in Radians
  double GetSpinEncoderRadians()
  {
    //TODO
    double rotation = ((-1 * spinMotor->GetSelectedSensorPosition() - spinEncoderInitialValue) / 2048 / SPIN_MOTOR_GEAR_RATIO * 2 * M_PI) - spinEncoderInitialHeading;
    return fmod(rotation, 2 * M_PI);
  }

  //Stops all motor velocity in swerve module
  void StopSwerveModule()
  {
    spinMotor->Set(ControlMode::PercentOutput, 0);
    driveMotor->Set(ControlMode::PercentOutput, 0);
  }

  //Returns the swerve module's state
  SwerveModuleState GetSwerveModuleState()
  {
    SwerveModuleState state = SwerveModuleState();
    state.speed = units::meters_per_second_t{fabs(GetDriveVelocity())};
    state.angle = Rotation2d(units::radian_t{GetMagEncoderReading() / 180 * M_PI});
    return state;
  }

  //Returns the swerve module's position
  SwerveModulePosition GetSwerveModulePosition()
  {
    SwerveModulePosition state = SwerveModulePosition();
    state.distance = units::length::centimeter_t{GetDriveEncoderMeters() * 100}; // It works, don't judge
    state.angle = Rotation2d(units::radian_t{GetMagEncoderReading()});
    return state;
  }

  // Spin swerve module motors to reach the drive speed and spin angle 
  void DriveSwerveModulePercent(double driveSpeed, double targetAngle)
  {
    // current encoder reading as an angle
    double wheelAngle = GetMagEncoderReading() * 180 / M_PI;
    // amount wheel has left to turn to reach target
    double error = 0;
    // if wheel should spin clockwise(1) or counterclockwise(-1) to reach the target
    int spinDirection = 0;
    // If the drive should spin forward(1) or backward(-1) to move in the correct direction
    int driveDirection = 0;

    // Corrects spin angle to make it positive
    if (targetAngle < 0)
    {
      targetAngle += 360;
    }

    // The below logic determines the most efficient way for the wheel to move to reach the desired angle
    // This could mean moving towards it clockwise, counterclockwise, or moving towards the opposite of the angle
    // and driving in the opposite direction
    if (wheelAngle < targetAngle)
    {
      // if target and wheelangle are less than 90 degrees apart we should spin directly towards the target angle
      if (targetAngle - wheelAngle <= 90)
      {
        error = targetAngle - wheelAngle;
        spinDirection = 1;
        driveDirection = 1;
      }
      // else if target angle is "1 quadrant" away from the wheel Angle spin counterclockwise to the opposite of
      // targetAngle and spin the drive motors in the opposite direction
      else if (targetAngle - wheelAngle <= 180)
      {
        // Distance the wheel must spin is now not the distance between the target and the wheelAngle, but rather
        // the distance between the opposite of the target and the wheelAngle
        error = 180 - (targetAngle - wheelAngle);
        spinDirection = -1;
        driveDirection = -1;
      }
      else if (targetAngle - wheelAngle <= 270)
      {
        error = (targetAngle - wheelAngle) - 180;
        spinDirection = 1;
        driveDirection = -1;
      }
      // if target and wheelAngle are less than 90 degrees apart we should spin directly towards the target angle
      // Here however, we must reverse the spin direction because the target is counterclockwise of the wheelAngle
      else
      {
        error = 360 - (targetAngle - wheelAngle);
        spinDirection = -1;
        driveDirection = 1;
      }
    }
    else if (wheelAngle > targetAngle)
    {
      // The logic below is similar to the logic above, but in the case where wheelAngle > targetAngle
      if (wheelAngle - targetAngle <= 90)
      {
        error = wheelAngle - targetAngle;
        spinDirection = -1;
        driveDirection = 1;
      }
      else if (wheelAngle - targetAngle <= 180)
      {
        error = 180 - (wheelAngle - targetAngle);
        spinDirection = 1;
        driveDirection = -1;
      }
      else if (wheelAngle - targetAngle <= 270)
      {
        error = (wheelAngle - targetAngle) - 180;
        spinDirection = -1;
        driveDirection = -1;
      }
      else
      {
        error = 360 - (wheelAngle - targetAngle);
        spinDirection = 1;
        driveDirection = 1;
      }
    }

    // simple P of PID, makes the wheel move slower as it reaches the target
    double output = WHEEL_SPIN_KP * (error / 90);

    // Move motors at speeds and directions determined earlier
    spinMotor->Set(ControlMode::PercentOutput, output * spinDirection);
    driveMotor->Set(ControlMode::PercentOutput, driveSpeed * driveDirection);
  }

  // Spin swerve module motors to reach the drive speed and spin angle 
  void DriveSwerveModuleMeters(double driveSpeed, double targetRadian)
  {
    DriveSwerveModulePercent(driveSpeed / SWERVE_DRIVE_MAX_MPS, targetRadian);
  }
};

class SwerveDrive
{
private:
  Pigeon2 *pigeonIMU;
  Translation2d m_frontLeft;
  Translation2d m_frontRight;
  Translation2d m_backLeft;
  Translation2d m_backRight;
  SwerveDriveKinematics<4> kinematics;
  SwerveDriveOdometry<4> *odometry;
  Trajectory currentTrajectory;

  Pose2d visionPose;
  double timeSinceOdometryRefresh;

  double lastX;
  double lastY;
  double lastSpin;

public:
  SwerveModule *FLModule, *FRModule, *BRModule, *BLModule;
  double pigeon_initial;

  // Instantiates SwerveDrive class by creating 4 swerve modules
  SwerveDrive(ctre::phoenix::motorcontrol::can::TalonFX *_FLDriveMotor,
              ctre::phoenix::motorcontrol::can::TalonFX *_FLSpinMotor, frc::DutyCycleEncoder *_FLMagEncoder,
              double _FLEncoderOffset, ctre::phoenix::motorcontrol::can::TalonFX *_FRDriveMotor,
              ctre::phoenix::motorcontrol::can::TalonFX *_FRSpinMotor, frc::DutyCycleEncoder *_FRMagEncoder,
              double _FREncoderOffset, ctre::phoenix::motorcontrol::can::TalonFX *_BRDriveMotor,
              ctre::phoenix::motorcontrol::can::TalonFX *_BRSpinMotor, frc::DutyCycleEncoder *_BRMagEncoder,
              double _BREncoderOffset, ctre::phoenix::motorcontrol::can::TalonFX *_BLDriveMotor,
              ctre::phoenix::motorcontrol::can::TalonFX *_BLSpinMotor, frc::DutyCycleEncoder *_BLMagEncoder,
              double _BLEncoderOffset, Pigeon2 *_pigeonIMU, double robotStartingRadian)
  //TODO CLEAN based off of drive width / length
  : m_frontLeft{0.29845_m, 0.2953_m},
    m_frontRight{0.29845_m, -0.2953_m},
    m_backLeft{-0.29845_m, 0.2953_m},
    m_backRight{-0.29845_m, -0.2953_m},
    kinematics{m_frontLeft, m_frontRight, m_backLeft, m_backRight}
  {
    FLModule = new SwerveModule(_FLDriveMotor, _FLSpinMotor, _FLMagEncoder, _FLEncoderOffset);
    FRModule = new SwerveModule(_FRDriveMotor, _FRSpinMotor, _FRMagEncoder, _FREncoderOffset);
    BLModule = new SwerveModule(_BLDriveMotor, _BLSpinMotor, _BLMagEncoder, _BLEncoderOffset);
    BRModule = new SwerveModule(_BRDriveMotor, _BRSpinMotor, _BRMagEncoder, _BREncoderOffset);
    
    pigeonIMU = _pigeonIMU;

    wpi::array<SwerveModulePosition, 4> positions = {FLModule->GetSwerveModulePosition(),
      FRModule->GetSwerveModulePosition(),
      BLModule->GetSwerveModulePosition(),
      BRModule->GetSwerveModulePosition()};

    //will screw up when robot doesn't start at 0 degrees
    odometry = new SwerveDriveOdometry<4>(kinematics, 
    Rotation2d(units::radian_t{GetIMURadians()}), 
    positions, 
    frc::Pose2d(0_m, 0_m, Rotation2d(units::radian_t{robotStartingRadian})));
  }

  double GetIMURadians()
  {
    double pigeon_angle = fmod(pigeonIMU->GetYaw(), 360);
    pigeon_angle -= pigeon_initial;
    if (pigeon_angle < 0)
      pigeon_angle += 360;
    pigeon_angle = 360 - pigeon_angle;
    if (pigeon_angle == 360)
      pigeon_angle = 0;
    pigeon_angle *= M_PI / 180;
    return pigeon_angle;
  }

  void ResetOdometry()
  {
    ResetOdometry(Pose2d(0_m, 0_m, Rotation2d(0_rad)));
  }

  //Resets Odometry
  void ResetOdometry(Pose2d position)
  {
    FLModule->ResetEncoders();
    FRModule->ResetEncoders();
    BLModule->ResetEncoders();
    BRModule->ResetEncoders();

    wpi::array<SwerveModulePosition, 4> positions = {FLModule->GetSwerveModulePosition(),
      FRModule->GetSwerveModulePosition(),
      BLModule->GetSwerveModulePosition(),
      BRModule->GetSwerveModulePosition()};

    odometry->ResetPosition( 
    Rotation2d(units::radian_t{GetIMURadians()}), 
    positions, 
    frc::Pose2d(Pose2d(position.Y(), position.X(), position.Rotation())));
  }

  void UpdateOdometry()
  {
    SmartDashboard::PutNumber("ROBOT ANGLE", GetIMURadians());
    wpi::array<SwerveModulePosition, 4> positions = {FLModule->GetSwerveModulePosition(),
      FRModule->GetSwerveModulePosition(),
      BLModule->GetSwerveModulePosition(),
      BRModule->GetSwerveModulePosition()};
    odometry->Update(units::radian_t{GetIMURadians()}, positions);
  }

  Pose2d GetPose()
  {
    return GetPoseOdometry();
  }

  Pose2d GetPoseOdometry()
  {
    Pose2d pose = odometry->GetPose();
    return Pose2d(pose.Y(), pose.X(), pose.Rotation()); 
  }

  void SetPoseVision(Pose2d pose)
  {
    visionPose = pose;
  }

  Pose2d GetPoseVision()
  {
    return visionPose;
  }

  void SetModuleStates(std::array<SwerveModuleState, 4> states)
  { 
    FRModule->DriveSwerveModuleMeters(states[0].speed.value(), states[0].angle.Degrees().value());
    FLModule->DriveSwerveModuleMeters(states[1].speed.value(), states[1].angle.Degrees().value());
    BRModule->DriveSwerveModuleMeters(states[2].speed.value(), states[2].angle.Degrees().value());
    BLModule->DriveSwerveModuleMeters(states[3].speed.value(), states[3].angle.Degrees().value());
  }

  void DriveSwervePercent(double STRAFE_Drive_Speed, double FWD_Drive_Speed, double Turn_Speed)
  {
    double angle = GetPose().Rotation().Radians().value();
    double oldFwd = FWD_Drive_Speed;
    FWD_Drive_Speed = FWD_Drive_Speed * cos(angle) + STRAFE_Drive_Speed * sin(angle);
    STRAFE_Drive_Speed = -1 * oldFwd * sin(angle) + STRAFE_Drive_Speed * cos(angle);

    SmartDashboard::PutNumber("Real FWD Drive", FWD_Drive_Speed);
    SmartDashboard::PutNumber("Real STRAFE Drive", STRAFE_Drive_Speed);
    SmartDashboard::PutNumber("Real Angle", angle);

    // If there is no drive input, don't drive the robot and just end the function
    if (FWD_Drive_Speed == 0 && STRAFE_Drive_Speed == 0 && Turn_Speed == 0)
    {
      FLModule->StopSwerveModule();
      FRModule->StopSwerveModule();
      BLModule->StopSwerveModule();
      BRModule->StopSwerveModule();

      return;
    }

    // Determine wheel speeds / wheel target positions
    // Equations explained at:
    // https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
    // After clicking above link press the top download to see how the equations work
    double driveRadius = sqrt(pow(DRIVE_LENGTH, 2) + pow(DRIVE_WIDTH, 2));

    double A = STRAFE_Drive_Speed - Turn_Speed * (DRIVE_LENGTH / driveRadius);
    double B = STRAFE_Drive_Speed + Turn_Speed * (DRIVE_LENGTH / driveRadius);
    double C = FWD_Drive_Speed - Turn_Speed * (DRIVE_WIDTH / driveRadius);
    double D = FWD_Drive_Speed + Turn_Speed * (DRIVE_WIDTH / driveRadius);

    double FR_Target_Angle = atan2(B, C) * 180 / M_PI;
    double FL_Target_Angle = atan2(B, D) * 180 / M_PI;
    double BL_Target_Angle = atan2(A, D) * 180 / M_PI;
    double BR_Target_Angle = atan2(A, C) * 180 / M_PI;

    double FR_Drive_Speed = sqrt(pow(B, 2) + pow(C, 2));
    double FL_Drive_Speed = sqrt(pow(B, 2) + pow(D, 2));
    double BL_Drive_Speed = sqrt(pow(A, 2) + pow(D, 2));
    double BR_Drive_Speed = sqrt(pow(A, 2) + pow(C, 2));

    // If Turn Speed and Drive Speed are both high, the equations above will output a number greater than 1.
    // Below we must scale down all of the drive speeds to make sure we do not tell the motor to go faster
    // than it's max value.

    double max = FR_Drive_Speed;
    if (FL_Drive_Speed > max)
      max = FL_Drive_Speed;
    if (BL_Drive_Speed > max)
      max = BL_Drive_Speed;
    if (BR_Drive_Speed > max)
      max = BR_Drive_Speed;

    if (max > 1)
    {
      FL_Drive_Speed /= max;
      FR_Drive_Speed /= max;
      BL_Drive_Speed /= max;
      BR_Drive_Speed /= max;
    }

    // Make all the motors move
    FLModule->DriveSwerveModulePercent(FL_Drive_Speed, FL_Target_Angle);
    FRModule->DriveSwerveModulePercent(FR_Drive_Speed, FR_Target_Angle);
    BLModule->DriveSwerveModulePercent(BL_Drive_Speed, BL_Target_Angle);
    BRModule->DriveSwerveModulePercent(BR_Drive_Speed, BR_Target_Angle);
  }

  void DriveSwerveMetersAndRadians(double STRAFE_Drive_Speed, double FWD_Drive_Speed, double Turn_Speed)
  {
    DriveSwervePercent(STRAFE_Drive_Speed / SWERVE_DRIVE_MAX_MPS, FWD_Drive_Speed / SWERVE_DRIVE_MAX_MPS, Turn_Speed / MAX_RADIAN_PER_SECOND);
  }

  void DriveToPose(Pose2d current, Pose2d target, double elapsedTime)
  {
    double intendedVelocity;

    double xDistance = target.X().value() - current.X().value();
    if (fabs(xDistance) < ALLOWABLE_ERROR_TRANSLATION)
      xDistance = 0;
    intendedVelocity = std::clamp(TRANSLATION_KP * xDistance, -1 * TRANSLATION_MAX_SPEED, TRANSLATION_MAX_SPEED);
    lastX += std::clamp(intendedVelocity - lastX, -1 * TRANSLATION_MAX_ACCEL * elapsedTime,
                   TRANSLATION_MAX_ACCEL * elapsedTime);

    double yDistance = target.Y().value() - current.Y().value();
    if (fabs(yDistance) < ALLOWABLE_ERROR_TRANSLATION)
      yDistance = 0;
    intendedVelocity = std::clamp(TRANSLATION_KP * yDistance, -1 * TRANSLATION_MAX_SPEED, TRANSLATION_MAX_SPEED);
    lastY += std::clamp(intendedVelocity - lastY, -1 * TRANSLATION_MAX_ACCEL * elapsedTime,
                   TRANSLATION_MAX_ACCEL * elapsedTime);

    double thetaDistance = target.RelativeTo(current).Rotation().Radians().value();
    if (thetaDistance > 180)
      thetaDistance = thetaDistance - 360;
    if (fabs(thetaDistance) < ALLOWABLE_ERROR_ROTATION)
      thetaDistance = 0;
    intendedVelocity = std::clamp(SPIN_KP * thetaDistance, -1 * SPIN_MAX_SPEED, SPIN_MAX_SPEED);
    lastSpin += std::clamp(intendedVelocity - lastSpin, -1 * SPIN_MAX_ACCEL * elapsedTime,
                   SPIN_MAX_ACCEL * elapsedTime);

    SmartDashboard::PutNumber("XDistance", xDistance);
    SmartDashboard::PutNumber("YDistance", yDistance);
    SmartDashboard::PutNumber("ThetaDistance", thetaDistance);

    SmartDashboard::PutNumber("Drive X", lastX);
    SmartDashboard::PutNumber("Drive Y", lastY);
    SmartDashboard::PutNumber("Drive Spin", lastSpin);

    DriveSwervePercent(lastX, lastY, lastSpin);
  }

  void DriveToPoseOdometry(Pose2d target, double elapsedTime)
  {
    DriveToPose(GetPoseOdometry(), target, elapsedTime);
  }

  void DriveToPoseVision(Pose2d target, double elapsedTime)
  {
    DriveToPose(visionPose, target, elapsedTime);
  }

  void DriveToPoseCombo(Pose2d target, double elapsedTime)
  {
    timeSinceOdometryRefresh += elapsedTime;
    if (ODOMETRY_REFRESH_TIME < timeSinceOdometryRefresh)
    {
      timeSinceOdometryRefresh = 0;
      ResetOdometry(visionPose);
    }

    DriveToPoseOdometry(target, elapsedTime);
  }

  void TurnToPointWhileDriving(double strafeSpeed, double fwdSpeed, Translation2d point, double elapsedTime)
  {
    Translation2d diff = point - GetPose().Translation();
    Rotation2d targetAngle = Rotation2d(units::radian_t{atan2(diff.X().value(), diff.Y().value())});
    double thetaDistance = (targetAngle - GetPose().Rotation()).Radians().value();
    if (thetaDistance > 180)
      thetaDistance = thetaDistance - 360;
    if (fabs(thetaDistance) < ALLOWABLE_ERROR_ROTATION)
      thetaDistance = 0;
    double intendedVelocity = std::clamp(SPIN_KP * thetaDistance, -1 * SPIN_MAX_SPEED, SPIN_MAX_SPEED);
    lastSpin += std::clamp(intendedVelocity - lastSpin, -1 * SPIN_MAX_ACCEL * elapsedTime,
                   SPIN_MAX_ACCEL * elapsedTime);
    DriveSwervePercent(strafeSpeed, fwdSpeed, lastSpin);
  }

  void GenerateTrajecotory(vector<Translation2d> waypoints, Pose2d goal)
  {
    //unfinished
    TrajectoryConfig trajectoryConfig{units::meters_per_second_t{SWERVE_DRIVE_MAX_MPS}, units::meters_per_second_squared_t{SWERVE_DRIVE_MAX_ACCELERATION}};
    trajectoryConfig.SetKinematics(kinematics);
    TrajectoryGenerator trajectoryGenerator{};

    //May delete itself and break everything
    currentTrajectory = trajectoryGenerator.GenerateTrajectory(
      this->GetPose(),
      waypoints,
      goal,
      trajectoryConfig
    );
  }
};
