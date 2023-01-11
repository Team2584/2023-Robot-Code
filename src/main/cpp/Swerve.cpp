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
  double encoderOffset;       /* Offset in magnetic encoder from 0 facing the front of the robot */
  double driveEncoderInitial; /* Used to computer the change in encoder tics, aka motor rotation */
  double spinEncoderInitialHeading;
  double spinEncoderInitialValue;

public:
  /**
   * Constructor for a Swerve Module: One of the four 2-motor systems in a swerve drive.
   *
   * @param driveMotor_ A pointer to a Talon FX Drive Motor (the one that makes the robot move)
   * @param spinMotor_ A pointer to a Talon FX Spin Motor (the one that makes the wheel rotate to change direction)
   * @param magEncoder_ A pointer to the absolute encoder used to track rotation of the swerve module wheels
   */
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

  /**
   * Finds the absolute heading of the swerve drive wheel relative to the robot.
   *
   * @return The Swerve Drive wheel's heading in radians with 0.0 being the front of the robot increasing clockwise.
   */
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

  /**
   * Resets the drive and spin encoders to 0, usually unessecesary as odometry works through change in encoder readings.
   */
  void ResetEncoders()
  {
    driveEncoderInitial = driveMotor->GetSelectedSensorPosition();
    spinEncoderInitialHeading = GetMagEncoderReading();
    spinEncoderInitialValue = -1 * spinMotor->GetSelectedSensorPosition();
  }

  /**
   *  Converts Talon Drive Encoder Reading to Meters
   */
  double GetDriveEncoderMeters()
  {
    return (driveMotor->GetSelectedSensorPosition() - driveEncoderInitial) / 2048 / DRIVE_MOTOR_GEAR_RATIO * DRIVE_MOTOR_CIRCUMFERENCE;
  }

  /**
   *  Converts Talon Drive Encoder Reading to Meters per Second
   */
  double GetDriveVelocity()
  {
    return driveMotor->GetSelectedSensorVelocity() / 2048 / 6.54 * 0.10322 * M_PI * 10;
  }

  /**
   *  INCOMPLETE DO NOT USE UNDER ANY CIRCUMSTANCE, USE GetMagEncoderReading() INSTEAD!
   */
  double GetSpinEncoderRadians()
  {
    // TODO
    double rotation = ((-1 * spinMotor->GetSelectedSensorPosition() - spinEncoderInitialValue) / 2048 / SPIN_MOTOR_GEAR_RATIO * 2 * M_PI) - spinEncoderInitialHeading;
    return fmod(rotation, 2 * M_PI);
  }

  /**
   *  Setss all motor speeds to 0.
   */
  void StopSwerveModule()
  {
    spinMotor->Set(ControlMode::PercentOutput, 0);
    driveMotor->Set(ControlMode::PercentOutput, 0);
  }

  /**
   *  Returns the current velocity and rotation of the swerve module, in a WPI struct
   */
  SwerveModuleState GetSwerveModuleState()
  {
    SwerveModuleState state = SwerveModuleState();
    state.speed = units::meters_per_second_t{fabs(GetDriveVelocity())};
    state.angle = Rotation2d(units::radian_t{GetMagEncoderReading()});
    return state;
  }

  /**
   *  Returns the current position and rotation of the swerve module, in a WPI struct
   */
  SwerveModulePosition GetSwerveModulePosition()
  {
    SwerveModulePosition state = SwerveModulePosition();
    state.distance = units::length::centimeter_t{GetDriveEncoderMeters() * 100}; // It works, don't judge
    state.angle = Rotation2d(units::radian_t{GetMagEncoderReading()});
    return state;
  }

  /**
   * Drives the Swerve Module at a certaint speed in a certain direction using a simple P feedback loop.
   * Utitlizes half-baked logic to decide the most efficient way for the swerve drive to spin the wheel.
   * Ask Avrick for a more detailed explanation of how this function works, it came to him in a dream.
   *
   * @param driveSpeed The desired velocity of the wheel in percent of maximum (0 - 1.0)
   * @param targetAngle The desired angle of the wheel in Radians
   */
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

  /**
   * Drives the Swerve Module at a certaint speed in a certain direction using a simple P feedback loop.
   *
   * @param driveSpeed The desired velocity of the wheel in meters per second
   * @param targetAngle The desired angle of the wheel in Radians
   */
  void DriveSwerveModuleMeters(double driveSpeed, double targetRadian)
  {
    DriveSwerveModulePercent(driveSpeed / SWERVE_DRIVE_MAX_MPS, targetRadian);
  }
};

class SwerveDrive
{
private:
  Pigeon2 *pigeonIMU;
  Translation2d m_frontLeft; /* Location of the front left wheel in relation to the center of the robot */
  Translation2d m_frontRight;
  Translation2d m_backLeft;
  Translation2d m_backRight;
  SwerveDriveKinematics<4> kinematics;
  SwerveDriveOdometry<4> *odometry;
  SwerveDriveOdometry<4> *visionOdometry; /* WIP for testing, probably not neccesary */
  pathplanner::PathPlannerTrajectory trajectory; 

  // All Variables below are "WIP" for testing and will hopefully be refactored later
  Pose2d visionPose;
  double timeSinceOdometryRefresh;
  bool seeTag = false;

  double lastX = 0; /* The last speed in the X direction */
  double lastY = 0;
  double lastSpin = 0;

  double runningIntegralX = 0; /* The running tally of error in the X direction, aka the Integral used for pId */
  double runningIntegralY = 0;
  double runningIntegralSpin = 0;

public:
  SwerveModule *FLModule, *FRModule, *BRModule, *BLModule;
  double pigeon_initial;

  /**
   * Instantiates a swerve drive in a stupid way with too many instance variables that aren't really imporant.
   * Just know there are 8 motors and 4 mag encoders on a Swerve Drive
   * There is also a Pigeon IMU which includes an accelerometer and gyroscope.
   */
  SwerveDrive(ctre::phoenix::motorcontrol::can::TalonFX *_FLDriveMotor,
              ctre::phoenix::motorcontrol::can::TalonFX *_FLSpinMotor, frc::DutyCycleEncoder *_FLMagEncoder,
              double _FLEncoderOffset, ctre::phoenix::motorcontrol::can::TalonFX *_FRDriveMotor,
              ctre::phoenix::motorcontrol::can::TalonFX *_FRSpinMotor, frc::DutyCycleEncoder *_FRMagEncoder,
              double _FREncoderOffset, ctre::phoenix::motorcontrol::can::TalonFX *_BRDriveMotor,
              ctre::phoenix::motorcontrol::can::TalonFX *_BRSpinMotor, frc::DutyCycleEncoder *_BRMagEncoder,
              double _BREncoderOffset, ctre::phoenix::motorcontrol::can::TalonFX *_BLDriveMotor,
              ctre::phoenix::motorcontrol::can::TalonFX *_BLSpinMotor, frc::DutyCycleEncoder *_BLMagEncoder,
              double _BLEncoderOffset, Pigeon2 *_pigeonIMU, double robotStartingRadian)
      // TODO CLEAN based off of drive width / length variables rather than hard coded
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

    // Instantiates WPI's swerve odometry class so they can do math for me
    odometry = new SwerveDriveOdometry<4>(kinematics,
                                          Rotation2d(units::radian_t{GetIMURadians()}),
                                          positions,
                                          frc::Pose2d(0_m, 0_m, Rotation2d(units::radian_t{robotStartingRadian})));

    // WIP, not actually neccesary
    visionOdometry = new SwerveDriveOdometry<4>(kinematics,
                                                Rotation2d(units::radian_t{GetIMURadians()}),
                                                positions,
                                                frc::Pose2d(0_m, 0_m, Rotation2d(units::radian_t{robotStartingRadian})));
  }

  /**
   * Converts a meters per second speed to a percent power argument for the falcon motors.
   */
  double VelocityToPercent(double velocity)
  {
    if (velocity > 0)
      return std::max((velocity + 0.193) / 4.44, 0.0);
    else 
      return std::min((velocity + 0.193) / 4.44, 0.0);  }

  /**
   * Converts a percent power argument for the falcon motors to a meters per second speed.
   */
  double PercentToVelocity(double percent)
  {
    if (percent > 0)
      return std::max(4.44 * percent - 0.193, 0.0);
    else 
      return std::min(4.44 * percent + 0.193, 0.0);
  }

  /**
   * Returns the absolute heading of the swerve drive according the the IMU (gyroscope).
   *
   * @return The Swerve Drive's heading in radians with 0.0 being the front of the robot increasing clockwise.
   */
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

  /**
   * Resets Odometry to (0,0) facing away from the driver
   */
  void ResetOdometry()
  {
    ResetOdometry(Pose2d(0_m, 0_m, Rotation2d(0_rad)));
  }

  /**
   * Resets Odometry to a position
   */
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

    // WIP, disregard
    visionOdometry->ResetPosition(
        Rotation2d(units::radian_t{GetIMURadians()}),
        positions,
        frc::Pose2d(Pose2d(position.Y(), position.X(), position.Rotation())));
  }

  /**
   * Updates the odometry reading based on change in each swerve module's positions.
   * Must be called every periodic loop for accuracy (once every 20ms or less)
   */
  void UpdateOdometry()
  {
    wpi::array<SwerveModulePosition, 4> positions = {FLModule->GetSwerveModulePosition(),
                                                     FRModule->GetSwerveModulePosition(),
                                                     BLModule->GetSwerveModulePosition(),
                                                     BRModule->GetSwerveModulePosition()};
    odometry->Update(units::radian_t{GetIMURadians()}, positions);
    visionOdometry->Update(units::radian_t{GetIMURadians()}, positions);
  }

  /**
   * Finds the Pose of the robot. Currently just uses odometry, but will incorporate vision in the future.
   */
  Pose2d GetPose()
  {
    return GetPoseOdometry();
  }

  /**
   * Finds the Pose of the robot according to odometry.
   */
  Pose2d GetPoseOdometry()
  {
    Pose2d pose = odometry->GetPose();
    return Pose2d(pose.Y(), pose.X(), pose.Rotation());
  }

  /**
   * WIP Please Disregard
   */
  Pose2d GetPoseVisionOdometry()
  {
    Pose2d pose = visionOdometry->GetPose();

    if (seeTag)
      return Pose2d(pose.Y(), pose.X(), pose.Rotation());
    else
      return GetPoseOdometry();
  }

  /**
   * Sets the pose of vision using jetson data. Must be done every update loop.
   *
   * @param pose current position of the robot as estimated by the jetson
   * @param SeeTag can the cameras see any april tags, aka is our vision data up to date
   */
  void SetPoseVision(Pose2d pose, bool SeeTag)
  {
    seeTag = SeeTag;

    if (!SeeTag)
      return;

    visionPose = pose;

    wpi::array<SwerveModulePosition, 4> positions = {FLModule->GetSwerveModulePosition(),
                                                     FRModule->GetSwerveModulePosition(),
                                                     BLModule->GetSwerveModulePosition(),
                                                     BRModule->GetSwerveModulePosition()};

    visionOdometry->ResetPosition(
        Rotation2d(units::radian_t{GetIMURadians()}),
        positions,
        frc::Pose2d(Pose2d(pose.Y(), pose.X(), pose.Rotation())));
  }

  /**
   * Finds the Pose of the robot. Currently just uses odometry, but will incorporate vision in the future.
   */
  Pose2d GetPoseVision()
  {
    return visionPose;
  }

  /**
   * Sets the swerve module states to their respective values
   */
  void SetModuleStates(std::array<SwerveModuleState, 4> states)
  {
    FRModule->DriveSwerveModuleMeters(states[0].speed.value(), states[0].angle.Degrees().value());
    FLModule->DriveSwerveModuleMeters(states[1].speed.value(), states[1].angle.Degrees().value());
    BRModule->DriveSwerveModuleMeters(states[2].speed.value(), states[2].angle.Degrees().value());
    BLModule->DriveSwerveModuleMeters(states[3].speed.value(), states[3].angle.Degrees().value());
  }

  /**
   * Drives the swerve drive, field oriented (in relation to the driver's pov) with an x y and spin.
   *
   * @param STRAFE_Drive_Speed The speed the robot should move left and right, positive being right, in percentage (0 - 1.0)
   * @param FWD_Drive_Speed The speed the robot should move forward and back, positive being forward, in percentage (0 - 1.0)
   * @param Turn_Speed The speed the robot should turn left and right, positive being clockwise, in percentage (0 - 1.0)
   */
  void DriveSwervePercent(double STRAFE_Drive_Speed, double FWD_Drive_Speed, double Turn_Speed)
  {
    // Converts our field oriented speeds to robot oriented, by using trig with the current robot angle.
    double angle = GetPose().Rotation().Radians().value();
    double oldFwd = FWD_Drive_Speed;
    FWD_Drive_Speed = FWD_Drive_Speed * cos(angle) + STRAFE_Drive_Speed * sin(angle);
    STRAFE_Drive_Speed = -1 * oldFwd * sin(angle) + STRAFE_Drive_Speed * cos(angle);

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

  /**
   * Drives the swerve drive, field oriented (in relation to the driver's pov) with an x y and spin.
   *
   * @param STRAFE_Drive_Speed The speed the robot should move left and right, positive being right, in meters per second
   * @param FWD_Drive_Speed The speed the robot should move forward and back, positive being forward, in meters per second
   * @param Turn_Speed The speed the robot should turn left and right, positive being clockwise, in radians per second
   */
  void DriveSwerveMetersAndRadians(double STRAFE_Drive_Speed, double FWD_Drive_Speed, double Turn_Speed)
  {
    DriveSwervePercent(VelocityToPercent(STRAFE_Drive_Speed), VelocityToPercent(FWD_Drive_Speed), Turn_Speed / MAX_RADIAN_PER_SECOND);
  }

  /**
   * Resets all errors and PID things to 0 to get ready for the PID loop in drive to pose
   */
  void BeginPIDLoop()
  {
    lastX = 0;
    lastY = 0;
    lastSpin = 0;
    runningIntegralX = 0;
  }

  /**
   * Drives the robot to a pose using a feedback PID loop
   *
   * @param current the robot's current Pose
   * @param target the target Pose
   * @param elapsedTime the time since our last iteration of the pid loop
   * @param translationMaxSpeed the max speed we want our robot to go as a percent of the robot's physical maximum velocity
   * @param translationMaxAccel the max acceleration we want our robot to go as a percent per second
   * @param allowableErrorTranslation the acceptable error for translation in meters (we stop the function once the robot is within this range)
   * @param translationP the P constant for translation PID
   * @param translationI the I constant for transaltion PID
   * @param translationIMaxEffect the maximum effect our I constant can have on the system to prevent overshooting
   * @param rotationMaxSpeed the max speed we want our robot to spin as a percent of the robot's physical maximum rotation speed
   * @param rotationMaxAccel the max acceleration we want our robot to spin as a percent per second
   * @param allowableErrorRotation the acceptable error for rotation in radians (we stop the function once the robot is within this range)
   * @param rotationP the P constant for spin PID
   * @param rotationI the I constant for spin PID
   * @param rotationIMaxEffect the maximum effect our I constant can have on the system to prevent overshooting
   */
  void DriveToPose(Pose2d current, Pose2d target, double elapsedTime,
                   double translationMaxSpeed, double translationMaxAccel, double allowableErrorTranslation,
                   double translationP, double translationI, double translationIMaxEffect,
                   double rotationMaxSpeed, double rotationMaxAccel, double allowableErrorRotation,
                   double rotationP, double rotationI, double rotationIMaxEffect, double useWeirdMinSpeedThing)
  {
    double intendedVelocity;
    double intendedI;

    double xSpeed;
    // Our current error in the x direction
    double xDistance = target.X().value() - current.X().value();

    // If we are within our allowable error, stop moving in the x direction
    if (fabs(xDistance) < allowableErrorTranslation)
    {
      xDistance = 0;
      runningIntegralX = 0;
    }

    // calculate our I in PID and clamp it between our maximum I effects
    intendedI = std::clamp(translationI * runningIntegralX, -1 * translationIMaxEffect, translationIMaxEffect);

    // Clamp our intended velocity to our maximum and minimum velocity to prevent the robot from going too fast
    intendedVelocity = std::clamp(translationP * xDistance + intendedI, -1 * translationMaxSpeed, translationMaxSpeed);

    // Make sure our change in velocity from the last loop is not going above our maximum acceleration
    lastX += std::clamp(intendedVelocity - lastX, -1 * translationMaxAccel * elapsedTime,
                        translationMaxAccel * elapsedTime);
    xSpeed = lastX;

    // This is a WIP that shouldn't be needed for the final robot with well tuned PID
    if (lastX > 0 && lastX < 0.06 && useWeirdMinSpeedThing)
      xSpeed = 0.06;
    else if (lastX < 0 && lastX > -0.06 && useWeirdMinSpeedThing)
      xSpeed = -0.06;

    // Repeat with the Y direction
    double ySpeed;
    double yDistance = target.Y().value() - current.Y().value();
    if (fabs(yDistance) < allowableErrorTranslation)
    {
      yDistance = 0;
      runningIntegralY = 0;
    }
    intendedI = std::clamp(translationI * runningIntegralY, -1 * translationIMaxEffect, translationIMaxEffect);
    intendedVelocity = std::clamp(translationP * yDistance + intendedI, -1 * translationMaxSpeed, translationMaxSpeed);
    lastY += std::clamp(intendedVelocity - lastY, -1 * translationMaxAccel * elapsedTime,
                        translationMaxAccel * elapsedTime);
    ySpeed = lastY;
    if (lastY > 0 && lastY < 0.06 && useWeirdMinSpeedThing)
      ySpeed = 0.06;
    else if (lastY < 0 && lastY > -0.06 && useWeirdMinSpeedThing)
      ySpeed = -0.06;

    // Repeat for spinning
    double spinSpeed;
    double thetaDistance = target.RelativeTo(current).Rotation().Radians().value();
    if (thetaDistance > 180)
      thetaDistance = thetaDistance - 360;
    if (fabs(thetaDistance) < allowableErrorRotation)
    {
      thetaDistance = 0;
      runningIntegralSpin = 0;
    }
    intendedI = std::clamp(rotationI * runningIntegralSpin, -1 * rotationIMaxEffect, rotationIMaxEffect);
    intendedVelocity = std::clamp(rotationP * thetaDistance + intendedI, -1 * rotationMaxSpeed, rotationMaxSpeed);
    lastSpin += std::clamp(intendedVelocity - lastSpin, -1 * rotationMaxAccel * elapsedTime,
                           rotationMaxAccel * elapsedTime);
    spinSpeed = lastSpin;
    if (lastSpin > 0 && lastSpin < 0.06 && useWeirdMinSpeedThing)
      spinSpeed = 0.06;
    else if (lastSpin < 0 && lastSpin > -0.06 && useWeirdMinSpeedThing)
      spinSpeed = -0.06;

    // Add to our running integral error count
    runningIntegralX += xDistance;
    runningIntegralY += yDistance;
    runningIntegralSpin += thetaDistance;

    // Debugging info
    SmartDashboard::PutNumber("XDistance", xDistance);
    SmartDashboard::PutNumber("YDistance", yDistance);
    SmartDashboard::PutNumber("ThetaDistance", thetaDistance);

    SmartDashboard::PutNumber("Drive X", lastX);
    SmartDashboard::PutNumber("Drive Y", lastY);
    SmartDashboard::PutNumber("Drive Spin", lastSpin);

    // Drive swerve at desired speeds
    DriveSwervePercent(xSpeed, ySpeed, spinSpeed);
  }

  /**
   * Determines our desired spin speed to rotate to face a point
   * Can be used in driver control to rotate to a point while manually driving the robot
   */
  double TurnToPointDesiredSpin(Pose2d current, Translation2d point, double elapsedTime, double allowableErrorRotation, double spinMaxSpeed, double spinMaxAccel, double spinP, double spinI)
  {
    // Determine what our target angle is
    Translation2d diff = point - current.Translation();
    Rotation2d targetAngle = Rotation2d(units::radian_t{atan2(diff.X().value(), diff.Y().value())});

    // Use PID similar to the above function to determine our desired spin speed
    double thetaDistance = (targetAngle - current.Rotation()).Radians().value();
    if (thetaDistance > 180)
      thetaDistance = thetaDistance - 360;
    if (fabs(thetaDistance) < allowableErrorRotation)
      thetaDistance = 0;
    double intendedVelocity = std::clamp(spinP * thetaDistance, -1 * spinMaxSpeed, spinMaxSpeed);
    lastSpin += std::clamp(intendedVelocity - lastSpin, -1 * spinMaxAccel * elapsedTime,
                           spinMaxAccel * elapsedTime);
    return lastSpin;
  }

  /**
   * Combination of drive to pose and turn to point to drive to a pose while continuously facing a point.
   * This lets your camera stay pointed at an april tag.
   * WIP Code that is likely uneeded
   */
  void DriveToPoseWhileFacingTag(Pose2d current, Pose2d target, Pose2d tag, double elapsedTime,
                                 double translationMaxSpeed, double translationMaxAccel, double allowableErrorTranslation,
                                 double translationP, double translationI, double translationIMaxEffect,
                                 double rotationMaxSpeed, double rotationMaxAccel, double allowableErrorRotation,
                                 double rotationP, double rotationI, double rotationIMaxEffect)
  {
    double intendedVelocity;
    double intendedI;

    double xDistance = target.X().value() - current.X().value();
    if (fabs(xDistance) < allowableErrorTranslation)
    {
      xDistance = 0;
      runningIntegralX = 0;
    }
    intendedI = std::clamp(translationI * runningIntegralX, -1 * translationIMaxEffect, translationIMaxEffect);
    intendedVelocity = std::clamp(translationP * xDistance + intendedI, -1 * translationMaxSpeed, translationMaxSpeed);
    lastX += std::clamp(intendedVelocity - lastX, -1 * translationMaxAccel * elapsedTime,
                        translationMaxAccel * elapsedTime);
    SmartDashboard::PutNumber("X Integral", runningIntegralX);
    SmartDashboard::PutNumber("X Integral Output", intendedI);

    double yDistance = target.Y().value() - current.Y().value();
    if (fabs(yDistance) < allowableErrorTranslation)
    {
      yDistance = 0;
      runningIntegralY = 0;
    }
    intendedI = std::clamp(translationI * runningIntegralY, -1 * translationIMaxEffect, translationIMaxEffect);
    intendedVelocity = std::clamp(translationP * yDistance + intendedI, -1 * translationMaxSpeed, translationMaxSpeed);
    lastY += std::clamp(intendedVelocity - lastY, -1 * translationMaxAccel * elapsedTime,
                        translationMaxAccel * elapsedTime);

    double spin = TurnToPointDesiredSpin(current, tag.Translation(), elapsedTime, allowableErrorRotation, rotationMaxSpeed, rotationMaxAccel, rotationP, rotationI);

    DriveSwervePercent(lastX, lastY, spin);
  }

  // Functions below use DriveToPose() with different inputs and PID constants for testing

  void DriveToPoseOdometry(Pose2d target, double elapsedTime)
  {
    DriveToPose(GetPoseOdometry(), target, elapsedTime, O_TRANSLATION_MAX_SPEED, O_TRANSLATION_MAX_ACCEL, O_ALLOWABLE_ERROR_TRANSLATION,
                O_TRANSLATION_KP, O_TRANSLATION_KI, O_TRANSLATION_KI_MAX, O_SPIN_MAX_SPEED, O_SPIN_MAX_ACCEL, O_ALLOWABLE_ERROR_ROTATION,
                O_SPIN_KP, O_SPIN_KI, O_SPIN_KI_MAX, false);
  }

  void DriveToPoseVision(Pose2d target, double elapsedTime)
  {
    DriveToPose(visionPose, target, elapsedTime, V_TRANSLATION_MAX_SPEED, V_TRANSLATION_MAX_ACCEL, V_ALLOWABLE_ERROR_TRANSLATION,
                V_TRANSLATION_KP, V_TRANSLATION_KI, V_TRANSLATION_KI_MAX, V_SPIN_MAX_SPEED, V_SPIN_MAX_ACCEL, V_ALLOWABLE_ERROR_ROTATION,
                V_SPIN_KP, V_SPIN_KI, V_SPIN_KI_MAX, true);
  }

  void DriveToPoseVisionOdometry(Pose2d target, double elapsedTime)
  {
    DriveToPose(GetPoseVisionOdometry(), target, elapsedTime, V_TRANSLATION_MAX_SPEED, V_TRANSLATION_MAX_ACCEL, V_ALLOWABLE_ERROR_TRANSLATION,
                V_TRANSLATION_KP, V_TRANSLATION_KI, V_TRANSLATION_KI_MAX, V_SPIN_MAX_SPEED, V_SPIN_MAX_ACCEL, V_ALLOWABLE_ERROR_ROTATION,
                V_SPIN_KP, V_SPIN_KI, V_SPIN_KI_MAX, true);
  }

  void DriveToPoseWhileFacingTagVision(Pose2d target, Pose2d tag, double elapsedTime)
  {
    DriveToPoseWhileFacingTag(visionPose, target, tag, elapsedTime, V_TRANSLATION_MAX_SPEED, V_TRANSLATION_MAX_ACCEL, V_ALLOWABLE_ERROR_TRANSLATION,
                              V_TRANSLATION_KP, V_TRANSLATION_KI, V_TRANSLATION_KI_MAX, V_SPIN_MAX_SPEED, V_SPIN_MAX_ACCEL, V_ALLOWABLE_ERROR_ROTATION,
                              V_SPIN_KP, V_SPIN_KI, V_SPIN_KI_MAX);
  }

  /*
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
  */

  /**
   * Initializes a trajectory to be run during autonomous by loading it into memory.
   * A trajectory is a curve that we tell the robot to move through. AKA a spline.
   * Run in auton Init
   */
  void InitializeTrajectory()
  {
  // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    trajectory = pathplanner::PathPlanner::loadPath("Pathfinder Curve", pathplanner::PathConstraints(4_mps, 3_mps_sq));
  }

  /**
   * Follow a trajectory through auton.
   * Must be called every autonomous loop.
   */
  void FollowTrajectory(units::second_t time, double elapsedTime)
  {
    UpdateOdometry();

    // Sample the state of the path at some seconds
    pathplanner::PathPlannerTrajectory::PathPlannerState state = trajectory.sample(time);
    auto xFF = state.velocity * state.pose.Rotation().Cos() * 1.2;
    auto yFF = state.velocity * state.pose.Rotation().Sin() * 1.2;
    double thetaFF = state.angularVelocity.value();

    SmartDashboard::PutNumber("Program Time", time.value());
    SmartDashboard::PutNumber("trajectory total time", trajectory.getTotalTime().value());

    SmartDashboard::PutNumber("X Mps", xFF.value());
    SmartDashboard::PutNumber("Y Mps", yFF.value());

    // Run simple PID to correct our robots course
    Translation2d pose = GetPose().Translation();
    Translation2d goal = state.pose.Translation();
    double xDistance = (-1 * goal.Y() - pose.X()).value();
    double yDistance = (goal.X() - pose.Y()).value();
    double xPid = std::clamp(S_TRANSLATION_KP * xDistance, -1 * S_TRANSLATION_MAX_SPEED, S_TRANSLATION_MAX_SPEED);
    double yPid = std::clamp(S_TRANSLATION_KP * yDistance, -1 * S_TRANSLATION_MAX_SPEED, S_TRANSLATION_MAX_SPEED);

    // Spin PID similar to drive to pose
    double thetaDistance = Pose2d(0_m, 0_m, Rotation2d(state.holonomicRotation)).RelativeTo(GetPose()).Rotation().Radians().value();
    if (fabs(thetaDistance) < S_ALLOWABLE_ERROR_ROTATION)
    {
      thetaDistance = 0;
      runningIntegralSpin = 0;
    }
    double intendedI = std::clamp(S_SPIN_KI * runningIntegralSpin, -1 * S_SPIN_KI_MAX, S_SPIN_KI_MAX);
    double spinPid = std::clamp(S_SPIN_KP * thetaDistance + intendedI, -1 * S_SPIN_MAX_SPEED, S_SPIN_MAX_SPEED);

    // Debugging Info
    SmartDashboard::PutNumber("X Odom", pose.X().value());
    SmartDashboard::PutNumber("Y Odom", pose.Y().value());

    SmartDashboard::PutNumber("x Dist", xDistance);
    SmartDashboard::PutNumber("y Dist", yDistance);

    SmartDashboard::PutNumber("x Pid", xPid);
    SmartDashboard::PutNumber("y Pid", yPid);

    SmartDashboard::PutNumber("Theta Distance", thetaDistance);
    SmartDashboard::PutNumber("spin Pid", spinPid);

    // If we have finished the spline, just stop
    if (trajectory.getTotalTime() < time && xDistance < S_ALLOWABLE_ERROR_TRANSLATION && yDistance < S_ALLOWABLE_ERROR_TRANSLATION && lastSpin == 0)
    {
      DriveSwervePercent(0, 0, 0);
      return;
    }

    // Drive the swerve drive
    DriveSwerveMetersAndRadians(xFF.value() + xPid, yFF.value() + yPid, thetaFF + spinPid);
  }
};
