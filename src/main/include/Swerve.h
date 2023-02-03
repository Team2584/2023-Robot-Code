#include "SwerveConstants.h"
#include "Robot.h"

class SwerveModule
{
    private:
        // Instance Variables for each swerve module
        ctre::phoenix::motorcontrol::can::TalonFX *driveMotor;
        rev::CANSparkMax *spinMotor;
        rev::SparkMaxRelativeEncoder *spinEncoder;
        frc::DutyCycleEncoder *magEncoder;
        double encoderOffset;       /* Offset in magnetic encoder from 0 facing the front of the robot */
        double driveEncoderInitial; /* Used to computer the change in encoder tics, aka motor rotation */
        double spinEncoderInitialHeading;
        double spinEncoderInitialValue;
        double runningIntegral = 0; /* Running sum of errors for integral in PID */

    public:
        SwerveModule(ctre::phoenix::motorcontrol::can::TalonFX *driveMotor_,
                rev::CANSparkMax *spinMotor_, frc::DutyCycleEncoder *magEncoder_,
                double encoderOffset_);

        double GetMagEncoderReading();
        void ResetEncoders();
        double GetDriveEncoderMeters();
        double GetDriveVelocity();
        double GetSpinEncoderRadians();
        void StopSwerveModule();
        SwerveModuleState GetSwerveModuleState();
        SwerveModulePosition GetSwerveModulePosition();
        void DriveSwerveModulePercent(double driveSpeed, double targetAngle);
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
        SwerveDrivePoseEstimator<4> *odometry;
        std::queue<pathplanner::PathPlannerTrajectory> trajectoryList; 
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

        SwerveDrive(ctre::phoenix::motorcontrol::can::TalonFX *_FLDriveMotor,
                rev::CANSparkMax *_FLSpinMotor, frc::DutyCycleEncoder *_FLMagEncoder, ctre::phoenix::motorcontrol::can::TalonFX *_FRDriveMotor,
                rev::CANSparkMax *_FRSpinMotor, frc::DutyCycleEncoder *_FRMagEncoder, ctre::phoenix::motorcontrol::can::TalonFX *_BRDriveMotor,
                rev::CANSparkMax *_BRSpinMotor, frc::DutyCycleEncoder *_BRMagEncoder, ctre::phoenix::motorcontrol::can::TalonFX *_BLDriveMotor,
                rev::CANSparkMax *_BLSpinMotor, frc::DutyCycleEncoder *_BLMagEncoder, Pigeon2 *_pigeonIMU);

        double VelocityToPercent(double velocity);
        double PercentToVelocity(double percent);
        double AngularVelocityToPercent(double velocity);
        double AngularPercentToVelocity(double percent);
        double GetIMURadians();
        void ResetOdometry();
        void ResetOdometry(Pose2d position);
        void UpdateOdometry(units::second_t currentTime);
        void AddPositionEstimate(Translation2d poseEstimate, units::second_t timeOfEstimate);
        void AddPositionEstimate(Pose2d poseEstimate, units::second_t timeOfEstimate);
        Pose2d GetPose();
        void DriveSwervePercent(double STRAFE_Drive_Speed, double FWD_Drive_Speed, double Turn_Speed);
        void DriveSwerveMetersAndRadians(double STRAFE_Drive_Speed, double FWD_Drive_Speed, double Turn_Speed);
        void BeginPIDLoop();
        void DriveToPose(Pose2d target, double elapsedTime);
        bool DriveToPose(Pose2d target, double elapsedTime,
                  double translationMaxSpeed, double translationMaxAccel, double allowableErrorTranslation,
                  double translationP, double translationI, double translationIMaxEffect,
                  double rotationMaxSpeed, double rotationMaxAccel, double allowableErrorRotation,
                  double rotationP, double rotationI, double rotationIMaxEffect, double useWeirdMinSpeedThing);
        double TurnToPointDesiredSpin(Translation2d point, double elapsedTime, double allowableErrorRotation, double spinMaxSpeed, double spinMaxAccel, double spinP, double spinI);
        void ResetTrajectoryList();
        void InitializeTrajectory(string trajectoryString);
        void InitializeTrajectory(string trajectoryString, units::meters_per_second_t velocity, units::meters_per_second_squared_t acceleration);
        void SetNextTrajectory();
        bool FollowTrajectory(units::second_t time, double elapsedTime);
        bool TurnToPixel(double offset, double elapsedTime);
        bool StrafeToPole(double offset, double elapsedTime);
};