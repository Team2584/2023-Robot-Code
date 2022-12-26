#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

class SwerveModule
{
private:
  // Instance Variables for each swerve module
  ctre::phoenix::motorcontrol::can::TalonFX *driveMotor, *spinMotor;
  frc::DutyCycleEncoder *magEncoder;
  double encoderOffset;

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
  }

  //Stops all motor velocity in swerve module
  void stopSwerveModule()
  {
    spinMotor->Set(ControlMode::PercentOutput, 0);
    driveMotor->Set(ControlMode::PercentOutput, 0);
  }

  // Spin swerve module motors to reach the drive speed and spin angle
  void driveSwerveModule(double driveSpeed, double targetAngle, double kp)
  {
    // current encoder reading as an angle
    double wheelAngle = GetEncoderReadingAsAngle();
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
    double output = kp * (error / 90);

    // Move motors at speeds and directions determined earlier
    spinMotor->Set(ControlMode::PercentOutput, output * spinDirection);
    driveMotor->Set(ControlMode::PercentOutput, driveSpeed * driveDirection);
  }

  // Converts Mag-Encoder Reading to an angle from 0-360
  double GetEncoderReadingAsAngle()
  {
    double encoderReading = magEncoder->GetAbsolutePosition();
    // subtract the encoder offset to make 0 degrees forward
    encoderReading -= encoderOffset;
    if (encoderReading < 0)
      encoderReading += 1;
    // Flip the degrees to make clockwise positive
    encoderReading = 1 - encoderReading;
    // Convert from 0-1 to degrees
    encoderReading *= 360;
    return encoderReading;
  }
};

class SwerveDrive
{
private:
  SwerveModule *FLModule, *FRModule, *BRModule, *BLModule;
  Pigeon2 *pigeonIMU;
  double driveLength, driveWidth;

public:
  // spin_kp is the p value used in the pid loop which points the wheel in the correct direction (TODO Clean up)
  double spin_kp = 0.6;

  // Instantiates SwerveDrive class by creating 4 swerve modules
  SwerveDrive(ctre::phoenix::motorcontrol::can::TalonFX *_FLDriveMotor,
              ctre::phoenix::motorcontrol::can::TalonFX *_FLSpinMotor, frc::DutyCycleEncoder *_FLMagEncoder,
              double _FLEncoderOffset, ctre::phoenix::motorcontrol::can::TalonFX *_FRDriveMotor,
              ctre::phoenix::motorcontrol::can::TalonFX *_FRSpinMotor, frc::DutyCycleEncoder *_FRMagEncoder,
              double _FREncoderOffset, ctre::phoenix::motorcontrol::can::TalonFX *_BRDriveMotor,
              ctre::phoenix::motorcontrol::can::TalonFX *_BRSpinMotor, frc::DutyCycleEncoder *_BRMagEncoder,
              double _BREncoderOffset, ctre::phoenix::motorcontrol::can::TalonFX *_BLDriveMotor,
              ctre::phoenix::motorcontrol::can::TalonFX *_BLSpinMotor, frc::DutyCycleEncoder *_BLMagEncoder,
              double _BLEncoderOffset, Pigeon2 *_pigeonIMU, double _driveLength,
              double _driveWidth)
  {
    FLModule = new SwerveModule(_FLDriveMotor, _FLSpinMotor, _FLMagEncoder, _FLEncoderOffset);
    FRModule = new SwerveModule(_FRDriveMotor, _FRSpinMotor, _FRMagEncoder, _FREncoderOffset);
    BLModule = new SwerveModule(_BLDriveMotor, _BLSpinMotor, _BLMagEncoder, _BLEncoderOffset);
    BRModule = new SwerveModule(_BRDriveMotor, _BRSpinMotor, _BRMagEncoder, _BREncoderOffset);
    pigeonIMU = _pigeonIMU;
    driveLength = _driveLength;
    driveWidth = _driveWidth;
  }

  void moveSwerveDrive(double FWD_Drive_Speed, double STRAFE_Drive_Speed, double Turn_Speed)
  {
    // If there is no drive input, don't drive the robot and just end the function
    if (FWD_Drive_Speed == 0 && STRAFE_Drive_Speed == 0 && Turn_Speed == 0)
    {
      FLModule->stopSwerveModule();
      FRModule->stopSwerveModule();
      BLModule->stopSwerveModule();
      BRModule->stopSwerveModule();

      return;
    }

    // Determine wheel speeds / wheel target positions
    // Equations explained at:
    // https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
    // After clicking above link press the top download to see how the equations work
    double driveRadius = sqrt(pow(driveLength, 2) + pow(driveWidth, 2));

    double A = STRAFE_Drive_Speed - Turn_Speed * (driveLength / driveRadius);
    double B = STRAFE_Drive_Speed + Turn_Speed * (driveLength / driveRadius);
    double C = FWD_Drive_Speed - Turn_Speed * (driveLength / driveRadius);
    double D = FWD_Drive_Speed + Turn_Speed * (driveLength / driveRadius);

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

    frc::SmartDashboard::PutNumber("FR Drive Speed", FR_Drive_Speed);
    frc::SmartDashboard::PutNumber("FR Target Angle", FR_Target_Angle);

    // Make all the motors move
    FLModule->driveSwerveModule(FL_Drive_Speed, FL_Target_Angle, spin_kp);
    FRModule->driveSwerveModule(FR_Drive_Speed, FR_Target_Angle, spin_kp);
    BLModule->driveSwerveModule(BL_Drive_Speed, BL_Target_Angle, spin_kp);
    BRModule->driveSwerveModule(BR_Drive_Speed, BR_Target_Angle, spin_kp);
  }
};
