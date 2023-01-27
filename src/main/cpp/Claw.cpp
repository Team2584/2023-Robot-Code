#include "Robot.h"
#include "ClawConstants.h"

#include <frc/smartdashboard/SmartDashboard.h>

class Claw
{
private:
    double runningClawIntegral = 0;
    double lastClawSpeed = 0;

public:
    rev::CANSparkMax *wristMotor;
    rev::CANSparkMax *clawMotor;
    rev::SparkMaxRelativeEncoder *clawEncoder;

  /**
   * Instantiates a two motor elevator lift
   */
  Claw(rev::CANSparkMax *wrist, rev::CANSparkMax *claw)
  {
    wristMotor = wrist;
    clawMotor = claw;
    clawEncoder =  new rev::SparkMaxRelativeEncoder(clawMotor->GetEncoder());
    clawEncoder->SetPosition(0.0);
  }

  void StopWristCoast()
  {
    wristMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    MoveWristPercent(0);
  }

  void StopWristBrake()
  {
    wristMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    MoveWristPercent(0);
  }

  /**
   * Moves the wrist at a percent speed, positive being upwards
   */
  void MoveWristPercent(double percent)
  {
    wristMotor->Set(percent);
  }


  double ClawEncoderReading()
  {
    return clawEncoder->GetPosition();
  }

  void MoveClawPercent(double percent)
  {
    clawMotor->Set(percent);
  }

  void PIDClaw(double point, double elapsedTime)
  {
    double error = point - ClawEncoderReading();

     if (fabs(error) < ALLOWABLE_ERROR_CLAW)
    {
      error = 0;
    }

    // calculate our I in PID and clamp it between our maximum I effects
    double intendedI = std::clamp(KI * runningClawIntegral, -1 * KIMAX, KIMAX);

    // Clamp our intended velocity to our maximum and minimum velocity to prevent the robot from going too fast
    double intendedVelocity = std::clamp(KP * error + intendedI, -1 * MAX_SPEED, MAX_SPEED);

    // Make sure our change in velocity from the last loop is not going above our maximum acceleration
    lastClawSpeed += std::clamp(intendedVelocity - lastClawSpeed, -1 * MAX_ACCELERATION * elapsedTime,
                        MAX_ACCELERATION * elapsedTime);

    SmartDashboard::PutNumber("error", error);
    MoveClawPercent(lastClawSpeed);
  }

  void OpenClaw(double elapsedTime)
  {
    PIDClaw(0, elapsedTime);
  }

};