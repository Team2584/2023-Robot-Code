#include "Claw.h"

/**
 * Instantiates a two motor elevator lift
 */
Claw::Claw(rev::CANSparkMax *wrist, rev::CANSparkMax *claw)
{
  wristMotor = wrist;
  clawMotor = claw;
  wristMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  clawEncoder =  new rev::SparkMaxRelativeEncoder(clawMotor->GetEncoder());
  clawEncoder->SetPosition(1.0);
  wristEncoder =  new rev::SparkMaxRelativeEncoder(wristMotor->GetEncoder());
  wristEncoder->SetPosition(0.0);

  magEncoder = new rev::SparkMaxAbsoluteEncoder(wristMotor->GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle));
}

double Claw::WristEncoderReading()
{
  return wristEncoder->GetPosition();
}

double Claw::MagEncoderReading()
{
  double reading = magEncoder->GetPosition();
  if (reading > 0.5)
    reading -= 1;
  return reading * 2 * M_PI;
}

/**
 * Moves the wrist at a percent speed, positive being upwards
 */
void Claw::MoveWristPercent(double percent)
{
  wristMotor->Set(percent);
}

bool Claw::PIDWrist(double point, double elapsedTime)
{
  double error = MagEncoderReading() - point;

  SmartDashboard::PutNumber("error", error);

  if (fabs(error) < ALLOWABLE_ERROR_WRIST)
  {
    MoveWristPercent(0);
    return true;
  }

  // calculate our I in PID and clamp it between our maximum I effects
  double intendedI = std::clamp(WRISTKI * runningWristIntegral, -1 * WRISTKIMAX, WRISTKIMAX);

  // Clamp our intended velocity to our maximum and minimum velocity to prevent the robot from going too fast
  double intendedVelocity = std::clamp(WRISTKP * error + intendedI, -1 * WRISTMAX_SPEED, WRISTMAX_SPEED);

  // Make sure our change in velocity from the last loop is not going above our maximum acceleration
  lastWristSpeed += std::clamp(intendedVelocity - lastWristSpeed, -1 * WRISTMAX_ACCELERATION * elapsedTime,
                      WRISTMAX_ACCELERATION * elapsedTime);

  SmartDashboard::PutNumber("error", error);
  MoveWristPercent(lastWristSpeed);
  return false;
}

double Claw::ClawEncoderReading()
{
  return clawEncoder->GetPosition();
}

void Claw::ResetClawEncoder()
{
  clawEncoder->SetPosition(0.0);
}

void Claw::MoveClawPercent(double percent)
{
  clawMotor->Set(percent);
}

bool Claw::PIDClaw(double point, double elapsedTime)
{
  double error = point - ClawEncoderReading();

    if (fabs(error) < ALLOWABLE_ERROR_CLAW)
  {
    MoveClawPercent(0);
    return true;
  }

  // calculate our I in PID and clamp it between our maximum I effects
  double intendedI = std::clamp(CLAWKI * runningClawIntegral, -1 * CLAWKIMAX, CLAWKIMAX);

  // Clamp our intended velocity to our maximum and minimum velocity to prevent the robot from going too fast
  double intendedVelocity = std::clamp(CLAWKP * error + intendedI, -1 * CLAWMAX_SPEED, CLAWMAX_SPEED);

  // Make sure our change in velocity from the last loop is not going above our maximum acceleration
  lastClawSpeed += std::clamp(intendedVelocity - lastClawSpeed, -1 * CLAWMAX_ACCELERATION * elapsedTime,
                      CLAWMAX_ACCELERATION * elapsedTime);

  SmartDashboard::PutNumber("error", error);
  MoveClawPercent(lastClawSpeed);
  return false;
}

bool Claw::OpenClaw(double elapsedTime)
{
  return PIDClaw(-20, elapsedTime);
}
