#include "Elevator.h"

/**
 * Instantiates a two motor elevator lift
 */
Elevator::Elevator(rev::CANSparkMax *winchL_, rev::CANSparkMax *winchR_, TimeOfFlight *tofSensor_)
{
  winchL = winchL_;
  winchR = winchR_;
  winchR->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  winchL->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  winchEncoder = new rev::SparkMaxRelativeEncoder(winchL->GetEncoder());
  winchEncoder->SetPosition(0.0);
  tofSensor = tofSensor_;
}

double Elevator::winchEncoderReading()
{
  return winchEncoder->GetPosition();
}

void Elevator::ResetElevatorEncoder()
{
  winchEncoder->SetPosition(0.0);
}


double Elevator::TOFSReading()
{
  return (tofSensor->GetRange()) / 100;
}

double Elevator::TOFSElevatorHeight()
{
  if (!tofSensor->IsRangeValid())
      return lastHeight;
  return ((tofSensor->GetRange() - 30) * 0.70710678 + 260) / 1000;
}

/**
 * Moves the elevator at a percent speed, positive being upwards
 */
void Elevator::MoveElevatorPercent(double percent)
{
  winchR->Set(percent);
  winchL->Set(percent);   
}


void Elevator::StopElevator()
{
  MoveElevatorPercent(0);
}

void Elevator::StartPIDLoop()
{
  runningIntegral = 0;
  lastSpeed = 0;
}

bool Elevator::SetElevatorHeightPID(double height, double elapsedTime)
{
  double error = height - winchEncoderReading();

    if (fabs(error) < ALLOWABLE_ERROR_ELEV)
  {
    runningIntegral = 0;
    MoveElevatorPercent(0);
    return true;
  }

  // calculate our I in PID and clamp it between our maximum I effects
  double intendedI = std::clamp(ELEVKI * runningIntegral, -1 * ELEVKIMAX, ELEVKIMAX);

  // Clamp our intended velocity to our maximum and minimum velocity to prevent the robot from going too fast
  double intendedVelocity = std::clamp(ELEVKP * error + intendedI, -1 * ELEVMAX_SPEED, ELEVMAX_SPEED);

  // Make sure our change in velocity from the last loop is not going above our maximum acceleration
  lastSpeed += std::clamp(intendedVelocity - lastSpeed, -1 * ELEVMAX_ACCELERATION * elapsedTime,
                      ELEVMAX_ACCELERATION * elapsedTime);

  runningIntegral += error;

  MoveElevatorPercent(lastSpeed + ELEVHOLDFF);
  return false;
}