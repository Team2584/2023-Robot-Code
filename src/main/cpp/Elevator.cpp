#include "ElevatorConstants.h"
#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>

class ElevatorLift
{
private:
    double runningIntegral = 0;
    double lastSpeed = 0;
    double lastHeight = 0;

public:
    rev::CANSparkMax *winchL, *winchR;
    rev::SparkMaxRelativeEncoder *winchEncoder;
    TimeOfFlight *tofSensor;

  /**
   * Instantiates a two motor elevator lift
   */
  ElevatorLift(rev::CANSparkMax *winchL_, rev::CANSparkMax *winchR_, TimeOfFlight *tofSensor_)
  {
    winchL = winchL_;
    winchR = winchR_;
    winchR->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    winchL->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    winchEncoder = new rev::SparkMaxRelativeEncoder(winchL->GetEncoder());
    winchEncoder->SetPosition(0.0);
    tofSensor = tofSensor_;
  }

  void ResetElevatorEncoder()
  {
    winchEncoder->SetPosition(0.0);
  }

  double winchEncoderReading()
  {
    return -winchEncoder->GetPosition();
  }

  double TOFSReading()
  {
    return (tofSensor->GetRange()) / 100;
  }

  double TOFSElevatorHeight()
  {
    if (!tofSensor->IsRangeValid())
        return lastHeight;
    return ((tofSensor->GetRange() - 30) * 0.70710678 + 260) / 1000;
  }

  void StopElevator()
  {
    MoveElevatorPercent(0);
  }

  /**
   * Moves the elevator at a percent speed, positive being upwards
   */
  void MoveElevatorPercent(double percent)
  {
    winchR->Set(percent);
    winchL->Set(-percent);   
  }

  void StartPIDLoop()
  {
    runningIntegral = 0;
    lastSpeed = 0;
  }

  bool SetElevatorHeightPID(double height, double elapsedTime)
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
};