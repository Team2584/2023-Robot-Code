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
    tofSensor = tofSensor_;
  }

  double TOFSElevatorHeight()
  {
    if (!tofSensor->IsRangeValid())
        return lastHeight;
    return ((tofSensor->GetRange() - 30) * 0.70710678 + 260) / 1000;
  }

  void StopElevatorCoast()
  {
    winchR->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    winchL->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    MoveElevatorPercent(0);
  }

  void StopElevatorBreak()
  {
    winchR->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    winchL->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    MoveElevatorPercent(0);
  }

  /**
   * Moves the elevator at a percent speed, positive being upwards
   */
  void MoveElevatorPercent(double percent)
  {
    winchR->Set(percent);
    winchL->Set(percent);   
  }

  void StartPIDLoop()
  {
    runningIntegral = 0;
    lastSpeed = 0;
  }

  void SetElevatorHeightPID(double height, double elapsedTime)
  {
    double error = height - TOFSElevatorHeight();

     if (fabs(error) < ALLOWABLE_ERROR_HEIGHT)
    {
      error = 0;
      runningIntegral = 0;
    }

    // calculate our I in PID and clamp it between our maximum I effects
    double intendedI = std::clamp(KI * runningIntegral, -1 * KIMAX, KIMAX);

    // Clamp our intended velocity to our maximum and minimum velocity to prevent the robot from going too fast
    double intendedVelocity = std::clamp(KP * error + intendedI, -1 * MAX_SPEED, MAX_SPEED);

    // Make sure our change in velocity from the last loop is not going above our maximum acceleration
    lastSpeed += std::clamp(intendedVelocity - lastSpeed, -1 * MAX_ACCELERATION * elapsedTime,
                        MAX_ACCELERATION * elapsedTime);


    MoveElevatorPercent(lastSpeed + HOLDFF);
  }
};