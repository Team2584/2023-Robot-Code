#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>

class Claw
{
private:

public:
    rev::CANSparkMax *wristMotor;

  /**
   * Instantiates a two motor elevator lift
   */
  Claw(rev::CANSparkMax *wrist)
  {
    wristMotor = wrist;
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
   * Moves the elevator at a percent speed, positive being upwards
   */
  void MoveWristPercent(double percent)
  {
    wristMotor->Set(percent);
  }
};