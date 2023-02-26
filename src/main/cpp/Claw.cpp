#include "Robot.h"
#include "ClawConstants.h"

#include <frc/smartdashboard/SmartDashboard.h>

class Claw
{
private:
    double runningClawIntegral = 0;
    double lastClawSpeed = 0;
    double runningWristIntegral = 0;
    double lastWristSpeed = 0;
    double initalClawPIDTime = 0;
    
public:
    rev::CANSparkMax *wristMotor;
    rev::CANSparkMax *clawMotor;
    rev::SparkMaxRelativeEncoder *wristEncoder, *clawEncoder; 
    rev::SparkMaxAnalogSensor *distanceSensor;
    rev::SparkMaxAbsoluteEncoder *magEncoder;

  /**
   * Instantiates a two motor elevator lift
   */
  Claw(rev::CANSparkMax *wrist, rev::CANSparkMax *claw)
  {
    wristMotor = wrist;
    clawMotor = claw;
    wristMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    clawMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    clawEncoder =  new rev::SparkMaxRelativeEncoder(clawMotor->GetEncoder());
    clawEncoder->SetPosition(1.0);
    wristEncoder =  new rev::SparkMaxRelativeEncoder(wristMotor->GetEncoder());
    wristEncoder->SetPosition(0.0);
    distanceSensor = new rev::SparkMaxAnalogSensor(clawMotor->GetAnalog());

    magEncoder = new rev::SparkMaxAbsoluteEncoder(wristMotor->GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle));
  }

  double WristEncoderReading()
  {
    return wristEncoder->GetPosition();
  }

  double MagEncoderReading()
  {
    double reading = magEncoder->GetPosition();
    if (reading > 0.5)
      reading -= 1;
    return reading * 2 * M_PI;
  }

  /**
   * Moves the wrist at a percent speed, positive being upwards
   */
  void MoveWristPercent(double percent)
  {
    wristMotor->Set(-percent);
  }

  bool PIDWrist(double point, double elapsedTime)
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

    SmartDashboard::PutNumber("intended Velocity", intendedVelocity);
    SmartDashboard::PutNumber("intended I", intendedI);
    SmartDashboard::PutNumber("final speed", lastWristSpeed);

    if (lastWristSpeed < -0.2)
      lastWristSpeed = -0.2;
    MoveWristPercent(lastWristSpeed + WRISTFF);
    return false;
  }

  bool PIDWristDown(double elapsedTime)
  {
    return PIDWrist(2.15, elapsedTime);
  }

  bool PIDWristUp(double elapsedTime)
  {
    return PIDWrist(0.2, elapsedTime);
  }

  double ClawEncoderReading()
  {
    return clawEncoder->GetPosition();
  }

  void MoveClawPercent(double percent)
  {
   SmartDashboard::PutNumber("claw current", clawMotor->GetOutputCurrent());
   SmartDashboard::PutNumber("claw speed", percent);
   SmartDashboard::PutNumber("Distance Sensor", distanceSensor->GetPosition());
    clawMotor->Set(percent);
  }

  void BeginClawPID()
  {
    lastClawSpeed = 0;
    runningClawIntegral = 0;
    initalClawPIDTime = 0;
  }

  bool PIDClaw(double point, double elapsedTime)
  {
    double error = point - ClawEncoderReading();

     if (fabs(error) < ALLOWABLE_ERROR_CLAW)
    {
      MoveClawPercent(0);
      runningClawIntegral = 0;
      return true;
    }

    // calculate our I in PID and clamp it   between our maximum I effects
    double intendedI = std::clamp(CLAWKI * runningClawIntegral, -1 * CLAWKIMAX, CLAWKIMAX);
    if (fabs(error) < 4)
      runningClawIntegral += error;

    // Clamp our intended velocity to our maximum and minimum velocity to prevent the robot from going too fast
    double intendedVelocity = std::clamp(CLAWKP * error + intendedI, -1 * CLAWMAX_SPEED, CLAWMAX_SPEED);

    // Make sure our change in velocity from the last loop is not going above our maximum acceleration
    lastClawSpeed += std::clamp(intendedVelocity - lastClawSpeed, -1 * CLAWMAX_ACCELERATION * elapsedTime,
                        CLAWMAX_ACCELERATION * elapsedTime);

    SmartDashboard::PutNumber("claw I", intendedI);

    MoveClawPercent(lastClawSpeed);
    return false;
  }

  void ResetClawEncoder()
  {
    clawEncoder->SetPosition(0.0);
  }

  bool OpenClaw(double elapsedTime)
  {
    return PIDClaw(12, elapsedTime);
  }

  bool CloseClaw(double elapsedTime)
  {
    SmartDashboard::PutNumber("clawpidtime", initalClawPIDTime);
    initalClawPIDTime += elapsedTime;
    //Grab til it stops or we hit limit switch
    MoveClawPercent(-0.8);// change to just closes once we have working limit switches
    if (ClawEncoderReading() <= 0 || initalClawPIDTime > 0.5)
    {
      MoveClawPercent(0);
      return true;
    }
    return false;
  }

  bool ConeInClaw()
  { //0: 2.23, 
    double expectedDistance = -0.135 * ClawEncoderReading() + 2.85;
    return distanceSensor->GetPosition() > expectedDistance || distanceSensor->GetPosition() > 2.87;
  }

};