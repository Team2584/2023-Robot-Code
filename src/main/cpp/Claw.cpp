#include "Robot.h"
#include "ClawConstants.h"
#include "PicoColorSensor.h"

#include <frc/smartdashboard/SmartDashboard.h>

class Claw
{
private:
    double runningClawIntegral = 0;
    double lastClawSpeed = 0;
    double runningWristIntegral = 0;
    double lastWristSpeed = 0;
    double initalClawPIDTime = 0;
    bool usingBeamBreaks = true;

public:
    rev::CANSparkMax *wristMotor;
    rev::CANSparkMax clawMotor;
    rev::SparkMaxRelativeEncoder *wristEncoder, *clawEncoder; 
    frc::DigitalInput *regualarBreambreak;
    frc::DigitalInput *substationBreambreak;
    rev::SparkMaxAbsoluteEncoder *magEncoder;
    rev::SparkMaxLimitSwitch closedLimit, openLimit;
    pico::ColorSensor *colorSensor;

  /**
   * Instantiates a two motor elevator lift
   */
  Claw(rev::CANSparkMax *wrist)
  :clawMotor{9, rev::CANSparkMax::MotorType::kBrushless},
  closedLimit{clawMotor.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen)},
  openLimit{clawMotor.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen)}
  {
    wristMotor = wrist;
    wristMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    clawMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    clawEncoder = new rev::SparkMaxRelativeEncoder(clawMotor.GetEncoder());
    clawEncoder->SetPosition(1.0);
    wristEncoder =  new rev::SparkMaxRelativeEncoder(wristMotor->GetEncoder());
    wristEncoder->SetPosition(0.0);
    colorSensor = new pico::ColorSensor();
    substationBreambreak = new frc::DigitalInput(0);
    regualarBreambreak = new frc::DigitalInput(1);
   // closedLimit = new rev::SparkMaxLimitSwitch(clawMotor->GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed));
   // openLimit = new rev::SparkMaxLimitSwitch(clawMotor->GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed));
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
    return PIDWrist(1.95, elapsedTime);
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
    SmartDashboard::PutBoolean("closed Limit", closedLimit.Get());
    SmartDashboard::PutBoolean("open Limit", openLimit.Get());
    SmartDashboard::PutNumber("digital frequency", regualarBreambreak->Get());

    if (closedLimit.Get())
      ResetClawEncoder(0);
    else if (openLimit.Get())
      ResetClawEncoder(13);
    clawMotor.Set(percent);
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

  void ResetClawEncoder(double val)
  {
    clawEncoder->SetPosition(val);
  }

  bool OpenClaw(double elapsedTime)
  {
    return PIDClaw(12.5, elapsedTime);
  }

  bool CloseClaw(double elapsedTime)
  {
    SmartDashboard::PutNumber("clawpidtime", initalClawPIDTime);
    initalClawPIDTime += elapsedTime;
    //Grab til it stops or we hit limit switch
    MoveClawPercent(-0.9);
    if (ClawEncoderReading() <= 0.25 || initalClawPIDTime > 0.75)
    {
      MoveClawPercent(0);
      return true;
    }
    return false;
  }

  bool GetUsingBeamBreaks()
  {
    return usingBeamBreaks;
  }

  void SetUsingBeamBreaks(bool shouldUse)
  {
    usingBeamBreaks = shouldUse;
  }


  bool ObjectInClaw()
  { 
    if (!usingBeamBreaks)
      return false;
    
    return regualarBreambreak->Get();
  }

  bool ObjectInClawSubstation()
  {
    if (!usingBeamBreaks)
      return false;

    return substationBreambreak->Get();
  }

  bool IsObjectCone()
  {
    pico::ColorSensor::RawColor color = colorSensor->GetRawColor0();
    double r = static_cast<double>(color.red);
    double g = static_cast<double>(color.green);
    double b = static_cast<double>(color.blue);
    double mag = r + g + b;
    r /= mag;
    g /= mag;
    b /= mag;
    r *= 255;
    g *= 255;
    b *= 255;
    SmartDashboard::PutNumber("color sensor blue", b * 255);
      SmartDashboard::PutNumber("color sensor red", r * 255);
    SmartDashboard::PutNumber("color sensor green", g * 255);
    if (ClawEncoderReading() > 7)
      return false;
    else if (ClawEncoderReading() < 3)
      return true;
    else if (ObjectInClaw())
      return !(g > 30 && b < 100);
    else
      return ClawEncoderReading() < 4;
  }

};