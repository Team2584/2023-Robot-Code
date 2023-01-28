#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/phoenix/sensors/WPI_Pigeon2.h>

ctre::phoenix::sensors::WPI_Pigeon2 gyroScope = ctre::phoenix::sensors::WPI_Pigeon2(0);
/*void Swerve::driveToCharger(Pose2d current){
    if (onCharger)
    {
        if (current.Rotation().X() == 0)
        {
            if (current.Rotation().Z() != 90)
            {
                balence(current);
            }
        }
        else
        {
            pidDriveTo(current.Rotation().X());
        }
    }
    else
    {
        if (current.Y() > goalYpos)
        {
            DriveToPose(current, 
            //TODO spotAboveGoal, 
            double elapsedTime, double translationMaxSpeed, double translationMaxAccel, double allowableErrorTranslation, double translationP, double translationI, double translationIMaxEffect, double rotationMaxSpeed, double rotationMaxAccel, double allowableErrorRotation, double rotationP, double rotationI, double rotationIMaxEffect, double useWeirdMinSpeedThing);
        }
        else
        {
            DriveToPose(current, 
            //TODO spotBelowGoal, 
            double elapsedTime, double translationMaxSpeed, double translationMaxAccel, double allowableErrorTranslation, double translationP, double translationI, double translationIMaxEffect, double rotationMaxSpeed, double rotationMaxAccel, double allowableErrorRotation, double rotationP, double rotationI, double rotationIMaxEffect, double useWeirdMinSpeedThing);
        }
    }
}*/

