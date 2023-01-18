void Swerve::driveToCharger(Pose2d current){
    if (onCharger)
    {
        if (balenced)
        {
            strafe();
        }
        else
        {
            pidDriveTo(current.Rotation());
        }
    }
    else
    {
        if (current.Y() > goalYpos)
        {
            DriveToPose(current, spotAboveGoal, double elapsedTime, double translationMaxSpeed, double translationMaxAccel, double allowableErrorTranslation, double translationP, double translationI, double translationIMaxEffect, double rotationMaxSpeed, double rotationMaxAccel, double allowableErrorRotation, double rotationP, double rotationI, double rotationIMaxEffect, double useWeirdMinSpeedThing);
        }
        else
        {
            DriveToPose(current, spotBelowGoal, double elapsedTime, double translationMaxSpeed, double translationMaxAccel, double allowableErrorTranslation, double translationP, double translationI, double translationIMaxEffect, double rotationMaxSpeed, double rotationMaxAccel, double allowableErrorRotation, double rotationP, double rotationI, double rotationIMaxEffect, double useWeirdMinSpeedThing);
        }
    }
}