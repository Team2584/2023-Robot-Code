void Swerve::driveToCharger(Pose2d current){
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
}

void Swerve::balence(Pose2d current){
      gyroRot = gyroScope.GetPitch();     //Pull pitch angle from gyroscope
    frc::SmartDashboard::PutNumber("gyroRot", gyroRot); //on the dashboard, output the gyroRot number
    float deadZone = 2.5;                           //deadzone angle
    float motorMaxSpeed = 0.02;                     //max speed of motor in %
    float motorVelocity;                            //final velocity of motor
    int direction = (gyroRot > 0) ? -1 : 1;         // if gyroRot is greater than 0, change direction to -1, vice versa. This is for correction, we want to move opposite direction from tilt

    if (abs(gyroRot) > deadZone)
        motorVelocity = direction * (abs(gyroRot) - deadZone) * motorMaxSpeed * 1 / 9; //when rotation of gyro exceeds the deadzone, set motor velocity (this is proportional to the gyro angle)
    if (motorVelocity > 0.4)
        motorVelocity = 0.4;
    DriveSwervePercent(0, motorVelocity, 0);
}