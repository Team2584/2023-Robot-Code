#include "Limelight.h"

Limelight::Limelight(std::shared_ptr<nt::NetworkTable> limelightTable)
{
    // limelight table should be something like: nt::NetworkTableInstance::GetDefault().GetTable("limelight")
    // change the table name from "limelight" to the network table name your limelight is putting values in (default is "limelight")
    lemonTable = limelightTable;
}

void Limelight::updateLimelightValues()
{
tx = lemonTable->GetNumber("tx", txDefault)/(320/2) * 5; // get number and normalise :: reports -1 to 1
ty = lemonTable->GetNumber("ty", tyDefault)/(240/2); // get number and normalise :: reports -1 to 1
ta = lemonTable->GetNumber("ta", taDefault)/100;     // get number and normalise :: reports 0 to 1
}

void Limelight::changeTargetDefaults(double txD, double tyD, double taD)
{
    txDefault = txD;
    tyDefault = tyD;
    taDefault = taD;
}

double Limelight::getTargetX()
{
    updateLimelightValues();
    SmartDashboard::PutNumber("Limelight X", tx);
    return tx;
}

double Limelight::getTargetY()
{
    updateLimelightValues();
    return ty;
}

double Limelight::getTargetArea()
{
    updateLimelightValues();
    return ta;
}

bool Limelight::TargetExists()
{
    return ta != 0.0;
}

void Limelight::limelightToSmartDashboard()
{
    SmartDashboard::PutNumber("TargetX", getTargetX());
    SmartDashboard::PutNumber("TargetY", getTargetY());
    SmartDashboard::PutNumber("TargetArea", getTargetArea());
}