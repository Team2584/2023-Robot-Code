#include "Robot.h"
#include "LimelightConstants.h"

class Limelight
{

private:
    double tx = 0.0; double txDefault = 0.0;
    double ty = 0.0; double tyDefault = 0.0;
    double ta = 0.0; double taDefault = 0.0;
    double lastX = 0.0; double lastY = 0.0;
    // nt::NetworkTableInstance::GetDefault().GetTable("limelight")
    std::shared_ptr<nt::NetworkTable> lemonTable;

public:
    Limelight(std::shared_ptr<nt::NetworkTable> limelightTable){
        // limelight table should be something like: nt::NetworkTableInstance::GetDefault().GetTable("limelight")
        // change the table name from "limelight" to the network table name your limelight is putting values in (default is "limelight")
        lemonTable = limelightTable;
    }

    void updateLimelightValues(){
        bool tv = lemonTable->GetNumber("tv", false);
        tx = lemonTable->GetNumber("tx", txDefault)/(320/2) * 5; // get number and normalise :: reports -1 to 1
        ty = (lemonTable->GetNumber("ty", tyDefault) - 24.85)/(240/2); // get number and normalise :: reports -1 to 1
        ta = lemonTable->GetNumber("ta", taDefault)/100;     // get number and normalise :: reports 0 to 1
        if (tv)
        {
            lastX = tx;
            lastY = ty;
        }
        else
        {
            tx = lastX;
            ty = lastY;
        }
    }

    void changeTargetDefaults(double txD, double tyD, double taD){
        txDefault = txD;
        tyDefault = tyD;
        taDefault = taD;
    }

    double getTargetX(){
        updateLimelightValues();
        SmartDashboard::PutNumber("Limelight X", tx);
        return tx;
    }

    double getTargetY(){
        updateLimelightValues();
        SmartDashboard::PutNumber("Limelight Y", ty);
        return ty;
    }

    double getTargetArea(){
        updateLimelightValues();
        return ta;
    }

    bool TargetExists(){
        return ta != 0.0;
    }

    void TurnOffLimelight()
    {
        lemonTable->PutNumber("ledMode", 1);
    }
    
    void TurnOnLimelight()
    {
        lemonTable->PutNumber("ledMode", 3);
    }

    void limelightToSmartDashboard(){
        SmartDashboard::PutNumber("TargetX", getTargetX());
        SmartDashboard::PutNumber("TargetY", getTargetY());
        SmartDashboard::PutNumber("TargetArea", getTargetArea());
    }
};