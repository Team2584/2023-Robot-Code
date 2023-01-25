#include "Robot.h"
#include "LimelightConstants.h"

class Limelight
{

private:
    double tx = 0.0; double txDefault = 0.0;
    double ty = 0.0; double tyDefault = 0.0;
    double ta = 0.0; double taDefault = 0.0;
    // nt::NetworkTableInstance::GetDefault().GetTable("limelight")
    std::shared_ptr<nt::NetworkTable> lemonTable;

public:
    Limelight(std::shared_ptr<nt::NetworkTable> limelightTable){
        // limelight table should be something like: nt::NetworkTableInstance::GetDefault().GetTable("limelight")
        // change the table name from "limelight" to the network table name your limelight is putting values in (default is "limelight")
        lemonTable = limelightTable;
    }

    void updateLimelightValues(){
    tx = lemonTable->GetNumber("tx", txDefault)/(320/2) * 5; // get number and normalise :: reports -1 to 1
    ty = lemonTable->GetNumber("ty", tyDefault)/(240/2); // get number and normalise :: reports -1 to 1
    ta = lemonTable->GetNumber("ta", taDefault)/100;     // get number and normalise :: reports 0 to 1
    }

    void changeTargetDefaults(double txD, double tyD, double taD){
        txD = txDefault;
        tyD = tyDefault;
        taD = taDefault;
    }

    double getTargetX(){
        updateLimelightValues();
        SmartDashboard::PutNumber("Limelight X", tx);
        return tx;
    }

    double getTargetY(){
        updateLimelightValues();
        return ty;
    }

    double getTargetArea(){
        updateLimelightValues();
        return ta;
    }

    bool TargetExists(){
        return ta != 0.0;
    }

    void limelightToSmartDashboard(){
        SmartDashboard::PutNumber("TargetX", getTargetX());
        SmartDashboard::PutNumber("TargetY", getTargetY());
        SmartDashboard::PutNumber("TargetArea", getTargetArea());
    }
};