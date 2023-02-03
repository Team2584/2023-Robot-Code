#include "Robot.h"

#define CLAWKP 0.14
#define CLAWKI 0
#define CLAWKIMAX 0.1
#define ALLOWABLE_ERROR_CLAW 0.5
#define CLAWMAX_SPEED 0.7
#define CLAWMAX_ACCELERATION 5

#define WRISTKP 1
#define WRISTKI 0
#define WRISTKIMAX 0.1
#define ALLOWABLE_ERROR_WRIST 0.2   
#define WRISTMAX_SPEED 1.0
#define WRISTMAX_ACCELERATION 4

class Claw
{
    private:
        double runningClawIntegral = 0;
        double lastClawSpeed = 0;
        double runningWristIntegral = 0;
        double lastWristSpeed = 0;
    
    public:
        rev::CANSparkMax *wristMotor;
        rev::CANSparkMax *clawMotor;
        rev::SparkMaxRelativeEncoder *clawEncoder, *wristEncoder;
        rev::SparkMaxAbsoluteEncoder *magEncoder;
      
        Claw(rev::CANSparkMax *wrist, rev::CANSparkMax *claw);

        double WristEncoderReading();
        double MagEncoderReading();
        void MoveWristPercent(double percent);
        bool PIDWrist(double point, double elapsedTime);

        double ClawEncoderReading();
        void ResetClawEncoder();
        void MoveClawPercent(double percent);
        bool PIDClaw(double point, double elapsedTime);
        bool OpenClaw(double elapsedTime);
};