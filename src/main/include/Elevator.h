#include "Robot.h"

//Holding Elevator at Position PID
#define ELEVHOLDFF 0
#define ELEVKP 0.15
#define ELEVKI 0
#define ELEVKIMAX 0.1
#define ALLOWABLE_ERROR_ELEV 0.3
#define ELEVMAX_SPEED 0.9
#define ELEVMAX_ACCELERATION 8

class Elevator
{
    private:
        double runningIntegral = 0;
        double lastSpeed = 0;
        double lastHeight = 0;

    public:
        rev::CANSparkMax *winchL, *winchR;
        rev::SparkMaxRelativeEncoder *winchEncoder;
        TimeOfFlight *tofSensor;

        Elevator(rev::CANSparkMax *winchL_, rev::CANSparkMax *winchR_, TimeOfFlight *tofSensor_);

        double winchEncoderReading();
        void ResetElevatorEncoder();
        double TOFSReading();
        double TOFSElevatorHeight();
        void MoveElevatorPercent(double percent);
        void StopElevator();
        void StartPIDLoop();
        bool SetElevatorHeightPID(double height, double elapsedTime);
};
