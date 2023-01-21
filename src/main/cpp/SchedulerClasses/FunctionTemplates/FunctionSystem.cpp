// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.


// #include <functional>

// template <typename SystemType>
// class FunctionSystem : public frc2::SubsystemBase
// {
//     private:
//     SystemType* systemPointer;
//     std::function<void()> periodicFunction;

//     void Periodic()
//     {
//         periodicFunction();
//     }

//     public:
//     FunctionSystem(SystemType system)
//     {
//         systemPointer = system*;
//         periodicFunction = []() {return;};
//     }

// //This method is called once every scheduler iteration (usually, once every 20 ms). 
// //It is typically used for telemetry and other periodic actions that do not interfere with whatever Function is requiring the System.
//     void SetPeriodicFunction(std::function<void()> newPeriodicFunction)
//     {
//         periodicFunction = newPeriodicFunction;
//     }

//     //There should be a SubsystemBase.SetDefault command to set the default function that runs if this system is not being used.

//     bool operator == (System const &system) //make sure this works
//     {
//         return systemPointer == system.systemPointer;
//     }

//     bool operator != (System const &system) //make sure this works
//     {
//         return systemPointer != system.systemPointer;
//     }
// };