// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// //#include <frc2/command/CommandBase.h>
// //#include <frc2/command/CommandHelper.h>
// #pragma once
// #include "FunctionSystem.cpp"
// #include <string>

// template<typename T1, typename T2, typename T3>
// class Function : public frc2::CommandHelper<frc2::CommandBase, Function>//Add subsystems for the commands to use
// {
//     private:
//     std::function<bool(T1 param1, T2 param2, T3 param3)> function;
//     T1 param1;
//     T2 param2;
//     T3 param3;
//     bool isFinished;

//     Function(std::function<bool(T1 param1, T2 param2, T3 param3)> function)
//     {
//         this->function = function;
//     }

//     void SetParameters(T1 parameter1, T2 parameter2, T3 parameter3)
//     {
//         param1 = parameter1;
//         param2 = parameter2;
//         param3 = parameter3;
//     }

//     void Initialize() override
//     {

//     }

//   void Execute() override
//   {
//     isFinished = function(param1, param2, param3);
//   }

//   void End(bool interrupted) override
//   {

//   }

//   bool IsFinished() override
//   {
//     return isFinished;
//   }
// };