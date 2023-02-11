// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// #include "FunctionWrapper.cpp"
// #include <frc2/command/SequentialCommandGroup.h>
// #include <frc2/command/InstantCommand.h>


// class SequentialProgram : public frc2::CommandHelper<frc2::SequentialCommandGroup, SequentialProgram>
// {
//     private:
//     std::vector<std::unique_ptr<Command>> commands;

//     public:
//     SequentialProgram()
//     {

//     }

//     void AddFunction(std::function<bool()> function, int requirementID)
//     {
//         commands.push_back(std::make_unique<Command>(FunctionWrapper(function), SystemManager::GetInstance().GetSystem(requirementID)));
//     }

//     void Start()
//     {
//         frc2::SequentialCommandGroup::AddCommands(std::move(commands));
//         Schedule();
//     }
// };