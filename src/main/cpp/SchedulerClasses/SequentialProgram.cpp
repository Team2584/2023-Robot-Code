// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

#include "FunctionWrapper.cpp"
#include <frc2/command/SequentialCommandGroup.h>

class SequentialProgram
{
    private:
    std::vector<frc2::CommandPtr> commands;
    std::shared_ptr<frc2::SequentialCommandGroup> program;

    public:
    SequentialProgram()
    {

    }

    void AddFunction(std::function<bool()> function, int requirementID)
    {
        commands.push_back(std::move((FunctionWrapper(function, requirementID)).ToPtr()));
    }

    void Schedule()
    {
        program = std::make_shared<frc2::SequentialCommandGroup>(frc2::CommandPtr::UnwrapVector(std::move(commands)));
        //AddCommands(frc2::CommandPtr::UnwrapVector(std::move(commands)));
        program->Schedule();
    }
};