// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/SequentialCommandGroup.h>
#include "FunctionWrapper.cpp"
#include "SystemManager.cpp"


class SequentialProgram : public frc2::CommandHelper<frc2::SequentialCommandGroup, SequentialProgram>
{
    private:

    public:
    SequentialProgram()
    {

    }

    void AddFunction(std::function<bool()> function, int requirementID)
    {
        FunctionWrapper newCommand = FunctionWrapper(function);
        newCommand.AddRequirement(SystemManager::GetInstance().GetSystem(requirementID));
        AddCommands(FunctionWrapper(function));
    }
};