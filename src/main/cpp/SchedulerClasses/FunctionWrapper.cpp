// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <string>
#include <functional>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/FunctionalCommand.h>
#include "SystemManager.cpp"
#include <frc/smartdashboard/SmartDashboard.h>


class FunctionWrapper : public frc2::CommandHelper<frc2::CommandBase, FunctionWrapper>
{
    private:
    bool isFinished;
    std::function<bool()> function;
    //frc2::FunctionalCommand backingCommand;

    public:

    FunctionWrapper(std::function<bool()> function, int requirementID)
    :isFinished{false}, function{function}
    {
      AddRequirements(SystemManager::GetInstance().GetSystem(requirementID));
    }

    // FunctionWrapper(std::function<bool()> function, int requirementID)
    // :isFinished{false}, function{function}, backingCommand {
    //     frc2::FunctionalCommand(
    //     std::function<void()>([this](){Initialize();}), 
    //     std::function<void()>([this](){Execute();}), 
    //     std::function<void(bool)>([this](bool a){End(a);}), 
    //     std::function<bool()>([this](){return IsFinished();}))
    // }
    // {
    //     backingCommand.AddRequirements(SystemManager::GetInstance().GetSystem(requirementID));
    // }

    // void Schedule()
    // {
    //   backingCommand.Schedule();
    // }

    // void AddRequirement(frc2::Subsystem* requirement)
    // {
    //   frc2::CommandBase::AddRequirements(requirement);
    // }

private:
    void Initialize()
    {
      frc::SmartDashboard::PutBoolean("Initialized",   true);
    }

    void Execute()
    {
      frc::SmartDashboard::PutBoolean("Executed",   true);
      isFinished = function();
    }

    void End(bool interrupted)
    {
      frc::SmartDashboard::PutBoolean("Ended",   true);
    }

     bool IsFinished()
    {
     frc::SmartDashboard::PutBoolean("Finished",   true);
     return isFinished;
    }
};
