// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <string>
#include <functional>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "SystemManager.cpp"
#include <frc/smartdashboard/SmartDashboard.h>


class FunctionWrapper : public frc2::CommandHelper<frc2::CommandBase, FunctionWrapper>
{
    private:
    bool isFinished;
    std::function<bool()> function;

    public:

    FunctionWrapper(std::function<bool()> function)
    :isFinished{false}, function{function}
    {
    }

    FunctionWrapper(std::function<bool()> function, int requirementID)
    :isFinished{false}, function{std::move(function)}
    {
      AddRequirement(SystemManager::GetInstance().GetSystem(requirementID));
    }

    void AddRequirement(frc2::Subsystem* requirement)
    {
      frc2::CommandBase::AddRequirements(requirement);
    }

    void Initialize() override
    {
      frc::SmartDashboard::PutBoolean("Initialized",   true);
    }

    void Execute() override
    {
      frc::SmartDashboard::PutBoolean("Executed",   true);
      isFinished = function();
    }

    void End(bool interrupted) override
    {
      frc::SmartDashboard::PutBoolean("Ended",   true);
    }

     bool IsFinished() override
    {
     frc::SmartDashboard::PutBoolean("Finished",   true);
     return !isFinished;
    }
};
