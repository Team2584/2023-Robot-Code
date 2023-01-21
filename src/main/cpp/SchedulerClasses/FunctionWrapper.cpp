// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <string>
#include <functional>

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

    void Initialize() override
    {

    }

    void Execute() override
    {
      isFinished = function();
    }

    void End(bool interrupted) override
    {

    }

     bool IsFinished() override
    {
     return isFinished;
    }
};
