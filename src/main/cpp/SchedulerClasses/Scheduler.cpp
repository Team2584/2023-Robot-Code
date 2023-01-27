// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <vector>
#include <functional>
//#include "FunctionTemplates/Function.cpp"
#include <hash_set>
//#include <frc2/command/CommandBase.h>
//#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandScheduler.h>
#include "SystemManager.cpp"
#include "FunctionWrapper.cpp"

/* new
Scheduler scheduler = Scheduler();

Chassis chassis;
scheduler.AddRequirement(&chassis.MoveAtPower, chassis); Not working yet
Lift lift;
scheduler.AddRequirement(&life.MoveAtPowerForTime, lift); Not working yet

scheduler.Schedule(Function1<int>(&chassis.MoveAtPower, 50, "Chassis")); //string represents the requirements
scheduler.ScheduleTogether(Function1<int>(&chassis.MoveAtPower, 25, "Chassis"), Function2<int, int>(&lift.MoveListAtPowerForTime, 100, 10, "Lift")); //string represents the requirements
*/

class Scheduler
{
    private:
    std::vector<std::function<void()>> periodicFunctions;
    SystemManager system;

    public:
    Scheduler()
    {
        periodicFunctions = std::vector<std::function<void()>>();
        system = SystemManager();
    }

    void Schedule(FunctionWrapper* functionPointer, int systemRequirementID)
    {
        //functionPointer->AddRequirement(system.GetSystem(systemRequirementID));
        functionPointer->AddRequirement(system.System);
        frc2::CommandScheduler::GetInstance().Schedule(functionPointer);
    }

    /*void ScheduleTogether(frc2::CommandHelper<frc2::CommandBase, Function> function1, frc2::CommandHelper<frc2::CommandBase, Function> function2)
    {
        function1.CommandBase::AddParallel
        frc2::CommandBase:: parallelFunctions = frc2::ParallelCommandGroup();
        parallelFunctions.AddCommand(function1);
        parallelFunctions.AddCommand(function2);
        frc2::CommandScheduler::Schedule(&parallelFunctions); //Is this still gonna be alive?
    } */
    
    void Run()
    {
        frc2::CommandScheduler::GetInstance().Run();
    }
};

//Make scheduler work! Needs initialazation and it is possible that I need more commands in the Function and FunctionSystem classes.
//Make the string required system work by adding mapping from string to subsystem! Otherwise commands won't work
//Add back default functions and maybe the periodic functions
//Check if the Command Scheduler needs to be initialized!
