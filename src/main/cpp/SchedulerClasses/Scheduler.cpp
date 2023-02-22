// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// #include <vector>
// #include <hash_set>
// #include <frc2/command/CommandScheduler.h>
// #include "SequentialProgram.cpp"

/*
SmartDashboard::PutBoolean("StageOneComplete",   false);
SmartDashboard::PutBoolean("StageTwoComplete",  false);

SmartDashboard::PutBoolean("Initialized",   false);
SmartDashboard::PutBoolean("Executed",   false);
SmartDashboard::PutBoolean("Ended",   false);
SmartDashboard::PutBoolean("Finished",   false);

CommandScheduler::GetInstance().Enable();

testFunction = new FunctionWrapper([](){SmartDashboard::PutBoolean("StageOneComplete", true); return true;}, Systems::Chassis);
testFunction->Schedule();

testProgram = new SequentialProgram();
testProgram->AddFunction([](){SmartDashboard::PutBoolean("StageOneComplete", true); return false;}, Systems::Chassis);
testProgram->AddFunction([](){SmartDashboard::PutBoolean("StageTwoComplete", true); return true;}, Systems::Chassis);
testProgram->Schedule();
*/

// class Scheduler
// {
//     private:
//     std::vector<std::function<void()>> periodicFunctions;

//     public:
//     Scheduler()
//     {
//         //frc2::CommandScheduler::GetInstance().OnCommandInterrupt([](const frc2::Command& command) {frc::SmartDashboard::PutBoolean(command.GetName() + " was interrupted.", true);});
//         periodicFunctions = std::vector<std::function<void()>>();
//     }

//     // void Schedule(FunctionWrapper* functionPointer, int systemRequirementID)
//     // {
//     //     //functionPointer->AddRequirement(system.GetSystem(systemRequirementID));
//     //     //functionPointer->AddRequirement(new SystemWrapper());
        
//     //     frc::SmartDashboard::PutBoolean(functionPointer->GetName() + " was interrupted.", false);
//     //     functionPointer->AddRequirements(system.System);
//     //     functionPointer->Schedule();
//     //     //frc2::CommandScheduler::GetInstance().Schedule(functionPointer);
//     // }
    
//     // void Schedule(frc2::SequentialCommandGroup program)
//     // {
//     //     program.Schedule();
//     // }

//     /*void ScheduleTogether(frc2::CommandHelper<frc2::CommandBase, Function> function1, frc2::CommandHelper<frc2::CommandBase, Function> function2)
//     {
//         function1.CommandBase::AddParallel
//         frc2::CommandBase:: parallelFunctions = frc2::ParallelCommandGroup();
//         parallelFunctions.AddCommand(function1);
//         parallelFunctions.AddCommand(function2);
//         frc2::CommandScheduler::Schedule(&parallelFunctions); //Is this still gonna be alive?
//     } */
    
//     void Run()
//     {
//         frc2::CommandScheduler::GetInstance().Run();
//     }
// };

//Make scheduler work! Needs initialazation and it is possible that I need more commands in the Function and FunctionSystem classes.
//Make the string required system work by adding mapping from string to subsystem! Otherwise commands won't work
//Add back default functions and maybe the periodic functions
//Check if the Command Scheduler needs to be initialized!
