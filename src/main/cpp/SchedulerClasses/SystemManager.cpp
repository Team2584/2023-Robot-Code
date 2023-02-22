// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <unordered_map>
#include "SystemWrapper.cpp"

enum Systems
{
    ChassisSystem,
    ClawSystem,
};

class SystemManager
{
    private:
    std::unordered_map<int, SystemWrapper> systemMap;
    SystemManager()
    {
        systemMap = std::unordered_map<int, SystemWrapper>(); //Map from systemID to system
        systemMap.insert(std::pair<int, SystemWrapper>(0, SystemWrapper()));
        systemMap.insert(std::pair<int, SystemWrapper>(1, SystemWrapper()));
    }

    public:
    SystemWrapper* System = new SystemWrapper();

    static SystemManager& GetInstance()
    {
        static SystemManager manager;
        return manager;
    }

    void Add(int systemID)
    {
        systemMap.insert(std::pair<int, SystemWrapper>(systemID, SystemWrapper()));
        frc2::CommandScheduler::GetInstance().RegisterSubsystem(GetSystem(systemID));
    }

    SystemWrapper* GetSystem(int systemID)
    {
        return &systemMap.at(systemID);
    }

    void AddPeriodic(/*takes in an ID and function*/)
    {

    }
};
