// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <unordered_map>
#include "SystemWrapper.cpp"

enum Systems
{
    Chassis,
    Claw,
};

class SystemManager
{
    private:
    std::unordered_map<int, SystemWrapper> systemMap;
    // static SystemManager* manager;

    public:
    SystemWrapper* System = new SystemWrapper();
    SystemManager()
    {
        systemMap = std::unordered_map<int, SystemWrapper>(); //Map from systemID to system
        systemMap.insert(std::pair<int, SystemWrapper>(0, SystemWrapper()));
        systemMap.insert(std::pair<int, SystemWrapper>(1, SystemWrapper()));
    }
    void Add(int value)
    {
        systemMap.insert(std::pair<int, SystemWrapper>(value, SystemWrapper()));
    }

    SystemWrapper* GetSystem(int systemID)
    {
        SystemWrapper returnSystem = systemMap.at(systemID);
        return &returnSystem;
    }

    void AddPeriodic(/*takes in an ID and function*/)
    {

    }
};
