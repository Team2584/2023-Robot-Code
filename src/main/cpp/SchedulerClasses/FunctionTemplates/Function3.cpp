// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Function.cpp"

template <typename T1, typename T2, typename T3>
class Function3 : private Function<T1, T2, T3>
{
    public:
    Function3(std::function<bool(T1 param1, T2 param2, T3 param3)> function)
    : base([](T1 param1, T2 param2, T3 param3) {return function(param1, param2, param3);})
    {

    }

    bool Run(T1 parameter1, T2 parameter2, T3 parameter3)
    {
        return base::Run(parameter1, parameter2, parameter3);
    }
};