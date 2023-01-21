// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Function.cpp"

template <typename T1, typename T2>
class Function2 : private Function<T1, T2, byte>
{
    public:
    Function2(std::function<bool(T1 param1, T2 param2)> function)
    : base([](T1 param1, T2 param2, byte filler1) {return function(param1, param2);})
    {

    }

    bool Run(T1 parameter1, T2 parameter2)
    {
        return base::Run(parameter1, parameter2, 0);
    }
};