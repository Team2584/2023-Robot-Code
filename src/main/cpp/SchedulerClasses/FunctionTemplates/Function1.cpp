// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// #include "Function.cpp"

// template <typename T1>
// class Function1 : public Function<T1, std::byte, std::byte>
// {
//     public:
//     Function1(std::function<bool(T1 param1)> function)
//     : base([](T1 param1, std::byte filler1, std::byte filler2) {return function(param1);})
//     {

//     }

//     bool Run(T1 parameter1)
//     {
//         return base::Run(parameter1, 0, 0);
//     }
// };