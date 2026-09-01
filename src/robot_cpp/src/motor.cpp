#include "robot_cpp/motor.hpp"
#include <iostream>

Motor::Motor(const std::string& name): name_(name)
{
}

void Motor::forward()
{
    std::cout << "(" << name_.c_str() << ")" << " Motor forward" << std::endl;
}

void Motor::backward()
{
    std::cout << "(" << name_.c_str() << ")" << " Motor backward" << std::endl;
}

void Motor::stop()
{
    std::cout << "(" << name_.c_str() << ")" << " Motor stop" << std::endl;
}