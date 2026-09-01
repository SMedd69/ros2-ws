#pragma once

#include "robot_cpp/motor.hpp"

class MotorController
{
public:
    MotorController();

    void forward();
    void backward();
    void turn_left();
    void turn_right();
    void stop();

private:
    Motor left_motor_;
    Motor right_motor_;
};