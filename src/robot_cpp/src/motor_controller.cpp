#include "robot_cpp/motor_controller.hpp"

MotorController::MotorController()
    : left_motor_("left"), right_motor_("right")
{
}

void MotorController::forward()
{
    left_motor_.forward();
    right_motor_.forward();
}

void MotorController::backward()
{
    left_motor_.backward();
    right_motor_.backward();
}

void MotorController::turn_left()
{
    left_motor_.backward();
    right_motor_.forward();
}

void MotorController::turn_right()
{
    left_motor_.forward();
    right_motor_.backward();
}

void MotorController::stop()
{
    left_motor_.stop();
    right_motor_.stop();
}