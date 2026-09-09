#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include <string>

class MotorController
{
public:

    MotorController();

    void forward(double speed);
    void backward(double speed);

    void turn_left(double speed);
    void turn_right(double speed);

    void turn_around(double speed);

    void stop();

};

#endif