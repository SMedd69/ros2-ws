#pragma once

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/robot_command.hpp"
#include "robot_interfaces/msg/safety.hpp"
#include "robot_interfaces/msg/component_status.hpp"
#include "robot_cpp/motor_controller.hpp"

class MotorNode : public rclcpp::Node
{
public:
    MotorNode();

private:
    void command_callback(
        const robot_interfaces::msg::RobotCommand::SharedPtr msg
    );

    void safety_callback(
        const robot_interfaces::msg::Safety::SharedPtr msg
    );


    // Current safety level
    uint8_t safety_level_;

    MotorController motor_controller_;

    rclcpp::Subscription<
        robot_interfaces::msg::RobotCommand
    >::SharedPtr command_subscription_;

    rclcpp::Subscription<
        robot_interfaces::msg::Safety
    >::SharedPtr safety_subscription_;
};