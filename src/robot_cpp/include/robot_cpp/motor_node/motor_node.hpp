#ifndef MOTOR_NODE_HPP
#define MOTOR_NODE_HPP

#include <chrono>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "robot_interfaces/msg/robot_command.hpp"
#include "robot_interfaces/msg/safety.hpp"
#include "robot_interfaces/msg/component_status.hpp"

#include "robot_cpp/motor_node/motor_controller.hpp"


class MotorNode : public rclcpp::Node
{
public:

    MotorNode();

private:

    // --------------------------------
    // Callbacks
    // --------------------------------

    void command_callback(
        const robot_interfaces::msg::RobotCommand::SharedPtr msg
    );

    void safety_callback(
        const robot_interfaces::msg::Safety::SharedPtr msg
    );

    void safety_watchdog_callback();

    // --------------------------------
    // Motor control
    // --------------------------------

    void publish_velocity(
        double linear,
        double angular
    );

    void stop_robot();

    // --------------------------------
    // ROS
    // --------------------------------

    rclcpp::Subscription<robot_interfaces::msg::RobotCommand>::SharedPtr
        command_subscription_;

    rclcpp::Subscription<robot_interfaces::msg::Safety>::SharedPtr
        safety_subscription_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
        cmd_vel_publisher_;

    rclcpp::TimerBase::SharedPtr
        safety_watchdog_timer_;

    // --------------------------------
    // Safety
    // --------------------------------

    uint8_t safety_level_;

    std::chrono::steady_clock::time_point
        last_safety_message_;

    bool safety_timeout_active_;

    // --------------------------------
    // Motor controller
    // --------------------------------

    MotorController motor_controller_;
};

#endif