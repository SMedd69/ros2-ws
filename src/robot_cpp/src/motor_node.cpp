#include "robot_cpp/motor_node.hpp"


MotorNode::MotorNode()
: Node("motor_node"),
  safety_level_(robot_interfaces::msg::ComponentStatus::CRITICAL)
{

    // --------------------------------
    // Robot command subscription
    // --------------------------------

    command_subscription_ = this->create_subscription<robot_interfaces::msg::RobotCommand>(
        "/robot/cmd",
        10,
        std::bind(
            &MotorNode::command_callback,
            this,
            std::placeholders::_1
        )
    );

    // --------------------------------
    // Safety subscription
    // --------------------------------

    safety_subscription_ = this->create_subscription<robot_interfaces::msg::Safety>(
        "/robot/safety",
        10,
        std::bind(
            &MotorNode::safety_callback,
            this,
            std::placeholders::_1
        )
    );

    RCLCPP_INFO(this->get_logger(), "Motor Node started");

    last_safety_message_ = std::chrono::steady_clock::now();

    safety_watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MotorNode::safety_watchdog_callback, this)
    );
}

// --------------------------------
// Safety callback
// --------------------------------
void MotorNode::safety_callback(const robot_interfaces::msg::Safety::SharedPtr msg)
{
    last_safety_message_ = std::chrono::steady_clock::now();

    if (safety_timeout_active_)
    {
        RCLCPP_INFO(this->get_logger(), "SafetyNode communication restored.");
    }

    safety_timeout_active_ = false;

    safety_level_ = msg->global_level;

    RCLCPP_INFO(
        this->get_logger(),
        "Safety level: %u | %s",
        safety_level_,
        msg->global_reason.c_str()
    );

    if (safety_level_ >= robot_interfaces::msg::ComponentStatus::CRITICAL)
    {
        motor_controller_.stop();
        RCLCPP_WARN(this->get_logger(), "Safety level too high -> STOP");
    }
}


// --------------------------------
// Wathcdog callback
// --------------------------------
void MotorNode::safety_watchdog_callback()
{
    const auto now = std::chrono::steady_clock::now();

    const auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_safety_message_
        );

    if (elapsed > std::chrono::milliseconds(500))
    {
        safety_level_ =
            robot_interfaces::msg::ComponentStatus::CRITICAL;

        if (!safety_timeout_active_)
        {
            motor_controller_.stop();

            RCLCPP_ERROR(
                this->get_logger(),
                "SafetyNode timeout! "
                "No safety message received for more than 500 ms. "
                "Motor stopped."
            );

            safety_timeout_active_ = true;
        }
    }
}

// --------------------------------
// Robot command callback
// --------------------------------
void MotorNode::command_callback(const robot_interfaces::msg::RobotCommand::SharedPtr msg)
{
    // --------------------------------
    // Safety has priority
    // --------------------------------

    if (safety_level_ >= robot_interfaces::msg::ComponentStatus::CRITICAL)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Command ignored: safety level too high"
        );

        motor_controller_.stop();

        return;
    }


    // --------------------------------
    // Execute command
    // --------------------------------

    switch (msg->command)
    {
        case robot_interfaces::msg::RobotCommand::FORWARD:

            RCLCPP_INFO(
                this->get_logger(),
                "FORWARD"
            );

            motor_controller_.forward();

            break;


        case robot_interfaces::msg::RobotCommand::STOP:

            RCLCPP_INFO(
                this->get_logger(),
                "STOP"
            );

            motor_controller_.stop();

            break;


        case robot_interfaces::msg::RobotCommand::LOW_BATTERY:

            RCLCPP_INFO(
                this->get_logger(),
                "LOW_BATTERY"
            );

            motor_controller_.backward();

            break;


        case robot_interfaces::msg::RobotCommand::GO_CHARGING:

            RCLCPP_INFO(
                this->get_logger(),
                "GO_CHARGING"
            );

            motor_controller_.turn_left();

            break;


        default:

            RCLCPP_WARN(
                this->get_logger(),
                "Unknown command"
            );

            motor_controller_.stop();

            break;
    }
}
