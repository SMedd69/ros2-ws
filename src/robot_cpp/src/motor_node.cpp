#include "robot_cpp/motor_node.hpp"
#include <cmath>

MotorNode::MotorNode()
: Node("motor_node"),
  safety_level_(robot_interfaces::msg::ComponentStatus::CRITICAL),
  safety_timeout_active_(false)
{

    // --------------------------------
    // /robot/cmd
    // --------------------------------

    command_subscription_ =
        this->create_subscription<robot_interfaces::msg::RobotCommand>(
            "/robot/cmd",
            10,
            std::bind(
                &MotorNode::command_callback,
                this,
                std::placeholders::_1
            )
        );


    // --------------------------------
    // /robot/safety
    // --------------------------------

    safety_subscription_ =
        this->create_subscription<robot_interfaces::msg::Safety>(
            "/robot/safety",
            10,
            std::bind(
                &MotorNode::safety_callback,
                this,
                std::placeholders::_1
            )
        );


    // --------------------------------
    // Gazebo /cmd_vel
    // --------------------------------

    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10
        );


    // --------------------------------
    // Initial state
    // --------------------------------

    last_safety_message_ =
        std::chrono::steady_clock::now();


    // --------------------------------
    // Safety watchdog
    // --------------------------------

    safety_watchdog_timer_ =
        this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(
                &MotorNode::safety_watchdog_callback,
                this
            )
        );


    RCLCPP_INFO(
        this->get_logger(),
        "Motor Node started"
    );
}


// ============================================================
// SAFETY CALLBACK
// ============================================================

void MotorNode::safety_callback(
    const robot_interfaces::msg::Safety::SharedPtr msg
)
{
    last_safety_message_ =
        std::chrono::steady_clock::now();


    if (safety_timeout_active_)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "SafetyNode communication restored."
        );
    }


    safety_timeout_active_ = false;


    safety_level_ = msg->global_level;

    // --------------------------------
    // CRITICAL = immediate stop
    // --------------------------------

    if (
        safety_level_
        >= robot_interfaces::msg::ComponentStatus::CRITICAL
    )
    {
        stop_robot();

        RCLCPP_WARN(
            this->get_logger(),
            "Safety level too high -> STOP"
        );
    }
}


// ============================================================
// SAFETY WATCHDOG
// ============================================================

void MotorNode::safety_watchdog_callback()
{
    const auto now =
        std::chrono::steady_clock::now();


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
            stop_robot();


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


// ============================================================
// ROBOT COMMAND
// ============================================================

void MotorNode::command_callback(
    const robot_interfaces::msg::RobotCommand::SharedPtr msg
)
{

    // --------------------------------
    // Safety priority
    // --------------------------------

    if (
        safety_level_
        >= robot_interfaces::msg::ComponentStatus::CRITICAL
    )
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Command ignored: safety level too high"
        );

        stop_robot();

        return;
    }


    // --------------------------------
    // Execute command
    // --------------------------------

    switch (msg->command)
    {

        // ================================================
        // FORWARD
        // ================================================

        case robot_interfaces::msg::RobotCommand::FORWARD:

            RCLCPP_INFO(
                this->get_logger(),
                "FORWARD | speed=%.2f",
                msg->linear_speed
            );

            publish_velocity(
                msg->linear_speed,
                msg->angular_speed
            );

            break;


        // ================================================
        // REVERSE
        // ================================================

        case robot_interfaces::msg::RobotCommand::REVERSE:

            RCLCPP_INFO(
                this->get_logger(),
                "REVERSE | speed=%.2f",
                msg->linear_speed
            );

            publish_velocity(
                -msg->linear_speed,
                msg->angular_speed
            );

            break;


        // ================================================
        // TURN LEFT
        // ================================================

        case robot_interfaces::msg::RobotCommand::TURN_LEFT:

            RCLCPP_INFO(
                this->get_logger(),
                "TURN_LEFT | angular=%.2f",
                msg->angular_speed
            );

            publish_velocity(
                msg->linear_speed,
                std::abs(msg->angular_speed)
            );

            break;


        // ================================================
        // TURN RIGHT
        // ================================================

        case robot_interfaces::msg::RobotCommand::TURN_RIGHT:

            RCLCPP_INFO(
                this->get_logger(),
                "TURN_RIGHT | angular=%.2f",
                msg->angular_speed
            );

            publish_velocity(
                msg->linear_speed,
                -std::abs(msg->angular_speed)
            );

            break;


        // ================================================
        // TURN AROUND
        // ================================================

        case robot_interfaces::msg::RobotCommand::TURN_AROUND:

            RCLCPP_INFO(
                this->get_logger(),
                "TURN_AROUND | angular=%.2f",
                msg->angular_speed
            );

            publish_velocity(
                0.0,
                std::abs(msg->angular_speed)
            );

            break;


        // ================================================
        // STOP
        // ================================================

        case robot_interfaces::msg::RobotCommand::STOP:

            RCLCPP_INFO(
                this->get_logger(),
                "STOP"
            );

            stop_robot();

            break;


        // ================================================
        // LOW BATTERY
        // ================================================

        case robot_interfaces::msg::RobotCommand::LOW_BATTERY:

            RCLCPP_INFO(
                this->get_logger(),
                "LOW_BATTERY"
            );

            // Pour l'instant :
            // arrêt du robot.
            stop_robot();

            break;


        // ================================================
        // GO CHARGING
        // ================================================

        case robot_interfaces::msg::RobotCommand::GO_CHARGING:

            RCLCPP_INFO(
                this->get_logger(),
                "GO_CHARGING"
            );

            // La navigation vers la station sera
            // implémentée plus tard.
            stop_robot();

            break;


        // ================================================
        // UNKNOWN
        // ================================================

        default:

            RCLCPP_WARN(
                this->get_logger(),
                "Unknown command"
            );

            stop_robot();

            break;
    }
}


// ============================================================
// PUBLISH VELOCITY
// ============================================================

void MotorNode::publish_velocity(
    double linear,
    double angular
)
{
    geometry_msgs::msg::Twist msg;


    msg.linear.x = linear;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;


    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = angular;


    cmd_vel_publisher_->publish(msg);
}


// ============================================================
// STOP ROBOT
// ============================================================

void MotorNode::stop_robot()
{
    geometry_msgs::msg::Twist msg;

    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;

    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;

    cmd_vel_publisher_->publish(msg);
}