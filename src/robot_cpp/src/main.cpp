#include "rclcpp/rclcpp.hpp"
#include "robot_cpp/motor_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MotorNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}