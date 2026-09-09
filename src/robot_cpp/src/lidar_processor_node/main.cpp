#include "rclcpp/rclcpp.hpp"
#include "robot_cpp/lidar_processor_node/lidar_processor_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LidarProcessorNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}