#ifndef ROBOT_CPP__LIDAR_PROCESSOR_NODE__LIDAR_PROCESSOR_NODE_HPP_
#define ROBOT_CPP__LIDAR_PROCESSOR_NODE__LIDAR_PROCESSOR_NODE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

#include "robot_interfaces/msg/lidar_environment.hpp"

#include "robot_cpp/lidar_processor_node/lidar_point.hpp"
#include "robot_cpp/lidar_processor_node/lidar_filter.hpp"
#include "robot_cpp/lidar_processor_node/lidar_segmentation.hpp"
#include "robot_cpp/lidar_processor_node/lidar_environment.hpp"

class LidarProcessorNode : public rclcpp::Node
{
public:
    LidarProcessorNode();

private:
    void lidar_callback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    std::vector<LidarPoint> point_cloud_to_points(const sensor_msgs::msg::PointCloud2 &msg) const;
    sensor_msgs::msg::PointCloud2 points_to_point_cloud(const std::vector<LidarPoint> &points, const std_msgs::msg::Header &header) const;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        filtered_cloud_publisher_;

    rclcpp::Publisher<robot_interfaces::msg::LidarEnvironment>::SharedPtr
        environment_publisher_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        lidar_subscription_;

    LidarFilter filter_;
    LidarSegmentation segmentation_;
    LidarEnvironment environment_;
};

#endif  // ROBOT_CPP__LIDAR_PROCESSOR_NODE__LIDAR_PROCESSOR_NODE_HPP_