#include "robot_cpp/lidar_processor_node/lidar_processor_node.hpp"

#include <cmath>
#include <cstring>
#include <limits>

#include "sensor_msgs/point_cloud2_iterator.hpp"

LidarProcessorNode::LidarProcessorNode()
    : Node("lidar_processor_node"),
      filter_(),
      segmentation_(0.08f),
      environment_(0.7f)
{
    // ---------------------------------------------------------
    // Subscriber
    // ---------------------------------------------------------

    lidar_subscription_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/simulation/lidar_points/points",
            rclcpp::SensorDataQoS(),
            std::bind(
                &LidarProcessorNode::lidar_callback,
                this,
                std::placeholders::_1));

    // ---------------------------------------------------------
    // Filtered point cloud publisher
    // ---------------------------------------------------------
    filtered_cloud_publisher_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/robot/lidar/points",
            rclcpp::SensorDataQoS());

    // ---------------------------------------------------------
    // Environment publisher
    // ---------------------------------------------------------
    environment_publisher_ =
        this->create_publisher<robot_interfaces::msg::LidarEnvironment>(
            "/robot/lidar/environment",
            10);

    RCLCPP_INFO(
        this->get_logger(),
        "Lidar processor node started");
}

void LidarProcessorNode::lidar_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // ---------------------------------------------------------
    // 1. Acquisition / conversion
    // ---------------------------------------------------------

    const auto points = point_cloud_to_points(*msg);

    if (points.empty())
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Received an empty LiDAR point cloud");

        return;
    }

    // ---------------------------------------------------------
    // 2. Cleaning
    // ---------------------------------------------------------

    const auto filtered_points = filter_.filter(points);

    auto filtered_cloud_msg =
        points_to_point_cloud(
            filtered_points,
            msg->header
        );

    filtered_cloud_publisher_->publish(filtered_cloud_msg);

    // ---------------------------------------------------------
    // 3. Segmentation
    // ---------------------------------------------------------

    const auto segmentation_result = segmentation_.segment(filtered_points);

    // ---------------------------------------------------------
    // 4. Environment representation
    // ---------------------------------------------------------

    const auto environment_data = environment_.analyze(
        segmentation_result.ground_points,
        segmentation_result.non_ground_points
    );

    // ---------------------------------------------------------
    // 5. Publication
    // ---------------------------------------------------------

    robot_interfaces::msg::LidarEnvironment environment_msg;

    environment_msg.ground_detected = environment_data.ground_detected;

    environment_msg.obstacle_detected = environment_data.obstacle_detected;

    environment_msg.ground_distance = environment_data.ground_distance;

    environment_msg.front_distance = environment_data.front_distance;
    environment_msg.left_distance = environment_data.left_distance;
    environment_msg.right_distance = environment_data.right_distance;

    environment_msg.min_distance = environment_data.min_distance;

    environment_publisher_->publish(environment_msg);

    // ---------------------------------------------------------
    // Debug
    // ---------------------------------------------------------

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "LiDAR: %zu points | ground: %zu | non-ground: %zu | min: %.2f m",
        filtered_points.size(),
        segmentation_result.ground_points.size(),
        segmentation_result.non_ground_points.size(),
        environment_data.min_distance
    );
}

std::vector<LidarPoint> LidarProcessorNode::point_cloud_to_points(const sensor_msgs::msg::PointCloud2 &msg) const
{
    std::vector<LidarPoint> points;

    points.reserve(
        static_cast<std::size_t>(msg.width) *
        static_cast<std::size_t>(msg.height)
    );

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        LidarPoint point;

        point.x = *iter_x;
        point.y = *iter_y;
        point.z = *iter_z;

        points.push_back(point);
    }

    return points;
}

sensor_msgs::msg::PointCloud2 LidarProcessorNode::points_to_point_cloud(
    const std::vector<LidarPoint> &points,
    const std_msgs::msg::Header &header) const
{
    sensor_msgs::msg::PointCloud2 output;

    output.header = header;
    output.height = 1;
    output.width = points.size();
    output.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(output);

    modifier.setPointCloud2FieldsByString(1, "xyz");

    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(output, "z");

    for (const auto &point : points)
    {
        *iter_x = point.x;
        *iter_y = point.y;
        *iter_z = point.z;

        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    return output;
}