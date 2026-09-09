#ifndef ROBOT_CPP__LIDAR_PROCESSOR_NODE__LIDAR_ENVIRONMENT_HPP_
#define ROBOT_CPP__LIDAR_PROCESSOR_NODE__LIDAR_ENVIRONMENT_HPP_

#include <vector>

#include "robot_cpp/lidar_processor_node/lidar_point.hpp"

struct LidarEnvironmentData
{
    bool ground_detected = false;
    bool obstacle_detected = false;

    float ground_distance = 0.0f;

    float front_distance = 0.0f;
    float left_distance = 0.0f;
    float right_distance = 0.0f;

    float min_distance = 0.0f;
};

class LidarEnvironment
{
public:
    explicit LidarEnvironment(float obstacle_threshold = 0.7f);

    LidarEnvironmentData analyze(
        const std::vector<LidarPoint> &ground_points,
        const std::vector<LidarPoint> &non_ground_points) const;

private:
    float obstacle_threshold_;

    float min_horizontal_distance(
        const std::vector<LidarPoint> &points) const;
};

#endif  // ROBOT_CPP__LIDAR_PROCESSOR_NODE__LIDAR_ENVIRONMENT_HPP_