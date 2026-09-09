#include "robot_cpp/lidar_processor_node/lidar_environment.hpp"

#include <cmath>
#include <limits>

LidarEnvironment::LidarEnvironment(float obstacle_threshold)
    : obstacle_threshold_(obstacle_threshold)
{
}

LidarEnvironmentData LidarEnvironment::analyze(
    const std::vector<LidarPoint> &ground_points,
    const std::vector<LidarPoint> &non_ground_points) const
{
    LidarEnvironmentData environment;

    // ---------------------------------------------------------
    // Ground
    // ---------------------------------------------------------

    if (!ground_points.empty())
    {
        environment.ground_detected = true;

        environment.ground_distance = min_horizontal_distance(ground_points);
    }

    // ---------------------------------------------------------
    // Minimum obstacle distance
    // ---------------------------------------------------------

    if (!non_ground_points.empty())
    {
        environment.min_distance = min_horizontal_distance(non_ground_points);
    }

    // ---------------------------------------------------------
    // Obstacle sectors
    //
    // angle = atan2(y, x)
    //
    //        LEFT
    //          /
    //         /
    //        ↑
    //        │
    //        │ FRONT
    //        │
    //        ↓
    //
    //          \
    //           \
    //          RIGHT
    // ---------------------------------------------------------

    float front_distance = std::numeric_limits<float>::max();
    float left_distance = std::numeric_limits<float>::max();
    float right_distance = std::numeric_limits<float>::max();

    constexpr float right_sector_min = -0.5f;
    constexpr float right_sector_max = -0.166f;

    constexpr float front_sector_min = -0.166f;
    constexpr float front_sector_max = 0.166f;

    constexpr float left_sector_min = 0.166f;
    constexpr float left_sector_max = 0.5f;

    for (const auto &point : non_ground_points)
    {
        const float distance = std::sqrt(point.x * point.x + point.y * point.y);

        if (distance > obstacle_threshold_)
        {
            continue;
        }

        const float angle = std::atan2(point.y, point.x);

        environment.obstacle_detected = true;

        if (angle >= right_sector_min && angle < right_sector_max)
        {
            right_distance = std::min(right_distance, distance);
        }
        else if (angle >= front_sector_min && angle <= front_sector_max)
        {
            front_distance = std::min(front_distance, distance);
        }
        else if (angle > left_sector_min && angle <= left_sector_max)
        {
            left_distance = std::min(left_distance, distance);
        }
    }

    // ---------------------------------------------------------
    // No obstacle in sector
    // ---------------------------------------------------------

    if (front_distance != std::numeric_limits<float>::max())
    {
        environment.front_distance = front_distance;
    }

    if (left_distance != std::numeric_limits<float>::max())
    {
        environment.left_distance = left_distance;
    }

    if (right_distance != std::numeric_limits<float>::max())
    {
        environment.right_distance = right_distance;
    }

    return environment;
}

float LidarEnvironment::min_horizontal_distance(const std::vector<LidarPoint> &points) const
{
    if (points.empty())
    {
        return 0.0f;
    }

    float minimum = std::numeric_limits<float>::max();

    for (const auto &point : points)
    {
        const float distance = std::sqrt(point.x * point.x + point.y * point.y);
        minimum = std::min(minimum, distance);
    }

    return minimum;
}