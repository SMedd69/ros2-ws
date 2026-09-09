#include "robot_cpp/lidar_processor_node/lidar_filter.hpp"

#include <cmath>

std::vector<LidarPoint> LidarFilter::filter(
    const std::vector<LidarPoint> &points) const
{
    std::vector<LidarPoint> filtered_points;

    filtered_points.reserve(points.size());

    for (const auto &point : points)
    {
        if (!std::isfinite(point.x) ||
            !std::isfinite(point.y) ||
            !std::isfinite(point.z))
        {
            continue;
        }

        filtered_points.push_back(point);
    }

    return filtered_points;
}