#ifndef ROBOT_CPP__LIDAR_PROCESSOR_NODE__LIDAR_FILTER_HPP_
#define ROBOT_CPP__LIDAR_PROCESSOR_NODE__LIDAR_FILTER_HPP_

#include <vector>

#include "robot_cpp/lidar_processor_node/lidar_point.hpp"

class LidarFilter
{
public:
    std::vector<LidarPoint> filter(
        const std::vector<LidarPoint> &points) const;
};

#endif  // ROBOT_CPP__LIDAR_PROCESSOR_NODE__LIDAR_FILTER_HPP_