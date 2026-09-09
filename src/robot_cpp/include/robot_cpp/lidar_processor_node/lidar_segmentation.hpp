#ifndef ROBOT_CPP__LIDAR_PROCESSOR_NODE__LIDAR_SEGMENTATION_HPP_
#define ROBOT_CPP__LIDAR_PROCESSOR_NODE__LIDAR_SEGMENTATION_HPP_

#include <vector>

#include "robot_cpp/lidar_processor_node/lidar_point.hpp"

struct LidarSegmentationResult
{
    std::vector<LidarPoint> ground_points;
    std::vector<LidarPoint> non_ground_points;
};

class LidarSegmentation
{
public:
    explicit LidarSegmentation(float ground_z_threshold = 0.08f);

    LidarSegmentationResult segment(const std::vector<LidarPoint> &points) const;

private:
    float ground_z_threshold_;
};

#endif  // ROBOT_CPP__LIDAR_PROCESSOR_NODE__LIDAR_SEGMENTATION_HPP_