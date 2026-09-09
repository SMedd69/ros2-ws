#include "robot_cpp/lidar_processor_node/lidar_segmentation.hpp"

LidarSegmentation::LidarSegmentation(float ground_z_threshold)
    : ground_z_threshold_(ground_z_threshold)
{
}

LidarSegmentationResult LidarSegmentation::segment(const std::vector<LidarPoint> &points) const
{
    LidarSegmentationResult result;

    result.ground_points.reserve(points.size());
    result.non_ground_points.reserve(points.size());

    for (const auto &point : points)
    {
        if (point.z <= ground_z_threshold_)
        {
            result.ground_points.push_back(point);
        }
        else
        {
            result.non_ground_points.push_back(point);
        }
    }

    return result;
}