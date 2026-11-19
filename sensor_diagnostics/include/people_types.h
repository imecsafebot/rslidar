#pragma once

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <cstdint>

struct PersonDetection
{
    int id = -1;
    Eigen::Vector3f center = Eigen::Vector3f::Zero();

    float width = 0.0f;
    float depth = 0.0f;
    float height = 0.0f;

    std::size_t points = 0;
    float speed = 0.0f;
    bool moving = false;
};

using InputPoint = pcl::PointXYZI;
using InputCloud = pcl::PointCloud<InputPoint>;
