#pragma once

#include "point_cloud.hpp"

namespace lidar
{

    class SyntheticSceneGenerator
    {
    public:
        static PointCloud::Ptr generateUrbanStreet(size_t target_points = 80000);
    };

} // namespace lidar