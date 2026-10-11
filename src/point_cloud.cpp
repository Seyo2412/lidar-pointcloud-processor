#include "point_cloud.hpp"
#include <limits>

namespace lidar
{

    void PointCloud::getBoundingBox(Eigen::Vector3f &min_bound, Eigen::Vector3f &max_bound) const
    {
        if (points_.empty())
        {
            min_bound.setZero();
            max_bound.setZero();
            return;
        }

        min_bound.setConstant(std::numeric_limits<float>::max());
        max_bound.setConstant(std::numeric_limits<float>::lowest());

        for (const auto &pt : points_)
        {
            min_bound.x() = std::min(min_bound.x(), pt.x);
            min_bound.y() = std::min(min_bound.y(), pt.y);
            min_bound.z() = std::min(min_bound.z(), pt.z);

            max_bound.x() = std::max(max_bound.x(), pt.x);
            max_bound.y() = std::max(max_bound.y(), pt.y);
            max_bound.z() = std::max(max_bound.z(), pt.z);
        }
    }

} // namespace lidar