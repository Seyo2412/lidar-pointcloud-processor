#pragma once

#include "point_cloud.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tuple>
#include <memory>
#include <cmath>

namespace lidar
{

    struct PlaneModel
    {
        Eigen::Vector3f normal{0.0f, 0.0f, 1.0f};
        float d{0.0f};

        [[nodiscard]] float distanceTo(const Eigen::Vector3f &p) const noexcept
        {
            return std::abs(normal.dot(p) + d);
        }
    };

    class RansacPlaneSegmenter
    {
    public:
        RansacPlaneSegmenter(int max_iterations = 100, float distance_threshold = 0.15f)
            : max_iterations_(max_iterations), distance_threshold_(distance_threshold) {}

        void setMaxIterations(int iters) noexcept { max_iterations_ = iters; }
        void setDistanceThreshold(float thresh) noexcept { distance_threshold_ = thresh; }

        [[nodiscard]] std::tuple<PointCloud::Ptr, PointCloud::Ptr, PlaneModel>
        segment(const PointCloud &input) const;

    private:
        int max_iterations_{100};
        float distance_threshold_{0.15f};
    };

} // namespace lidar