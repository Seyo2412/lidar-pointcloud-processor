#pragma once

#include "point_cloud.hpp"
#include <vector>

namespace lidar
{

    struct BoundingBox3D
    {
        Eigen::Vector3f min_point;
        Eigen::Vector3f max_point;
        Eigen::Vector3f center;
        size_t point_count{0};
    };

    class EuclideanClusterExtractor
    {
    public:
        EuclideanClusterExtractor(float cluster_tolerance = 0.6f,
                                  size_t min_cluster_size = 20,
                                  size_t max_cluster_size = 15000)
            : tolerance_(cluster_tolerance),
              min_size_(min_cluster_size),
              max_size_(max_cluster_size) {}

        void setClusterTolerance(float tol) { tolerance_ = tol; }
        void setMinClusterSize(size_t s) { min_size_ = s; }
        void setMaxClusterSize(size_t s) { max_size_ = s; }

        // Gibt Liste separater Punktwolken-Cluster und zugehörige Bounding-Boxen zurück
        [[nodiscard]] std::pair<std::vector<PointCloud::Ptr>, std::vector<BoundingBox3D>>
        extract(const PointCloud &input) const;

    private:
        float tolerance_{0.6f};
        size_t min_size_{20};
        size_t max_size_{15000};
    };

} // namespace lidar