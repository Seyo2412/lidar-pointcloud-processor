#pragma once

#include <Eigen/Core>
#include <vector>
#include <cstdint>

namespace lidar
{

    struct alignas(16) Point3D
    {
        float x{0.0f};
        float y{0.0f};
        float z{0.0f};
        float intensity{0.0f}; // LiDAR-Reflexionsgrad [0.0, 1.0]
        uint32_t ring{0};      // Laser-Ring ID
        int32_t label{0};      // Segmentierungs-Label (0: Unlabeled, 1: Ground, 2+: Cluster)

        Point3D() = default;
        Point3D(float x_, float y_, float z_, float intensity_ = 1.0f, uint32_t ring_ = 0)
            : x(x_), y(y_), z(z_), intensity(intensity_), ring(ring_) {}

        [[nodiscard]] Eigen::Vector3f toEigen() const
        {
            return Eigen::Vector3f(x, y, z);
        }
    };

} // namespace lidar