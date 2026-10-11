#pragma once

#include "point_types.hpp"
#include <string>
#include <memory>

namespace lidar
{

    class PointCloud
    {
    public:
        using Ptr = std::shared_ptr<PointCloud>;
        using ConstPtr = std::shared_ptr<const PointCloud>;

        PointCloud() = default;

        void addPoint(const Point3D &pt) { points_.push_back(pt); }
        void reserve(size_t n) { points_.reserve(n); }
        void clear() { points_.clear(); }

        [[nodiscard]] size_t size() const noexcept { return points_.size(); }
        [[nodiscard]] bool empty() const noexcept { return points_.empty(); }

        [[nodiscard]] std::vector<Point3D> &points() noexcept { return points_; }
        [[nodiscard]] const std::vector<Point3D> &points() const noexcept { return points_; }

        Point3D &operator[](size_t idx) { return points_[idx]; }
        const Point3D &operator[](size_t idx) const { return points_[idx]; }

        // I/O & Utils
        void getBoundingBox(Eigen::Vector3f &min_bound, Eigen::Vector3f &max_bound) const;

    private:
        std::vector<Point3D> points_;
    };

} // namespace lidar