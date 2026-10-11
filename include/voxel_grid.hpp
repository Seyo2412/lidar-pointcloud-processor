#pragma once

#include "point_cloud.hpp"
#include <unordered_map>

namespace lidar
{

    struct VoxelCoord
    {
        int64_t x{0};
        int64_t y{0};
        int64_t z{0};

        bool operator==(const VoxelCoord &other) const noexcept
        {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct VoxelCoordHash
    {
        std::size_t operator()(const VoxelCoord &c) const noexcept
        {
            // 64-Bit FNV/Murmur-basierter Hash für 3D-Koordinaten
            std::size_t h = 14695981039346656037ULL;
            h = (h ^ static_cast<std::size_t>(c.x)) * 1099511628211ULL;
            h = (h ^ static_cast<std::size_t>(c.y)) * 1099511628211ULL;
            h = (h ^ static_cast<std::size_t>(c.z)) * 1099511628211ULL;
            return h;
        }
    };

    class VoxelGridFilter
    {
    public:
        explicit VoxelGridFilter(float leaf_size = 0.2f) : leaf_size_(leaf_size) {}

        void setLeafSize(float size) { leaf_size_ = size; }
        [[nodiscard]] float getLeafSize() const noexcept { return leaf_size_; }

        [[nodiscard]] PointCloud::Ptr filter(const PointCloud &input) const;

    private:
        float leaf_size_{0.2f};
    };

} // namespace lidar