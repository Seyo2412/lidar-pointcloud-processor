#include "voxel_grid.hpp"
#include <cmath>

namespace lidar
{

    struct CentroidAccumulator
    {
        Eigen::Vector3f sum_pos{0.0f, 0.0f, 0.0f};
        float sum_intensity{0.0f};
        uint32_t count{0};
        int32_t dominant_label{0};
    };

    PointCloud::Ptr VoxelGridFilter::filter(const PointCloud &input) const
    {
        auto filtered = std::make_shared<PointCloud>();
        if (input.empty() || leaf_size_ <= 1e-4f)
        {
            return filtered;
        }

        const float inv_leaf = 1.0f / leaf_size_;
        std::unordered_map<VoxelCoord, CentroidAccumulator, VoxelCoordHash> voxel_map;
        voxel_map.reserve(input.size() / 2);

        for (const auto &pt : input.points())
        {
            VoxelCoord coord{
                static_cast<int64_t>(std::floor(pt.x * inv_leaf)),
                static_cast<int64_t>(std::floor(pt.y * inv_leaf)),
                static_cast<int64_t>(std::floor(pt.z * inv_leaf))};

            auto &acc = voxel_map[coord];
            acc.sum_pos += pt.toEigen();
            acc.sum_intensity += pt.intensity;
            acc.dominant_label = pt.label;
            acc.count += 1;
        }

        filtered->reserve(voxel_map.size());
        for (const auto &[coord, acc] : voxel_map)
        {
            float inv_count = 1.0f / static_cast<float>(acc.count);
            Eigen::Vector3f center = acc.sum_pos * inv_count;
            Point3D p(center.x(), center.y(), center.z(), acc.sum_intensity * inv_count);
            p.label = acc.dominant_label;
            filtered->addPoint(p);
        }

        return filtered;
    }

} // namespace lidar