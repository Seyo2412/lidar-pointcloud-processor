#include "cluster_extraction.hpp"
#include <unordered_map>
#include <queue>
#include <cmath>

namespace lidar
{

    namespace
    {
        struct CellCoord
        {
            int64_t x{0}, y{0}, z{0};
            bool operator==(const CellCoord &o) const noexcept
            {
                return x == o.x && y == o.y && z == o.z;
            }
        };

        struct CellCoordHash
        {
            std::size_t operator()(const CellCoord &c) const noexcept
            {
                std::size_t h = 14695981039346656037ULL;
                h = (h ^ static_cast<std::size_t>(c.x)) * 1099511628211ULL;
                h = (h ^ static_cast<std::size_t>(c.y)) * 1099511628211ULL;
                h = (h ^ static_cast<std::size_t>(c.z)) * 1099511628211ULL;
                return h;
            }
        };
    } // namespace

    std::pair<std::vector<PointCloud::Ptr>, std::vector<BoundingBox3D>>
    EuclideanClusterExtractor::extract(const PointCloud &input) const
    {
        std::vector<PointCloud::Ptr> clusters;
        std::vector<BoundingBox3D> bboxes;

        const size_t n_points = input.size();
        if (n_points == 0)
        {
            return {clusters, bboxes};
        }

        // 1. Punkte in Spatial Grid einordnen
        const float inv_tol = 1.0f / tolerance_;
        std::unordered_map<CellCoord, std::vector<size_t>, CellCoordHash> grid;
        grid.reserve(n_points);

        for (size_t i = 0; i < n_points; ++i)
        {
            const auto &pt = input[i];
            CellCoord c{
                static_cast<int64_t>(std::floor(pt.x * inv_tol)),
                static_cast<int64_t>(std::floor(pt.y * inv_tol)),
                static_cast<int64_t>(std::floor(pt.z * inv_tol))};
            grid[c].push_back(i);
        }

        // 2. BFS Flood Fill Clustering
        std::vector<bool> visited(n_points, false);
        const float tol_sq = tolerance_ * tolerance_;
        int32_t cluster_id = 2; // Labels 0=unlabeled, 1=ground, ab 2=Cluster

        for (size_t i = 0; i < n_points; ++i)
        {
            if (visited[i])
                continue;

            std::vector<size_t> cluster_indices;
            std::queue<size_t> q;
            q.push(i);
            visited[i] = true;

            while (!q.empty())
            {
                size_t curr = q.front();
                q.pop();
                cluster_indices.push_back(curr);

                const auto &pt = input[curr];
                CellCoord c{
                    static_cast<int64_t>(std::floor(pt.x * inv_tol)),
                    static_cast<int64_t>(std::floor(pt.y * inv_tol)),
                    static_cast<int64_t>(std::floor(pt.z * inv_tol))};

                // 3x3x3 Nachbarzellen untersuchen
                for (int64_t dx = -1; dx <= 1; ++dx)
                {
                    for (int64_t dy = -1; dy <= 1; ++dy)
                    {
                        for (int64_t dz = -1; dz <= 1; ++dz)
                        {
                            CellCoord neighbor{c.x + dx, c.y + dy, c.z + dz};
                            auto it = grid.find(neighbor);
                            if (it == grid.end())
                                continue;

                            for (size_t neighbor_idx : it->second)
                            {
                                if (visited[neighbor_idx])
                                    continue;

                                float d2 = (input[neighbor_idx].toEigen() - pt.toEigen()).squaredNorm();
                                if (d2 <= tol_sq)
                                {
                                    visited[neighbor_idx] = true;
                                    q.push(neighbor_idx);
                                }
                            }
                        }
                    }
                }
            }

            // Größenfilter prüfen
            if (cluster_indices.size() >= min_size_ && cluster_indices.size() <= max_size_)
            {
                auto cluster_cloud = std::make_shared<PointCloud>();
                cluster_cloud->reserve(cluster_indices.size());

                BoundingBox3D bbox;
                bbox.point_count = cluster_indices.size();

                Eigen::Vector3f min_p(1e6f, 1e6f, 1e6f);
                Eigen::Vector3f max_p(-1e6f, -1e6f, -1e6f);

                for (size_t idx : cluster_indices)
                {
                    Point3D p = input[idx];
                    p.label = cluster_id;
                    cluster_cloud->addPoint(p);

                    min_p = min_p.cwiseMin(p.toEigen());
                    max_p = max_p.cwiseMax(p.toEigen());
                }

                bbox.min_point = min_p;
                bbox.max_point = max_p;
                bbox.center = 0.5f * (min_p + max_p);

                clusters.push_back(cluster_cloud);
                bboxes.push_back(bbox);
                cluster_id++;
            }
        }

        return {clusters, bboxes};
    }

} // namespace lidar