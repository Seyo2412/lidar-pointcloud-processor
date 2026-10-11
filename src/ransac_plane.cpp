#include "ransac_plane.hpp"
#include <Eigen/Geometry>
#include <random>
#include <unordered_set>

namespace lidar
{

    std::tuple<PointCloud::Ptr, PointCloud::Ptr, PlaneModel>
    RansacPlaneSegmenter::segment(const PointCloud &input) const
    {
        auto ground_cloud = std::make_shared<PointCloud>();
        auto obstacle_cloud = std::make_shared<PointCloud>();

        const size_t n_points = input.size();
        if (n_points < 3)
        {
            return {ground_cloud, obstacle_cloud, PlaneModel{}};
        }

        std::mt19937 rng(1337);
        std::uniform_int_distribution<size_t> dist(0, n_points - 1);

        std::unordered_set<size_t> best_inliers;
        PlaneModel best_plane;

        for (int iter = 0; iter < max_iterations_; ++iter)
        {
            // Drei distinkte Punkte ziehen
            size_t idx1 = dist(rng);
            size_t idx2 = dist(rng);
            size_t idx3 = dist(rng);

            if (idx1 == idx2 || idx1 == idx3 || idx2 == idx3)
            {
                continue;
            }

            Eigen::Vector3f p1 = input[idx1].toEigen();
            Eigen::Vector3f p2 = input[idx2].toEigen();
            Eigen::Vector3f p3 = input[idx3].toEigen();

            Eigen::Vector3f v1 = p2 - p1;
            Eigen::Vector3f v2 = p3 - p1;
            Eigen::Vector3f normal = v1.cross(v2);

            if (normal.squaredNorm() < 1e-6f)
            {
                continue; // Kollineare Punkte
            }

            normal.normalize();
            float d = -normal.dot(p1);

            // Sanity Check: LiDAR-Bodenplatte zeigt im Fahrzeug-Koordinatensystem vorwiegend nach oben/unten (Z-Achse)
            if (std::abs(normal.z()) < 0.65f)
            {
                continue;
            }

            std::unordered_set<size_t> current_inliers;
            current_inliers.reserve(n_points / 2);

            for (size_t i = 0; i < n_points; ++i)
            {
                float dist_to_plane = std::abs(normal.dot(input[i].toEigen()) + d);
                if (dist_to_plane <= distance_threshold_)
                {
                    current_inliers.insert(i);
                }
            }

            if (current_inliers.size() > best_inliers.size())
            {
                best_inliers = std::move(current_inliers);
                best_plane.normal = normal;
                best_plane.d = d;
            }
        }

        // Punkte aufteilen und labeln
        ground_cloud->reserve(best_inliers.size());
        obstacle_cloud->reserve(n_points - best_inliers.size());

        for (size_t i = 0; i < n_points; ++i)
        {
            Point3D pt = input[i];
            if (best_inliers.contains(i))
            {
                pt.label = 1; // 1 = Ground
                ground_cloud->addPoint(pt);
            }
            else
            {
                pt.label = 2; // 2 = Obstacle candidate
                obstacle_cloud->addPoint(pt);
            }
        }

        return {ground_cloud, obstacle_cloud, best_plane};
    }

} // namespace lidar