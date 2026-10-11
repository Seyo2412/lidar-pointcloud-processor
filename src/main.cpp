#include "point_cloud.hpp"
#include "synthetic_generator.hpp"
#include "voxel_grid.hpp"
#include "ransac_plane.hpp"
#include "cluster_extraction.hpp"
#include "viewer.hpp"
#include <iostream>
#include <chrono>

int main()
{
    std::cout << "===========================================\n";
    std::cout << "   3D LiDAR Point Cloud Processor (C++20)  \n";
    std::cout << "===========================================\n\n";

    // 1. Synthetische LiDAR-Szene erzeugen
    std::cout << "[1] Generiere Test-Szene (100.000 Punkte)...\n";
    auto raw_cloud = lidar::SyntheticSceneGenerator::generateUrbanStreet(100000);
    std::cout << "    Punkte: " << raw_cloud->size() << "\n\n";

    // 2. Voxel Grid Downsampling
    std::cout << "[2] Voxel Downsampling (Leaf Size: 0.20 m)...\n";
    lidar::VoxelGridFilter voxel_filter(0.20f);
    auto t0 = std::chrono::high_resolution_clock::now();
    auto downsampled = voxel_filter.filter(*raw_cloud);
    auto t1 = std::chrono::high_resolution_clock::now();
    auto voxel_time = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    std::cout << "    Reduziert auf : " << downsampled->size() << " Punkte ("
              << (voxel_time / 1000.0f) << " ms)\n\n";

    // 3. RANSAC Ground Segmentation
    std::cout << "[3] RANSAC Ground Segmentation...\n";
    lidar::RansacPlaneSegmenter ransac(100, 0.15f);
    t0 = std::chrono::high_resolution_clock::now();
    auto [ground, obstacles, plane] = ransac.segment(*downsampled);
    t1 = std::chrono::high_resolution_clock::now();
    auto ransac_time = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    std::cout << "    Fahrbahn-Punkte  : " << ground->size() << "\n";
    std::cout << "    Hindernis-Punkte : " << obstacles->size() << " ("
              << (ransac_time / 1000.0f) << " ms)\n\n";

    // 4. Euclidean Clustering
    std::cout << "[4] Euclidean Cluster Extraction...\n";
    lidar::EuclideanClusterExtractor extractor(0.75f, 40, 10000);
    t0 = std::chrono::high_resolution_clock::now();
    auto [clusters, bboxes] = extractor.extract(*obstacles);
    t1 = std::chrono::high_resolution_clock::now();
    auto cluster_time = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    std::cout << "    Gefundene Hindernisse: " << clusters.size() << " ("
              << (cluster_time / 1000.0f) << " ms)\n\n";

    std::cout << "[5] Starte interaktiven 3D OpenGL Viewer...\n";
    std::cout << "    Steuerung: Linksklick + Ziehen = Rotieren | Scrollrad = Zoom\n";

    lidar::PointCloudViewer viewer("3D LiDAR Processor - Ground & Object Detection", 1280, 720);
    if (viewer.init())
    {
        viewer.run(*ground, clusters, bboxes);
    }

    return 0;
}