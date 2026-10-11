#include "point_cloud.hpp"
#include "synthetic_generator.hpp"
#include "voxel_grid.hpp"
#include "ransac_plane.hpp"
#include "cluster_extraction.hpp"
#include <cassert>
#include <iostream>

void test_synthetic_generation() {
    auto cloud = lidar::SyntheticSceneGenerator::generateUrbanStreet(10000);
    assert(cloud->size() > 5000 && "Punktwolke muss Punkte enthalten");
    std::cout << "[PASS] Synthetic Scene Generation\n";
}

void test_voxel_downsampling() {
    auto cloud = lidar::SyntheticSceneGenerator::generateUrbanStreet(20000);
    lidar::VoxelGridFilter filter(0.5f);
    auto downsampled = filter.filter(*cloud);
    assert(downsampled->size() < cloud->size() && "Downsampling muss Datenmenge reduzieren");
    assert(!downsampled->empty() && "Downsampled Wolke darf nicht leer sein");
    std::cout << "[PASS] Voxel Grid Downsampling\n";
}

void test_ransac_ground() {
    auto cloud = lidar::SyntheticSceneGenerator::generateUrbanStreet(20000);
    lidar::VoxelGridFilter filter(0.3f);
    auto downsampled = filter.filter(*cloud);

    lidar::RansacPlaneSegmenter ransac(50, 0.2f);
    auto [ground, obstacles, plane] = ransac.segment(*downsampled);

    assert(ground->size() > 0 && "Bodenpunkte muessen erkannt werden");
    assert(obstacles->size() > 0 && "Hindernisse muessen erkannt werden");
    assert(std::abs(plane.normal.z()) > 0.5f && "Boden-Normale muss vertikal orientiert sein");
    std::cout << "[PASS] RANSAC Ground Segmentation\n";
}

void test_clustering() {
    auto cloud = lidar::SyntheticSceneGenerator::generateUrbanStreet(20000);
    lidar::VoxelGridFilter filter(0.3f);
    auto downsampled = filter.filter(*cloud);

    lidar::RansacPlaneSegmenter ransac(50, 0.2f);
    auto [ground, obstacles, plane] = ransac.segment(*downsampled);

    lidar::EuclideanClusterExtractor extractor(0.75f, 20, 10000);
    auto [clusters, bboxes] = extractor.extract(*obstacles);

    assert(!clusters.empty() && "Mindestens ein Hindernis muss geclustert werden");
    assert(clusters.size() == bboxes.size() && "Jedes Cluster braucht eine Bounding-Box");
    std::cout << "[PASS] Euclidean Cluster Extraction\n";
}

int main() {
    std::cout << "--- Running Pipeline Unit Tests ---\n";
    test_synthetic_generation();
    test_voxel_downsampling();
    test_ransac_ground();
    test_clustering();
    std::cout << "--- All Tests Passed Successfully ---\n";
    return 0;
}