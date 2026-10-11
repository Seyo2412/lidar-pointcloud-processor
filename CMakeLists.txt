cmake_minimum_required(VERSION 3.20)
project(lidar_pointcloud_processor VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native")

list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")

find_package(Eigen3 REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

# Core Library
add_library(pointcloud_core
    src/point_types.cpp
    src/point_cloud.cpp
    src/synthetic_generator.cpp
    src/voxel_grid.cpp
    src/ransac_plane.cpp
    src/cluster_extraction.cpp
    src/viewer.cpp
)

target_include_directories(pointcloud_core PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    /opt/homebrew/include
    /opt/homebrew/include/eigen3
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(pointcloud_core PUBLIC 
    Eigen3::Eigen
    glfw
    OpenGL::GL
)

# Hauptprogramm
add_executable(lidar_processor src/main.cpp)
target_link_libraries(lidar_processor PRIVATE pointcloud_core)

# Tests
enable_testing()
add_executable(run_tests tests/test_pipeline.cpp)
target_link_libraries(run_tests PRIVATE pointcloud_core)
add_test(NAME PipelineValidation COMMAND run_tests)