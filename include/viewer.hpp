#pragma once

#include "point_cloud.hpp"
#include "cluster_extraction.hpp"
#include <vector>
#include <Eigen/Geometry>

struct GLFWwindow;

namespace lidar
{

    class PointCloudViewer
    {
    public:
        PointCloudViewer(const std::string &title = "3D LiDAR Point Cloud Visualizer",
                         int width = 1200, int height = 800);
        ~PointCloudViewer();

        bool init();
        void run(const PointCloud &ground,
                 const std::vector<PointCloud::Ptr> &clusters,
                 const std::vector<BoundingBox3D> &bboxes);

    private:
        static void mouseButtonCallback(GLFWwindow *window, int button, int action, int mods);
        static void cursorPosCallback(GLFWwindow *window, double xpos, double ypos);
        static void scrollCallback(GLFWwindow *window, double xoffset, double yoffset);

        std::string title_;
        int width_;
        int height_;
        GLFWwindow *window_{nullptr};

        // Orbit-Kamerastatus
        static inline float cam_yaw{45.0f};
        static inline float cam_pitch{25.0f};
        static inline float cam_distance{25.0f};
        static inline double last_x{0.0};
        static inline double last_y{0.0};
        static inline bool mouse_dragging{false};
    };

} // namespace lidar