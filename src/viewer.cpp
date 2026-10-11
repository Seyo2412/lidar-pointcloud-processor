#define GL_SILENCE_DEPRECATION 1

#include "viewer.hpp"
#include <GLFW/glfw3.h>
#include <OpenGL/gl.h>
#include <iostream>
#include <cmath>

namespace lidar
{

    PointCloudViewer::PointCloudViewer(const std::string &title, int width, int height)
        : title_(title), width_(width), height_(height) {}

    PointCloudViewer::~PointCloudViewer()
    {
        if (window_)
        {
            glfwDestroyWindow(window_);
        }
        glfwTerminate();
    }

    void PointCloudViewer::mouseButtonCallback(GLFWwindow *, int button, int action, int)
    {
        if (button == GLFW_MOUSE_BUTTON_LEFT)
        {
            mouse_dragging = (action == GLFW_PRESS);
        }
    }

    void PointCloudViewer::cursorPosCallback(GLFWwindow *, double xpos, double ypos)
    {
        if (mouse_dragging)
        {
            float dx = static_cast<float>(xpos - last_x);
            float dy = static_cast<float>(ypos - last_y);
            cam_yaw += dx * 0.4f;
            cam_pitch = std::max(-85.0f, std::min(85.0f, cam_pitch + dy * 0.4f));
        }
        last_x = xpos;
        last_y = ypos;
    }

    void PointCloudViewer::scrollCallback(GLFWwindow *, double, double yoffset)
    {
        cam_distance = std::max(2.0f, std::min(150.0f, cam_distance - static_cast<float>(yoffset) * 2.0f));
    }

    bool PointCloudViewer::init()
    {
        glfwSetErrorCallback([](int error, const char *description)
                             { std::cerr << "[GLFW ERROR " << error << "]: " << description << "\n"; });

        if (!glfwInit())
        {
            std::cerr << "[FATAL] GLFW konnte nicht initialisiert werden.\n";
            return false;
        }

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
        glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
        glfwWindowHint(GLFW_FOCUSED, GLFW_TRUE);
        glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GLFW_TRUE);

        window_ = glfwCreateWindow(width_, height_, title_.c_str(), nullptr, nullptr);
        if (!window_)
        {
            std::cerr << "[FATAL] glfwCreateWindow fehlgeschlagen!\n";
            glfwTerminate();
            return false;
        }

        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);

        glfwSetMouseButtonCallback(window_, mouseButtonCallback);
        glfwSetCursorPosCallback(window_, cursorPosCallback);
        glfwSetScrollCallback(window_, scrollCallback);

        glEnable(GL_DEPTH_TEST);
        glPointSize(3.5f);

        std::cout << "[INFO] OpenGL Renderer: " << glGetString(GL_RENDERER) << "\n";
        return true;
    }

    void PointCloudViewer::run(const PointCloud &ground,
                               const std::vector<PointCloud::Ptr> &clusters,
                               const std::vector<BoundingBox3D> &bboxes)
    {
        if (!window_)
            return;

        const std::vector<Eigen::Vector3f> cluster_colors = {
            {1.0f, 0.20f, 0.25f},  // Leuchtendes Rot
            {0.20f, 0.95f, 0.35f}, // Hellgrün
            {1.0f, 0.82f, 0.10f},  // Warmes Gelb
            {0.15f, 0.75f, 1.0f},  // Cyan
            {0.95f, 0.35f, 0.95f}  // Magenta
        };

        while (!glfwWindowShouldClose(window_))
        {
            int fb_w, fb_h;
            glfwGetFramebufferSize(window_, &fb_w, &fb_h);
            glViewport(0, 0, fb_w, fb_h);

            glClearColor(0.06f, 0.07f, 0.09f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // Projektionsmatrix
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            float aspect = static_cast<float>(fb_w) / static_cast<float>(fb_h > 0 ? fb_h : 1);

            float fov = 45.0f * (3.14159265f / 180.0f);
            float near_z = 0.5f, far_z = 300.0f;
            float top = near_z * std::tan(fov * 0.5f);
            float right = top * aspect;
            glFrustum(-right, right, -top, top, near_z, far_z);

            // ModelView / Kamera
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();

            float rad_pitch = cam_pitch * (3.14159265f / 180.0f);
            float rad_yaw = cam_yaw * (3.14159265f / 180.0f);

            float eye_x = cam_distance * std::cos(rad_pitch) * std::sin(rad_yaw);
            float eye_y = -cam_distance * std::cos(rad_pitch) * std::cos(rad_yaw);
            float eye_z = cam_distance * std::sin(rad_pitch);

            Eigen::Vector3f target(0.0f, 8.0f, -1.0f);
            Eigen::Vector3f eye = target + Eigen::Vector3f(eye_x, eye_y, eye_z);

            Eigen::Vector3f forward = (target - eye).normalized();
            Eigen::Vector3f up(0.0f, 0.0f, 1.0f);
            Eigen::Vector3f side = forward.cross(up).normalized();
            Eigen::Vector3f up_cam = side.cross(forward).normalized();

            float m[16] = {
                side.x(), up_cam.x(), -forward.x(), 0.0f,
                side.y(), up_cam.y(), -forward.y(), 0.0f,
                side.z(), up_cam.z(), -forward.z(), 0.0f,
                -side.dot(eye), -up_cam.dot(eye), forward.dot(eye), 1.0f};
            glLoadMatrixf(m);

            // 1. Dezentes Sensor-Referenzgitter am Boden (z = -1.62m)
            glLineWidth(1.0f);
            glBegin(GL_LINES);
            glColor3f(0.12f, 0.15f, 0.20f);
            for (float gx = -12.0f; gx <= 12.0f; gx += 2.0f)
            {
                glVertex3f(gx, -25.0f, -1.62f);
                glVertex3f(gx, 35.0f, -1.62f);
            }
            for (float gy = -25.0f; gy <= 35.0f; gy += 2.0f)
            {
                glVertex3f(-12.0f, gy, -1.62f);
                glVertex3f(12.0f, gy, -1.62f);
            }
            glEnd();

            // 2. Fahrbahn-Punkte
            glBegin(GL_POINTS);
            glColor3f(0.18f, 0.35f, 0.50f);
            for (const auto &pt : ground.points())
            {
                glVertex3f(pt.x, pt.y, pt.z);
            }

            // 3. Cluster-Punkte
            for (size_t c = 0; c < clusters.size(); ++c)
            {
                const auto &col = cluster_colors[c % cluster_colors.size()];
                glColor3f(col.x(), col.y(), col.z());
                for (const auto &pt : clusters[c]->points())
                {
                    glVertex3f(pt.x, pt.y, pt.z);
                }
            }
            glEnd();

            // 4. Highlightete 3D Bounding Boxes
            glLineWidth(2.5f);
            for (size_t b = 0; b < bboxes.size(); ++b)
            {
                const auto &box = bboxes[b];
                const auto &col = cluster_colors[b % cluster_colors.size()];
                glColor3f(std::min(col.x() * 1.2f, 1.0f),
                          std::min(col.y() * 1.2f, 1.0f),
                          std::min(col.z() * 1.2f, 1.0f));

                float x0 = box.min_point.x(), x1 = box.max_point.x();
                float y0 = box.min_point.y(), y1 = box.max_point.y();
                float z0 = box.min_point.z(), z1 = box.max_point.z();

                glBegin(GL_LINES);
                // Untere Ebene
                glVertex3f(x0, y0, z0);
                glVertex3f(x1, y0, z0);
                glVertex3f(x1, y0, z0);
                glVertex3f(x1, y1, z0);
                glVertex3f(x1, y1, z0);
                glVertex3f(x0, y1, z0);
                glVertex3f(x0, y1, z0);
                glVertex3f(x0, y0, z0);
                // Obere Ebene
                glVertex3f(x0, y0, z1);
                glVertex3f(x1, y0, z1);
                glVertex3f(x1, y0, z1);
                glVertex3f(x1, y1, z1);
                glVertex3f(x1, y1, z1);
                glVertex3f(x0, y1, z1);
                glVertex3f(x0, y1, z1);
                glVertex3f(x0, y0, z1);
                // Vertikale Stützen
                glVertex3f(x0, y0, z0);
                glVertex3f(x0, y0, z1);
                glVertex3f(x1, y0, z0);
                glVertex3f(x1, y0, z1);
                glVertex3f(x1, y1, z0);
                glVertex3f(x1, y1, z1);
                glVertex3f(x0, y1, z0);
                glVertex3f(x0, y1, z1);
                glEnd();
            }

            glfwSwapBuffers(window_);
            glfwPollEvents();
        }
    }

} // namespace lidar