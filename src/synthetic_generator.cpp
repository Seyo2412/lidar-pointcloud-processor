#include "synthetic_generator.hpp"
#include <random>
#include <cmath>

namespace lidar
{

    PointCloud::Ptr SyntheticSceneGenerator::generateUrbanStreet(size_t target_points)
    {
        auto cloud = std::make_shared<PointCloud>();
        cloud->reserve(target_points);

        std::mt19937 gen(42);
        std::uniform_real_distribution<float> dis_x(-8.0f, 8.0f);
        std::uniform_real_distribution<float> dis_y(-25.0f, 35.0f);
        std::normal_distribution<float> noise(0.0f, 0.015f);

        // 1. Fahrbahn (70% der Punkte)
        size_t ground_count = static_cast<size_t>(target_points * 0.70);
        for (size_t i = 0; i < ground_count; ++i)
        {
            float x = dis_x(gen);
            float y = dis_y(gen);
            float z = -1.60f + noise(gen);
            cloud->addPoint(Point3D(x, y, z, 0.25f, 0));
        }

        // Hilfsfunktion: Solide Box (z. B. Fahrzeug) erzeugen
        auto add_box = [&](float cx, float cy, float cz, float sx, float sy, float sz, size_t count, uint32_t ring)
        {
            std::uniform_real_distribution<float> bx(-sx * 0.5f, sx * 0.5f);
            std::uniform_real_distribution<float> by(-sy * 0.5f, sy * 0.5f);
            std::uniform_real_distribution<float> bz(-sz * 0.5f, sz * 0.5f);
            for (size_t i = 0; i < count; ++i)
            {
                cloud->addPoint(Point3D(cx + bx(gen), cy + by(gen), cz + bz(gen), 0.85f, ring));
            }
        };

        // 2. Objekt 1: Vorausfahrendes Auto (Spur rechts, y = 12m)
        add_box(1.8f, 12.0f, -0.9f, 1.8f, 4.2f, 1.4f, static_cast<size_t>(target_points * 0.12), 1);

        // 3. Objekt 2: Entgegenkommender Transporter/SUV (Spur links, y = 20m)
        add_box(-2.2f, 20.0f, -0.7f, 2.0f, 4.8f, 1.8f, static_cast<size_t>(target_points * 0.10), 2);

        // 4. Objekt 3: Geparktes Auto am rechten Rand (y = 2m)
        add_box(4.5f, 2.0f, -0.9f, 1.8f, 3.8f, 1.3f, static_cast<size_t>(target_points * 0.05), 3);

        // 5. Randobjekte: Leitpfosten / Fußgänger
        auto add_cylinder = [&](float cx, float cy, float r, float h, size_t count)
        {
            std::uniform_real_distribution<float> ang(0.0f, 6.2831f);
            std::uniform_real_distribution<float> rad(0.0f, r);
            std::uniform_real_distribution<float> zh(-1.6f, -1.6f + h);
            for (size_t i = 0; i < count; ++i)
            {
                float a = ang(gen);
                float rd = rad(gen);
                cloud->addPoint(Point3D(cx + rd * std::cos(a), cy + rd * std::sin(a), zh(gen), 0.9f, 4));
            }
        };

        add_cylinder(-4.5f, 6.0f, 0.25f, 1.75f, static_cast<size_t>(target_points * 0.015)); // Fußgänger links
        add_cylinder(5.2f, 18.0f, 0.15f, 1.0f, static_cast<size_t>(target_points * 0.008));  // Pfosten rechts
        add_cylinder(-5.0f, -5.0f, 0.15f, 1.0f, static_cast<size_t>(target_points * 0.007)); // Pfosten hinten

        return cloud;
    }

} // namespace lidar