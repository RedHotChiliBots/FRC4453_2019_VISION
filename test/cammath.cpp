#define _USE_MATH_DEFINES // Needed on MSVC for M_PI.
#include <cmath>
#include "cammath.hpp"
#include "test.hpp"
#include <iostream>

// Converts radians to degrees.
constexpr double rad2deg(double v) {
    return v * (180.0 / M_PI);
}

// Converts degrees to radians.
constexpr double deg2rad(double v) {
    return v / (180.0 / M_PI);
}

TEST(camera_raycast_screenspace) {
    camera3<double> c(Eigen::Vector2<double>(3, 3), 90, Eigen::Vector3<double>(0,0,0), Eigen::Quaterniond::Identity());
    {
        auto r = c.cast_ray_screenspace(Eigen::Vector2<double>(1, 1));
        TEST_ASSERT(r.origin().x() == 0 && r.origin().y() == 0 && r.origin().z() == 0, "Ray pos incorrect for center");
        TEST_ASSERT(std::abs(r.direction().x()) < 0.1 && std::abs(r.direction().y() - 1.0) < 0.1 && std::abs(r.direction().z()) < 0.1, "Ray dir incorrect for center");
    }
    {
        auto r = c.cast_ray_screenspace(Eigen::Vector2<double>(0, 1));
        TEST_ASSERT(r.origin().x() == 0 && r.origin().y() == 0 && r.origin().z() == 0, "Ray pos incorrect for left");
        TEST_ASSERT(r.direction().x() < -0.01 && std::abs(r.direction().z()) < 0.01 && r.direction().y() > 0 , "Ray dir incorrect for left");
    }

    {
        auto r = c.cast_ray_screenspace(Eigen::Vector2<double>(2, 1));
        TEST_ASSERT(r.origin().x() == 0 && r.origin().y() == 0 && r.origin().z() == 0, "Ray pos incorrect for right");
        TEST_ASSERT(r.direction().x() > 0.01 && std::abs(r.direction().z()) < 0.01 && r.direction().y() > 0, "Ray dir incorrect for right");
    }

    {
        auto r = c.cast_ray_screenspace(Eigen::Vector2<double>(1, 0));
        TEST_ASSERT(r.origin().x() == 0 && r.origin().y() == 0 && r.origin().z() == 0, "Ray pos incorrect for up");
        TEST_ASSERT(r.direction().z() > 0.01 && std::abs(r.direction().x()) < 0.01 && r.direction().y() > 0, "Ray dir incorrect for up");
    }

    {
        auto r = c.cast_ray_screenspace(Eigen::Vector2<double>(1, 2));
        TEST_ASSERT(r.origin().x() == 0 && r.origin().y() == 0 && r.origin().z() == 0, "Ray pos incorrect for down");
        TEST_ASSERT(r.direction().z() < -0.01 && std::abs(r.direction().x()) < 0.01 && r.direction().y() > 0, "Ray dir incorrect for down");
    }

    TEST_PASS
}

TEST(camera_raycast_simple) {
    camera3<double> c(Eigen::Vector2<double>(3, 3), 90, Eigen::Vector3<double>(0,0,0), Eigen::Quaterniond::Identity());
    {
        auto r = c.cast_ray(Eigen::Vector2<double>(1, 1));
        TEST_ASSERT(r.origin().x() == 0 && r.origin().y() == 0 && r.origin().z() == 0, "Ray pos incorrect for center");
        TEST_ASSERT(std::abs(r.direction().x()) < 0.1 && std::abs(r.direction().y() - 1.0) < 0.1 && std::abs(r.direction().z()) < 0.1, "Ray dir incorrect for center");
    }
    {
        auto r = c.cast_ray(Eigen::Vector2<double>(0, 1));
        TEST_ASSERT(r.origin().x() == 0 && r.origin().y() == 0 && r.origin().z() == 0, "Ray pos incorrect for left");
        TEST_ASSERT(r.direction().x() < -0.01 && std::abs(r.direction().z()) < 0.01 && r.direction().y() > 0 , "Ray dir incorrect for left");
    }

    {
        auto r = c.cast_ray(Eigen::Vector2<double>(2, 1));
        TEST_ASSERT(r.origin().x() == 0 && r.origin().y() == 0 && r.origin().z() == 0, "Ray pos incorrect for right");
        TEST_ASSERT(r.direction().x() > 0.01 && std::abs(r.direction().z()) < 0.01 && r.direction().y() > 0, "Ray dir incorrect for right");
    }

    {
        auto r = c.cast_ray(Eigen::Vector2<double>(1, 0));
        TEST_ASSERT(r.origin().x() == 0 && r.origin().y() == 0 && r.origin().z() == 0, "Ray pos incorrect for up");
        TEST_ASSERT(r.direction().z() > 0.01 && std::abs(r.direction().x()) < 0.01 && r.direction().y() > 0, "Ray dir incorrect for up");
    }

    {
        auto r = c.cast_ray(Eigen::Vector2<double>(1, 2));
        TEST_ASSERT(r.origin().x() == 0 && r.origin().y() == 0 && r.origin().z() == 0, "Ray pos incorrect for down");
        TEST_ASSERT(r.direction().z() < -0.01 && std::abs(r.direction().x()) < 0.01 && r.direction().y() > 0, "Ray dir incorrect for down");
    }

    TEST_PASS
}

TEST(camera_raycast_rotation) {
    camera3<double> c(Eigen::Vector2<double>(3, 3), 90, Eigen::Vector3<double>(0,0,0), Eigen::Quaterniond(Eigen::AngleAxis<double>(deg2rad(-90), Eigen::Vector3d::UnitX())));
    {
        auto r = c.cast_ray(Eigen::Vector2<double>(1, 1));
        TEST_ASSERT(r.origin().x() == 0 && r.origin().y() == 0 && r.origin().z() == 0, "Ray pos incorrect for center");
        TEST_ASSERT(std::abs(r.direction().x()) < 0.1 && std::abs(r.direction().y()) < 0.1 && std::abs(r.direction().z() + 1.0) < 0.1, "Ray dir incorrect for center");
    }
    {
        auto r = c.cast_ray(Eigen::Vector2<double>(0, 1));
        std::cout << r.direction() << std::endl;
        TEST_ASSERT(r.origin().x() == 0 && r.origin().y() == 0 && r.origin().z() == 0, "Ray pos incorrect for left");
        TEST_ASSERT(r.direction().x() < -0.01 && std::abs(r.direction().y()) < 0.01 && r.direction().z() < 0 , "Ray dir incorrect for left");
    }

    {
        auto r = c.cast_ray(Eigen::Vector2<double>(2, 1));
        TEST_ASSERT(r.origin().x() == 0 && r.origin().y() == 0 && r.origin().z() == 0, "Ray pos incorrect for right");
        TEST_ASSERT(r.direction().x() > 0.01 && std::abs(r.direction().y()) < 0.01 && r.direction().z() < 0, "Ray dir incorrect for right");
    }

    {
        auto r = c.cast_ray(Eigen::Vector2<double>(1, 0));
        TEST_ASSERT(r.origin().x() == 0 && r.origin().y() == 0 && r.origin().z() == 0, "Ray pos incorrect for up");
        TEST_ASSERT(r.direction().y() > 0.01 && std::abs(r.direction().x()) < 0.01 && r.direction().z() < 0, "Ray dir incorrect for up");
    }

    {
        auto r = c.cast_ray(Eigen::Vector2<double>(1, 2));
        TEST_ASSERT(r.origin().x() == 0 && r.origin().y() == 0 && r.origin().z() == 0, "Ray pos incorrect for down");
        TEST_ASSERT(r.direction().y() < -0.01 && std::abs(r.direction().x()) < 0.01 && r.direction().z() < 0, "Ray dir incorrect for down");
    }

    TEST_PASS
}


START_TESTSUITE_REGISTRY(cammath)
REGISTER_TEST(camera_raycast_simple)
REGISTER_TEST(camera_raycast_rotation)
REGISTER_TEST(camera_raycast_screenspace)
END_TESTSUITE_REGISTRY