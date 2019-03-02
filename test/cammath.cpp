#include "../geom.hpp"
#include "../cammath.hpp"
#include "test.hpp"
#include <iostream>
TEST(camera_raycast_screenspace) {
    camera3<double> c(vector2<double>(3, 3), 90, vector3<double>(0,0,0), vector3<double>(0,0,0));
    {
        auto r = c.cast_ray_screenspace(vector2<double>(1, 1));
        TEST_ASSERT(r.position.x == 0 && r.position.y == 0 && r.position.z == 0, "Ray pos incorrect for center");
        TEST_ASSERT(std::abs(r.direction.x) < 0.1 && std::abs(r.direction.y - 1.0) < 0.1 && std::abs(r.direction.z) < 0.1, "Ray dir incorrect for center");
    }
    {
        auto r = c.cast_ray_screenspace(vector2<double>(0, 1));
        TEST_ASSERT(r.position.x == 0 && r.position.y == 0 && r.position.z == 0, "Ray pos incorrect for left");
        TEST_ASSERT(r.direction.x < -0.01 && std::abs(r.direction.z) < 0.01 && r.direction.y > 0 , "Ray dir incorrect for left");
    }

    {
        auto r = c.cast_ray_screenspace(vector2<double>(2, 1));
        TEST_ASSERT(r.position.x == 0 && r.position.y == 0 && r.position.z == 0, "Ray pos incorrect for right");
        TEST_ASSERT(r.direction.x > 0.01 && std::abs(r.direction.z) < 0.01 && r.direction.y > 0, "Ray dir incorrect for right");
    }

    {
        auto r = c.cast_ray_screenspace(vector2<double>(1, 0));
        TEST_ASSERT(r.position.x == 0 && r.position.y == 0 && r.position.z == 0, "Ray pos incorrect for up");
        TEST_ASSERT(r.direction.z > 0.01 && std::abs(r.direction.x) < 0.01 && r.direction.y > 0, "Ray dir incorrect for up");
    }

    {
        auto r = c.cast_ray_screenspace(vector2<double>(1, 2));
        TEST_ASSERT(r.position.x == 0 && r.position.y == 0 && r.position.z == 0, "Ray pos incorrect for down");
        TEST_ASSERT(r.direction.z < -0.01 && std::abs(r.direction.x) < 0.01 && r.direction.y > 0, "Ray dir incorrect for down");
    }

    TEST_PASS
}

TEST(camera_raycast_simple) {
    camera3<double> c(vector2<double>(3, 3), 90, vector3<double>(0,0,0), vector3<double>(0,0,0));
    {
        auto r = c.cast_ray(vector2<double>(1, 1));
        TEST_ASSERT(r.position.x == 0 && r.position.y == 0 && r.position.z == 0, "Ray pos incorrect for center");
        TEST_ASSERT(std::abs(r.direction.x) < 0.1 && std::abs(r.direction.y - 1.0) < 0.1 && std::abs(r.direction.z) < 0.1, "Ray dir incorrect for center");
    }
    {
        auto r = c.cast_ray(vector2<double>(0, 1));
        TEST_ASSERT(r.position.x == 0 && r.position.y == 0 && r.position.z == 0, "Ray pos incorrect for left");
        TEST_ASSERT(r.direction.x < -0.01 && std::abs(r.direction.z) < 0.01 && r.direction.y > 0 , "Ray dir incorrect for left");
    }

    {
        auto r = c.cast_ray(vector2<double>(2, 1));
        TEST_ASSERT(r.position.x == 0 && r.position.y == 0 && r.position.z == 0, "Ray pos incorrect for right");
        TEST_ASSERT(r.direction.x > 0.01 && std::abs(r.direction.z) < 0.01 && r.direction.y > 0, "Ray dir incorrect for right");
    }

    {
        auto r = c.cast_ray(vector2<double>(1, 0));
        TEST_ASSERT(r.position.x == 0 && r.position.y == 0 && r.position.z == 0, "Ray pos incorrect for up");
        TEST_ASSERT(r.direction.z > 0.01 && std::abs(r.direction.x) < 0.01 && r.direction.y > 0, "Ray dir incorrect for up");
    }

    {
        auto r = c.cast_ray(vector2<double>(1, 2));
        TEST_ASSERT(r.position.x == 0 && r.position.y == 0 && r.position.z == 0, "Ray pos incorrect for down");
        TEST_ASSERT(r.direction.z < -0.01 && std::abs(r.direction.x) < 0.01 && r.direction.y > 0, "Ray dir incorrect for down");
    }

    TEST_PASS
}

TEST(camera_raycast_rotation) {
    camera3<double> c(vector2<double>(3, 3), 90, vector3<double>(0,0,0), vector3<double>(deg2rad(-90.0),0,0));
    {
        auto r = c.cast_ray(vector2<double>(1, 1));
        TEST_ASSERT(r.position.x == 0 && r.position.y == 0 && r.position.z == 0, "Ray pos incorrect for center");
        TEST_ASSERT(std::abs(r.direction.x) < 0.1 && std::abs(r.direction.y) < 0.1 && std::abs(r.direction.z + 1.0) < 0.1, "Ray dir incorrect for center");
    }
    {
        auto r = c.cast_ray(vector2<double>(0, 1));
        std::cout << "R: (" << r.position.x << ", " << r.position.y << ", " << r.position.z << "), (" << r.direction.x << ", " << r.direction.y << ", " << r.direction.z << ")" << std::endl;
        TEST_ASSERT(r.position.x == 0 && r.position.y == 0 && r.position.z == 0, "Ray pos incorrect for left");
        TEST_ASSERT(r.direction.x < -0.01 && std::abs(r.direction.y) < 0.01 && r.direction.z > 0 , "Ray dir incorrect for left");
    }

    {
        auto r = c.cast_ray(vector2<double>(2, 1));
        TEST_ASSERT(r.position.x == 0 && r.position.y == 0 && r.position.z == 0, "Ray pos incorrect for right");
        TEST_ASSERT(r.direction.x > 0.01 && std::abs(r.direction.y) < 0.01 && r.direction.z > 0, "Ray dir incorrect for right");
    }

    {
        auto r = c.cast_ray(vector2<double>(1, 0));
        TEST_ASSERT(r.position.x == 0 && r.position.y == 0 && r.position.z == 0, "Ray pos incorrect for up");
        TEST_ASSERT(r.direction.y > 0.01 && std::abs(r.direction.x) < 0.01 && r.direction.z > 0, "Ray dir incorrect for up");
    }

    {
        auto r = c.cast_ray(vector2<double>(1, 2));
        TEST_ASSERT(r.position.x == 0 && r.position.y == 0 && r.position.z == 0, "Ray pos incorrect for down");
        TEST_ASSERT(r.direction.y < -0.01 && std::abs(r.direction.x) < 0.01 && r.direction.z > 0, "Ray dir incorrect for down");
    }

    TEST_PASS
}


START_TESTSUITE_REGISTRY(cammath)
REGISTER_TEST(camera_raycast_simple)
REGISTER_TEST(camera_raycast_rotation)
REGISTER_TEST(camera_raycast_screenspace)
END_TESTSUITE_REGISTRY