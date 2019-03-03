#define _USE_MATH_DEFINES // Needed on MSVC for M_PI.
#include <cmath>
#include "cammath.hpp"
#define TESTING
#include "transform.hpp"
#include "test.hpp"

#include <random>
#include <iostream>

// Converts radians to degrees.
constexpr double rad2deg(double v) {
    return v * (180.0 / M_PI);
}

// Converts degrees to radians.
constexpr double deg2rad(double v) {
    return v / (180.0 / M_PI);
}

TEST(transform_stability) {
    const camera3<double> cam(Eigen::Vector2<double>(1000.0, 1000.0), deg2rad(60.0), Eigen::Vector3<double>(0.0, 0.0, 0.0), Eigen::Quaterniond(Eigen::AngleAxis<double>(-40, Eigen::Vector3d::UnitX())));
    const Eigen::Hyperplane<double, 3> floor(Eigen::Vector3<double>(0.0, 0.0, -1.0).normalized(), Eigen::Vector3<double>(0.0, 0.0, -100.0)); 

    std::uniform_real_distribution<double> unif(0.1, 999.9);
    std::default_random_engine re;

    for(size_t i = 0; i < 10000; i++) {
        Eigen::Vector2<double> a(std::floor(unif(re)), std::floor(unif(re)));
        Eigen::Vector2<double> b(std::floor(unif(re)), std::floor(unif(re)));

        try {
            auto r = transform_vector(std::make_pair(a, b), cam, floor);
        } catch(...) {
            std::cout << "Failed at #" << i << " (" << a.x() << ", " << a.y() << "), (" << b.x() << ", " << b.y() << ")" << std::endl;
            TEST_ASSERT(false, "Transform failed.")
        }
    }

    TEST_PASS
}

START_TESTSUITE_REGISTRY(transform)
REGISTER_TEST(transform_stability)
END_TESTSUITE_REGISTRY