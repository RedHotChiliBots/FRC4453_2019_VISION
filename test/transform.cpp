#include "../geom.hpp"
#include "../cammath.hpp"
#define TESTING
#include "../transform.hpp"
#include "test.hpp"

#include <random>
#include <iostream>

TEST(transform_stability) {
    const camera3<double> cam(vector2<double>(1000.0, 1000.0), deg2rad(60.0), vector3<double>(0.0, 0.0, 0.0), vector3<double>(deg2rad(-40), 0.0, 0.0));
    constexpr plane3<double> floor(vector3<double>(0.0, 0.0, -100.0), vector3<double>(0.0, 0.0, -1.0).normalize()); 

    std::uniform_real_distribution<double> unif(0.1, 999.9);
    std::default_random_engine re;

    for(size_t i = 0; i < 1000000; i++) {
        vector2<double> a = vector2<double>(std::floor(unif(re)), std::floor(unif(re)));
        vector2<double> b = vector2<double>(std::floor(unif(re)), std::floor(unif(re)));

        try{
            auto r = transform_vector(std::make_pair(a, b), cam, floor);
        } catch(...) {
            std::cout << "Failed at #" << i << " (" << a.x << ", " << a.y << "), (" << b.x << ", " << b.y << ")" << std::endl;
            TEST_ASSERT(false, "Transform failed.")
        }
    }

    TEST_PASS
}

START_TESTSUITE_REGISTRY(transform)
REGISTER_TEST(transform_stability)
END_TESTSUITE_REGISTRY