#define _USE_MATH_DEFINES
#include "../geom.hpp"
#include "test.hpp"
#include <cmath>
#include <random>
TEST(vector2_ctor) {
    { // Default constructor
        vector2<int> a;
        TEST_ASSERT(a.x == 0 && a.y == 0, "Default vector is not zero");
    }
    
    { // Element constructor
        vector2<int> a(1, 2);
        TEST_ASSERT(a.x == 1 && a.y == 2, "Component ctor incorrect");
    }
    
    { // Copy constructor
        vector2<int> a(3, 4);
        vector2<int> b(a);
        TEST_ASSERT(b.x == a.x && b.y == a.y, "Copied-constructed vector does not match source");
    }

    {
        vector2<int> a(4, 5);
        vector2<int> b(a);
        vector2<int> c(std::move(b));
        TEST_ASSERT(c.x == a.x && c.y == a.y, "Move-constructed vector does not match source");
    }

    TEST_PASS
}

TEST(vector3_ctor) {
    { // Default constructor
        vector3<int> a;
        TEST_ASSERT(a.x == 0 && a.y == 0, "Default vector is not zero");
    }
    
    { // Element constructor
        vector3<int> a(1, 2, 3);
        TEST_ASSERT(a.x == 1 && a.y == 2 && a.z == 3, "Component ctor incorrect");
    }
    
    { // Copy constructor
        vector3<int> a(4, 5, 6);
        vector3<int> b(a);
        TEST_ASSERT(b.x == a.x && b.y == a.y && b.z == a.z, "Copied-constructed vector does not match source");
    }

    {
        vector3<int> a(7, 8, 9);
        vector3<int> b(a);
        vector3<int> c(std::move(b));
        TEST_ASSERT(c.x == a.x && c.y == a.y && c.z == a.z, "Move-constructed vector does not match source");
    }

    TEST_PASS
}

TEST(vector2_operators) {
    { // addition
        vector2<int> a(10, 30);
        vector2<int> b(20, 5);
        vector2<int> c = a + b;
        TEST_ASSERT(a.x + b.x == c.x && a.y + b.y == c.y, "Addition failed");

        a += b;
        TEST_ASSERT(a.x == c.x && a.y == c.y, "Addition-assignment failed");
    }
    { // subtraction
        vector2<int> a(30, 4);
        vector2<int> b(15, 6);
        vector2<int> c = a - b;
        TEST_ASSERT(a.x - b.x == c.x && a.y - b.y == c.y, "Subtraction failed");

        a -= b;
        TEST_ASSERT(a.x == c.x && a.y == c.y, "Subtraction-assignment failed");
    }
    { // scalar multiplication
        vector2<int> a(5, -3);
        vector2<int> c = a * 10;
        TEST_ASSERT(a.x * 10 == c.x && a.y * 10 == c.y, "Scalar multiplication failed");

        a *= 10;
        TEST_ASSERT(a.x == c.x && a.y == c.y, "Scalar multiplication-assignment failed");
    }
    { // scalar division
        vector2<int> a(400, -30);
        vector2<int> c = a / 10;
        TEST_ASSERT(a.x / 10 == c.x && a.y / 10 == c.y, "Scalar multiplication failed");

        a /= 10;
        TEST_ASSERT(a.x == c.x && a.y == c.y, "Scalar multiplication-assignment failed");
    }

    TEST_PASS
}

TEST(vector3_operators) {
    { // addition
        vector3<int> a(10, 30, 25);
        vector3<int> b(20, 5, 4);
        vector3<int> c = a + b;
        TEST_ASSERT(a.x + b.x == c.x && a.y + b.y == c.y, "Addition failed");

        a += b;
        TEST_ASSERT(a.x == c.x && a.y == c.y, "Addition-assignment failed");
    }
    { // subtraction
        vector3<int> a(30, 4, 23);
        vector3<int> b(15, 6, 76);
        vector3<int> c = a - b;
        TEST_ASSERT(a.x - b.x == c.x && a.y - b.y == c.y, "Subtraction failed");

        a -= b;
        TEST_ASSERT(a.x == c.x && a.y == c.y, "Subtraction-assignment failed");
    }
    { // scalar multiplication
        vector3<int> a(5, -3, 40);
        vector3<int> c = a * 10;
        TEST_ASSERT(a.x * 10 == c.x && a.y * 10 == c.y, "Scalar multiplication failed");

        a *= 10;
        TEST_ASSERT(a.x == c.x && a.y == c.y, "Scalar multiplication-assignment failed");
    }
    { // scalar division
        vector3<int> a(400, -30, 450);
        vector3<int> c = a / 10;
        TEST_ASSERT(a.x / 10 == c.x && a.y / 10 == c.y, "Scalar multiplication failed");

        a /= 10;
        TEST_ASSERT(a.x == c.x && a.y == c.y, "Scalar multiplication-assignment failed");
    }

    TEST_PASS
}

TEST(vector2_math) {
    { // theta
        vector2<double> v(0, 1);
        TEST_ASSERT(std::abs(v.theta() - M_PI_2) < 0.001, "Theta not working: case 0, 1");
        v = vector2<double>(1, 0);
        TEST_ASSERT(std::abs(v.theta() - 0) < 0.001, "Theta not working: case 1, 0");
        v = vector2<double>(0, -3);
        TEST_ASSERT(std::abs(v.theta() - -M_PI_2) < 0.001, "Theta not working: case 0, -3");
    }

    { // magnitude
        vector2<double> v(0, 1);
        TEST_ASSERT(std::abs(v.magnitude() - 1.0) < 0.001, "Magnitude not working: case 0, 1");
        v = vector2<double>(-1, 0);
        TEST_ASSERT(std::abs(v.magnitude() - 1.0) < 0.001, "Magnitude not working: case -1, 0");
        v = vector2<double>(3, 4);
        TEST_ASSERT(std::abs(v.magnitude() - 5.0) < 0.001, "Magnitude not working: case 3, 4");
    }

    TEST_PASS
}

TEST(vector3_math) {
    { // magnitude
        vector3<double> v(0, 1, 0);
        TEST_ASSERT(std::abs(v.magnitude() - 1.0) < 0.001, "Magnitude not working: case 0, 1, 0");
        v = vector3<double>(0, 0, -1);
        TEST_ASSERT(std::abs(v.magnitude() - 1.0) < 0.001, "Magnitude not working: case 0, 0, -1");
        v = vector3<double>(3, 4, 0);
        TEST_ASSERT(std::abs(v.magnitude() - 5.0) < 0.001, "Magnitude not working: case 3, 4, 0");
        v = vector3<double>(0, 4, 3);
        TEST_ASSERT(std::abs(v.magnitude() - 5.0) < 0.001, "Magnitude not working: case 0, 4, 3");
    }

    { // dot
        vector3<double> a(0, 1, 0), b(1, 0, 0);
        TEST_ASSERT(a.dot(b) < 0.001, "Dot failed for right angle");
        a = vector3<double>(1, 0, 0); b = vector3<double>(1, 0, 0);
        TEST_ASSERT(std::abs(a.dot(b) - 1) < 0.001, "Dot failed for parallel");
        a = vector3<double>(0, 0, 1); b = vector3<double>(0, 0, -1);
        TEST_ASSERT(std::abs(a.dot(b) - -1) < 0.001, "Dot failed for anti-parallel");
    }

    { // normalize
        vector3<double> v(23, -65, 3);
        v = v.normalize();
        TEST_ASSERT(std::abs(v.magnitude() - 1.0) < 0.001, "Normalize failed");

        double lower_bound = 0;
        double upper_bound = 10000;
        std::uniform_real_distribution<double> unif(lower_bound,upper_bound);
        std::default_random_engine re;

        for(size_t i = 0; i < 1000000; i++) {
            v.x = unif(re);
            v.y = unif(re);
            v.z = unif(re);
            v = v.normalize();
            TEST_ASSERT(std::abs(v.magnitude() - 1.0) < 0.001, "Normalize failed random case");
        }
    }

    { // truncate
        vector3<int> v(1, 2, 3);
        auto a = v.truncate();
        TEST_ASSERT(a.x == 1 && a.y == 2, "Truncate failed");
    }

    TEST_PASS
}

TEST(ray3_ctor) {
    { // element
        vector3<int> o(4, 2, 3), d(7, 1, 5);
        ray3<int> r(o, d);
        TEST_ASSERT(r.position.x == o.x && r.position.y == o.y && r.position.z == o.z, "Ray ctor failed: pos");
        TEST_ASSERT(r.direction.x == d.x && r.direction.y == d.y && r.direction.z == d.z, "Ray ctor failed: dir");
    }

    TEST_PASS
}

TEST(ray3_math) {
    { // point at dist
        ray3<int> r(vector3<int>(1, 1, 0), vector3<int>(-1, 0, 0));
        auto p = r.point_at_distance(4);
        TEST_ASSERT(p.x == -3 && p.y == 1 && p.z == 0, "Point at dist failed");
    }

    TEST_PASS
}

TEST(plane3_ctor) {
    { // element
        vector3<int> o(4, 2, 3), n(7, 1, 5);
        plane3<int> r(o, n);
        TEST_ASSERT(r.position.x == o.x && r.position.y == o.y && r.position.z == o.z, "Plane ctor failed: pos");
        TEST_ASSERT(r.normal.x == n.x && r.normal.y == n.y && r.normal.z == n.z, "Plane ctor failed: norm");
    }

    TEST_PASS
}

TEST(plane3_math) {
    { // intersect
        ray3<double> r(vector3<double>(1.0, 1.0, 0.0), vector3<double>(-1.0, 0.0, 0.0));
        
        plane3<double> p(vector3<double>(-5.0, 0.0, 0.0), vector3<double>(-1.0, 0.0, 0.0));

        auto i = p.ray_intersect(r).value();
        TEST_ASSERT(std::abs(i.x - -5) < 0.001 && std::abs(i.y - 1) < 0.001 && std::abs(i.z - 0) < 0.001, "Intersect failed");
    }

    { // intersect 2
        ray3<double> r(vector3<double>(0.0, 0.0, 0.0), vector3<double>(0.3, 0.1, -10.0).normalize());
        
        plane3<double> p(vector3<double>(0.0, 0.0, -10.0), vector3<double>(0.0, 0.0, -1.0));

        auto i = p.ray_intersect(r);
        TEST_ASSERT(i, "Intersect 2 failed");
    }

    TEST_PASS
}

TEST(mat4_ctor) {
    matrix4<int> m;
    for(size_t x = 0; x < 4; x++) {
        for(size_t y = 0; y < 4; y++) {
            TEST_ASSERT(m.data[x][y] == 0, "Ctor failed");
        }
    }

    TEST_PASS
}


START_TESTSUITE_REGISTRY(geom)
REGISTER_TEST(vector2_ctor)
REGISTER_TEST(vector3_ctor)
REGISTER_TEST(vector2_operators)
REGISTER_TEST(vector3_operators)
REGISTER_TEST(vector2_math)
REGISTER_TEST(vector3_math)
REGISTER_TEST(ray3_ctor)
REGISTER_TEST(ray3_math)
REGISTER_TEST(plane3_ctor)
REGISTER_TEST(plane3_math)
REGISTER_TEST(mat4_ctor)
END_TESTSUITE_REGISTRY
