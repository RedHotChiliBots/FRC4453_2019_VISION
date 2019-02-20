#include "../geom.hpp"
#include "test.hpp"

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

START_TESTSUITE_REGISTRY(geom)
REGISTER_TEST(vector2_ctor)
REGISTER_TEST(vector3_ctor)
REGISTER_TEST(vector2_operators)
REGISTER_TEST(vector3_operators)
END_TESTSUITE_REGISTRY
