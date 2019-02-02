#ifndef GEOM_H
#define GEOM_H
#include <gcem.hpp>
#include <optional>
#include <cstring>
#include <cmath>

template<typename T>
struct vector2 {
    T x, y;

    constexpr vector2() : x(0), y(0) {}

    template<typename U>
    constexpr vector2(vector2<U>& other) : x(other.x), y(other.y) {}
    template<typename U>
    constexpr vector2(vector2<U>&& other) : x(std::move(other.x)), y(std::move(other.y)) {}
    constexpr vector2(const T& x_i, const T& y_i) : x(x_i), y(y_i) {}
    constexpr vector2(T&& x_i, T&& y_i) : x(std::move(x_i)), y(std::move(y_i)) {}

    T theta() {
        return std::atan2(y, x);
    }

    constexpr T magnitude() {
        return gcem::sqrt(x*x + y*y);
    }

    vector2<T>& operator+=(const vector2<T>& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    vector2<T>& operator-=(const vector2<T>& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    vector2<T>& operator*=(const T& other) {
        x *= other;
        y *= other;
        return *this;
    }

    vector2<T>& operator/=(const T& other) {
        x /= other;
        y /= other;
        return *this;
    }

    constexpr vector2<T> operator+(const vector2<T>& b) {
        vector2<T> r(*this);
        r+=b;
        return r;
    }

    constexpr vector2<T> operator-(const vector2<T>& b) {
        vector2<T> r(*this);
        r-=b;
        return r;
    }

    constexpr vector2<T> operator*(const T& b) {
        vector2<T> r(*this);
        r*=b;
        return r;
    }

    constexpr vector2<T> operator/(const T& b) {
        vector2<T> r(*this);
        r/=b;
        return r;
    }
};

template<typename T>
struct vector3 {
    T x, y, z;

    constexpr vector3() : x(0), y(0), z(0) {}

    template<typename U>
    constexpr vector3(vector3<U>& other) : x(other.x), y(other.y), z(other.z) {}
    template<typename U>
    constexpr vector3(vector3<U>&& other) : x(std::move(other.x)), y(std::move(other.y)), z(std::move(other.z)) {}
    constexpr vector3(const T& x_i, const T& y_i, const T& z_i) : x(x_i), y(y_i), z(z_i) {}
    constexpr vector3(T&& x_i, T&& y_i, T&& z_i) : x(std::move(x_i)), y(std::move(y_i)), z(std::move(z_i)) {}

    constexpr T magnitude() const {
        return gcem::sqrt(x*x + y*y + z*z);
    }

    constexpr vector3<T> normalize() const {
        T d = magnitude();
        return vector3<T>(x / d, y / d, z / d);
    }

    T dot(const vector3<T>& other) const {
        return (x*other.x)+(y*other.y)+(z*other.z);
    }

    constexpr vector2<T> truncate() const {
        return vector2<T>(x, y);
    }

    template<typename U>
    constexpr vector3<T>& operator+=(const vector3<U>& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    template<typename U>
    constexpr vector3<T>& operator-=(const vector3<U>& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    template<typename U>
    constexpr vector3<T>& operator*=(const U& other) {
        x *= other;
        y *= other;
        z *= other;
        return *this;
    }

    constexpr vector3<T>& operator/=(const T& other) {
        x /= other;
        y /= other;
        z /= other;
        return *this;
    }

    constexpr vector3<T> operator+(const vector3<T>& b) const {
        vector3<T> r(*this);
        r+=b;
        return r;
    }

    constexpr vector3<T> operator-(const vector3<T>& b) const {
        vector3<T> r(*this);
        r-=b;
        return r;
    }

    constexpr vector3<T> operator*(const T& b) const {
        vector3<T> r(*this);
        r*=b;
        return r;
    }

    constexpr vector3<T> operator/(const T& b) const {
        vector3<T> r(*this);
        r/=b;
        return r;
    }
};

template<typename T>
struct ray3 {
    vector3<T> position;
    vector3<T> direction;

    constexpr ray3(const vector3<T>& p, const vector3<T>& d) : position(p), direction(d) {}
    constexpr ray3(const vector3<T>&& p, const vector3<T>&& d) : position(std::move(p)), direction(std::move(d)) {}
    
    constexpr vector3<T> point_at_distance(T dist) const {
        return position + (direction * dist);
    }
};

template<typename T>
struct plane3 {
    vector3<T> position;
    vector3<T> normal;

    constexpr plane3(const vector3<T>& p, const vector3<T>& n) : position(p), normal(n) {}
    constexpr plane3(const vector3<T>&& p, const vector3<T>&& n) : position(std::move(p)), normal(std::move(n)) {}

    constexpr std::optional<T> ray_intersect_distance(const ray3<T>& ray) const{
        T den = normal.dot(ray.direction);
        if(den > 1e-6) {
            return (position - ray.position).dot(normal) / den;
        }
        return {};
    }

    constexpr std::optional<vector3<T>> ray_intersect(const ray3<T>& ray) const {
        auto dist = ray_intersect_distance(ray);
        if(dist) {
            return ray.point_at_distance(*dist);
        }
        return {};
    }
};

template<typename T>
struct matrix4 {
    T data[4][4];
    constexpr matrix4() : data{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}} {}

    constexpr static matrix4<T> rotation_x(T x) {
        matrix4<T> r;

        r.data[0][0] = r.data[3][3] = 1;
        r.data[1][1] = r.data[2][2] = gcem::cos(x);
        r.data[2][1] =  gcem::sin(x);
        r.data[1][2] = -gcem::sin(x);

        return r;
    }

    constexpr static matrix4<T> rotation_y(T y) {
        matrix4<T> r;

        r.data[1][1] = r.data[3][3] = 1;
        r.data[0][0] = r.data[2][2] = gcem::cos(y);
        r.data[0][2] =  gcem::sin(y);
        r.data[2][0] = -gcem::sin(y);

        return r;
    }

    constexpr static matrix4<T> rotation_z(T z) {
        matrix4<T> r;

        r.data[2][2] = r.data[3][3] = 1;
        r.data[0][0] = r.data[1][1] = gcem::cos(z);
        r.data[1][0] =  gcem::sin(z);
        r.data[0][1] = -gcem::sin(z);

        return r;
    }

    constexpr static matrix4<T> rotation(T x, T y, T z) {
        return rotation_x(x) * rotation_y(y) * rotation_z(z);
    }

    constexpr static matrix4<T> translation(T x, T y, T z) {
        matrix4<T> r;

        r.data[0][3] = x;
        r.data[1][3] = y;
        r.data[2][3] = z;

        r.data[0][0] = r.data[1][1] = r.data[2][2] = r.data[3][3] = 1;

        r.data[0][1] = r.data[0][2]  = r.data[1][0] = r.data[1][2] = r.data[2][0] = r.data[2][1] = r.data[3][0] = r.data[3][1] = r.data[3][2] = 0;

        return r;
    }

    
    // Shamelessly lifted from https://cgit.freedesktop.org/mesa/glu/tree/src/libutil/project.c#n163. 
    // Seems to be some pure magic that I don't understand.
    matrix4<T> inverse() const { 
        matrix4<T> r;

        double inv[16], det;
        double m[16];
        std::memcpy(&m, &data, sizeof(m));

        inv[0] = m[5]  * m[10] * m[15] - 
                m[5]  * m[11] * m[14] - 
                m[9]  * m[6]  * m[15] + 
                m[9]  * m[7]  * m[14] +
                m[13] * m[6]  * m[11] - 
                m[13] * m[7]  * m[10];

        inv[4] = -m[4]  * m[10] * m[15] + 
                m[4]  * m[11] * m[14] + 
                m[8]  * m[6]  * m[15] - 
                m[8]  * m[7]  * m[14] - 
                m[12] * m[6]  * m[11] + 
                m[12] * m[7]  * m[10];

        inv[8] = m[4]  * m[9] * m[15] - 
                m[4]  * m[11] * m[13] - 
                m[8]  * m[5] * m[15] + 
                m[8]  * m[7] * m[13] + 
                m[12] * m[5] * m[11] - 
                m[12] * m[7] * m[9];

        inv[12] = -m[4]  * m[9] * m[14] + 
                m[4]  * m[10] * m[13] +
                m[8]  * m[5] * m[14] - 
                m[8]  * m[6] * m[13] - 
                m[12] * m[5] * m[10] + 
                m[12] * m[6] * m[9];

        inv[1] = -m[1]  * m[10] * m[15] + 
                m[1]  * m[11] * m[14] + 
                m[9]  * m[2] * m[15] - 
                m[9]  * m[3] * m[14] - 
                m[13] * m[2] * m[11] + 
                m[13] * m[3] * m[10];

        inv[5] = m[0]  * m[10] * m[15] - 
                m[0]  * m[11] * m[14] - 
                m[8]  * m[2] * m[15] + 
                m[8]  * m[3] * m[14] + 
                m[12] * m[2] * m[11] - 
                m[12] * m[3] * m[10];

        inv[9] = -m[0]  * m[9] * m[15] + 
                m[0]  * m[11] * m[13] + 
                m[8]  * m[1] * m[15] - 
                m[8]  * m[3] * m[13] - 
                m[12] * m[1] * m[11] + 
                m[12] * m[3] * m[9];

        inv[13] = m[0]  * m[9] * m[14] - 
                m[0]  * m[10] * m[13] - 
                m[8]  * m[1] * m[14] + 
                m[8]  * m[2] * m[13] + 
                m[12] * m[1] * m[10] - 
                m[12] * m[2] * m[9];

        inv[2] = m[1]  * m[6] * m[15] - 
                m[1]  * m[7] * m[14] - 
                m[5]  * m[2] * m[15] + 
                m[5]  * m[3] * m[14] + 
                m[13] * m[2] * m[7] - 
                m[13] * m[3] * m[6];

        inv[6] = -m[0]  * m[6] * m[15] + 
                m[0]  * m[7] * m[14] + 
                m[4]  * m[2] * m[15] - 
                m[4]  * m[3] * m[14] - 
                m[12] * m[2] * m[7] + 
                m[12] * m[3] * m[6];

        inv[10] = m[0]  * m[5] * m[15] - 
                m[0]  * m[7] * m[13] - 
                m[4]  * m[1] * m[15] + 
                m[4]  * m[3] * m[13] + 
                m[12] * m[1] * m[7] - 
                m[12] * m[3] * m[5];

        inv[14] = -m[0]  * m[5] * m[14] + 
                m[0]  * m[6] * m[13] + 
                m[4]  * m[1] * m[14] - 
                m[4]  * m[2] * m[13] - 
                m[12] * m[1] * m[6] + 
                m[12] * m[2] * m[5];

        inv[3] = -m[1] * m[6] * m[11] + 
                m[1] * m[7] * m[10] + 
                m[5] * m[2] * m[11] - 
                m[5] * m[3] * m[10] - 
                m[9] * m[2] * m[7] + 
                m[9] * m[3] * m[6];

        inv[7] = m[0] * m[6] * m[11] - 
                m[0] * m[7] * m[10] - 
                m[4] * m[2] * m[11] + 
                m[4] * m[3] * m[10] + 
                m[8] * m[2] * m[7] - 
                m[8] * m[3] * m[6];

        inv[11] = -m[0] * m[5] * m[11] + 
                m[0] * m[7] * m[9] + 
                m[4] * m[1] * m[11] - 
                m[4] * m[3] * m[9] - 
                m[8] * m[1] * m[7] + 
                m[8] * m[3] * m[5];

        inv[15] = m[0] * m[5] * m[10] - 
                m[0] * m[6] * m[9] - 
                m[4] * m[1] * m[10] + 
                m[4] * m[2] * m[9] + 
                m[8] * m[1] * m[6] - 
                m[8] * m[2] * m[5];

        det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

        if (det == 0)
            throw std::logic_error("Inverse cannot be found!");

        det = 1.0 / det;

        r.data[0][0] = inv[0]  * det;
        r.data[0][1] = inv[1]  * det;
        r.data[0][2] = inv[2]  * det;
        r.data[0][3] = inv[3]  * det;
        r.data[1][0] = inv[4]  * det;
        r.data[1][1] = inv[5]  * det;
        r.data[1][2] = inv[6]  * det;
        r.data[1][3] = inv[7]  * det;
        r.data[2][0] = inv[8]  * det;
        r.data[2][1] = inv[9]  * det;
        r.data[2][2] = inv[10] * det;
        r.data[2][3] = inv[11] * det;
        r.data[3][0] = inv[12] * det;
        r.data[3][1] = inv[13] * det;
        r.data[3][2] = inv[14] * det;
        r.data[3][3] = inv[15] * det;

        return r;
        
    }

    constexpr matrix4<T> operator*(const matrix4<T>& other) const {
        matrix4<T> r;

        r.data[0][0] = data[0][0]*other.data[0][0] + data[0][1]*other.data[1][0] + data[0][2]*other.data[2][0] + data[0][3]*other.data[3][0];
        r.data[0][1] = data[0][0]*other.data[0][1] + data[0][1]*other.data[1][1] + data[0][2]*other.data[2][1] + data[0][3]*other.data[3][1];
        r.data[0][2] = data[0][0]*other.data[0][2] + data[0][1]*other.data[1][2] + data[0][2]*other.data[2][2] + data[0][3]*other.data[3][2];
        r.data[0][3] = data[0][0]*other.data[0][3] + data[0][1]*other.data[1][3] + data[0][2]*other.data[2][3] + data[0][3]*other.data[3][3];

        r.data[1][0] = data[1][0]*other.data[0][0] + data[1][1]*other.data[1][0] + data[1][2]*other.data[2][0] + data[1][3]*other.data[3][0];
        r.data[1][1] = data[1][0]*other.data[0][1] + data[1][1]*other.data[1][1] + data[1][2]*other.data[2][1] + data[1][3]*other.data[3][1];
        r.data[1][2] = data[1][0]*other.data[0][2] + data[1][1]*other.data[1][2] + data[1][2]*other.data[2][2] + data[1][3]*other.data[3][2];
        r.data[1][3] = data[1][0]*other.data[0][3] + data[1][1]*other.data[1][3] + data[1][2]*other.data[2][3] + data[1][3]*other.data[3][3];

        r.data[2][0] = data[2][0]*other.data[0][0] + data[2][1]*other.data[1][0] + data[2][2]*other.data[2][0] + data[2][3]*other.data[3][0];
        r.data[2][1] = data[2][0]*other.data[0][1] + data[2][1]*other.data[1][1] + data[2][2]*other.data[2][1] + data[2][3]*other.data[3][1];
        r.data[2][2] = data[2][0]*other.data[2][0] + data[2][1]*other.data[1][2] + data[2][2]*other.data[2][2] + data[2][3]*other.data[3][2];
        r.data[2][3] = data[2][0]*other.data[0][3] + data[2][1]*other.data[1][3] + data[2][2]*other.data[2][3] + data[2][3]*other.data[3][3];

        r.data[3][0] = data[3][0]*other.data[0][0] + data[3][1]*other.data[1][0] + data[3][2]*other.data[2][0] + data[3][3]*other.data[3][0];
        r.data[3][1] = data[3][0]*other.data[0][1] + data[3][1]*other.data[1][1] + data[3][2]*other.data[2][1] + data[3][3]*other.data[3][1];
        r.data[3][2] = data[3][0]*other.data[0][2] + data[3][1]*other.data[1][2] + data[3][2]*other.data[2][2] + data[3][3]*other.data[3][2];
        r.data[3][3] = data[3][0]*other.data[0][3] + data[3][1]*other.data[1][3] + data[3][2]*other.data[2][3] + data[3][3]*other.data[3][3];

        return r;
    }

    constexpr vector3<T> operator*(const vector3<T>& other) const {
        T x, y, z;

        x = other.x * data[0][0] + other.y * data[0][1] + other.z * data[0][2] + data[0][3];
        y = other.x * data[1][0] + other.y * data[1][1] + other.z * data[1][2] + data[1][3];
        z = other.x * data[2][0] + other.y * data[2][1] + other.z * data[2][2] + data[2][3];

        return vector3<T>(x, y, z);
    }

};

#endif