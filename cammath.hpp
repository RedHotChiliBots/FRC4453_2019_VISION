#ifndef CAMMATH_H
#define CAMMATH_H
#include "geom.hpp"

template<typename T>
struct camera3 {
    vector2<T> size;
    T fov;
    matrix4<T> rotation_inv;
    matrix4<T> rotation_translation_inv;

    camera3(const vector2<T>& s, const T& f, vector3<T> t, vector3<T> r) : size(s), fov(f) {
        matrix4<T> rotation = matrix4<T>::rotation(r.x, r.y, r.z);
        matrix4<T> translation = matrix4<T>::translation(t.x, t.y, t.z);
        rotation_inv = rotation.inverse();
        rotation_translation_inv = (rotation * translation).inverse();
    }

    camera3(const vector2<T>&& s, const T&& f, vector3<T> t, vector3<T> r) : size(std::move(s)), fov(std::move(f)) {
        matrix4<T> rotation = matrix4<T>::rotation(r.x, r.y, r.z);
        matrix4<T> translation = matrix4<T>::translation(t.x, t.y, t.z);
        rotation_inv = rotation.inverse();
        rotation_translation_inv = (rotation * translation).inverse();
    }

    constexpr T aspect_ratio() const {
        return size.x/size.y;
    }


    constexpr ray3<T> cast_ray_screenspace(const vector2<T>& p) const {
        auto ndc = vector2<T>((p.x + 0.5)/size.x, (p.y + 0.5)/size.y);

        return ray3<T>(vector3<T>(0.0, 0.0, 0.0), vector3<T>((2.0*ndc.x - 1.0) * aspect_ratio() * gcem::tan(fov/2.0), (1.0 - 2.0 * ndc.y) * gcem::tan(fov/2.0), 1.0).normalize());
    }

    constexpr ray3<T> cast_ray(const vector2<T>& p) const {
        auto ray = cast_ray_screenspace(p);
        
        return ray3<T>(rotation_translation_inv * ray.position, rotation_inv * ray.direction);
    }

};

#endif