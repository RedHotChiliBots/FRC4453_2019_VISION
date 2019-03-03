#ifndef CAMMATH_H
#define CAMMATH_H
//#include "geom.hpp"
#include <Eigen/Eigen>
#include <gcem.hpp>

/**
 * Represents a 3d camera.
 */
template<typename T>
struct camera3 {
    Eigen::Vector2<T> size; // Resolution of camera.
    T fov;
    Eigen::Quaternion<T> rotation; // Rotation of camera.
    Eigen::Translation<T, 3> translation; // Position of camera.

    camera3(const Eigen::Vector2<T>& s, const T& f, Eigen::Vector3<T>& t, Eigen::Quaternion<T>& r) : size(s), fov(f), rotation(r), translation(t) {}

    camera3(const Eigen::Vector2<T>&& s, const T&& f, Eigen::Vector3<T>&& t, Eigen::Quaternion<T>&& r) : size(std::move(s)), fov(std::move(f)), rotation(r), translation(t) {}

    constexpr T aspect_ratio() const {
        return size.x()/size.y();
    }

    // Cast a ray through a pixel, with no transformations.
    Eigen::ParametrizedLine<T, 3> cast_ray_screenspace(const Eigen::Vector2<T>& p) const {
        auto ndc = Eigen::Vector2<T>((p.x() + 0.5)/size.x(), (p.y() + 0.5)/size.y());

        return Eigen::ParametrizedLine<T, 3>(Eigen::Vector3<T>(0.0, 0.0, 0.0), Eigen::Vector3<T>((2.0*ndc.x() - 1.0) * aspect_ratio() * gcem::tan(fov/2.0), 1.0, (1.0 - 2.0 * ndc.y()) * gcem::tan(fov/2.0)).normalized());
    }

    // Cast a ray, with transformations applied.
    Eigen::ParametrizedLine<T, 3> cast_ray(const Eigen::Vector2<T>& p) const {
        auto ray = cast_ray_screenspace(p);
        
        // Move and rotate 
        return Eigen::ParametrizedLine<T, 3>(translation * ray.origin(), rotation * ray.direction());
    }
};

#endif