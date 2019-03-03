#ifndef TRANSFORM_HPP
#define TRANSFORM_HPP
#include <Eigen/Core>
#include "cammath.hpp"

/**
 * Projects vectors from a camera onto a vertical plane.
 */
template<typename T>
std::pair<Eigen::Vector2<T>, Eigen::Vector2<T>> transform_vector(const std::pair<Eigen::Vector2<T>, Eigen::Vector2<T>>& vp, const camera3<T>& cam, const Eigen::Hyperplane<T, 3>& plane) {
    auto a_r = cam.cast_ray(vp.first); // Cast a ray for each end of the vector,
    auto b_r = cam.cast_ray(vp.second);

    // Get intersections with plane, chop off Z value, return them.
    return std::make_pair(a_r.intersectionPoint(plane).head(2), b_r.intersectionPoint(plane).head(2));
}

#endif