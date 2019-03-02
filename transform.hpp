#ifndef TRANSFORM_HPP
#define TRANSFORM_HPP
#include <Eigen/Core>
#include "cammath.hpp"
#ifdef TESTING
#include <iostream>
#endif
template<typename T>
std::pair<Eigen::Vector2<T>, Eigen::Vector2<T>> transform_vector(const std::pair<Eigen::Vector2<T>, Eigen::Vector2<T>>& vp, const camera3<T>& cam, const Eigen::Hyperplane<T, 3>& plane) {
    auto a_r = cam.cast_ray(vp.first);
    auto b_r = cam.cast_ray(vp.second);

#ifdef TESTING
    std::cout << "Ra: (" << a_r.position().x() << ", " << a_r.position().y() << ", " << a_r.position().z() << "), (" << a_r.direction().x() << ", " << a_r.direction().y() << ", " << a_r.direction().z() << ")" << std::endl;
    std::cout << "Rb: (" << b_r.position().x() << ", " << b_r.position().y() << ", " << b_r.position().z() << "), (" << b_r.direction().x() << ", " << b_r.direction().y() << ", " << b_r.direction().z() << ")" << std::endl;
#endif

    return std::make_pair(a_r.intersectionPoint(plane).head(2), b_r.intersectionPoint(plane).head(2));
}
#endif