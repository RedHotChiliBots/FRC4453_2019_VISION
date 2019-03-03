#ifndef CONVERSION_HPP
#define CONVERSION_HPP
#include <Eigen/Core>
#include "libpixyusb2.h"

/**
 * Converts a pixy vector into two Eigen vectors (one for each end).
 */
template<typename T>
std::pair<Eigen::Vector2<T>, Eigen::Vector2<T>> pixy_vec_to_vec2(const Vector& v) {
    T a_x(v.m_x0);
    T a_y(v.m_y0);
    T b_x(v.m_x1);
    T b_y(v.m_y1);
    return std::make_pair(Eigen::Vector2<T>(a_x, a_y), Eigen::Vector2<T>(b_x, b_y));
}

#endif