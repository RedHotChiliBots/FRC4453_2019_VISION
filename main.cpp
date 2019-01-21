#include <iostream>
#include <cmath>
#include <algorithm>

#include <libpixyusb2.h>
#include <gcem.hpp>

constexpr double rad2deg(double v) {
    return v * (180.0 / M_PI);
}

constexpr double deg2rad(double v) {
    return v / (180.0 / M_PI);
}

constexpr static double CAM_HEIGHT = 4; // 4 inches. TODO: Find the real value of camera height.
constexpr static double CAM_DOWNTILT = deg2rad(25.0); // Angle below horizontal.
constexpr static double CAM_ANGLE = deg2rad(90.0) - CAM_DOWNTILT;
constexpr static double CAM_FOV = deg2rad(60.0);
constexpr static double CAM_ASPECT_RATIO = 4.0/3.0;
constexpr static double CAM_VFOV = CAM_FOV / CAM_ASPECT_RATIO;
constexpr static uint8_t CAM_VECW = 79;
constexpr static uint8_t CAM_VECH = 52;

constexpr static double CAM_FOCAL_LENGTH_VEC = (0.5 * CAM_VECW) / gcem::tan(0.5 * CAM_FOV);

template<typename T>
struct vector2 {
    vector2(T& x_i, T& y_i) : x(x_i), y(y_i) {}
    vector2(T&& x_i, T&& y_i) : x(std::move(x_i)), y(std::move(y_i)) {}

    T x, y;

    vector2<T>& operator+=(const vector2<T>& other) {
        x += other.x;
        y += other.y;
        return *this;
    };

    // TODO: More operators for vector2.
};

void vector_to_world(vector2<uint8_t> v, vector2<double> v_out) {
    // TODO: Write vector_to_world.
    // Math looks like y = CAM_HEIGHT * tan(y_angle)
    // x = y * tan(x_angle) 
}

int main() {
    Pixy2 pixy;
    std::cout << "Connecting to PixyCam..." << std::endl;

    int result;

    if((result = pixy.init()) < 0) {
        std::cout << "Failed to open PixyCam: " << result << std::endl;
        return -1;
    }

    pixy.setLamp(100, 100);

    while(true) {
        pixy.line.getMainFeatures(LINE_VECTOR, true);
        
        if(pixy.line.numVectors == 0) {
            continue;
        }

        auto the_vector = pixy.line.vectors[0];

        the_vector.m_y0 = -the_vector.m_y0;
        the_vector.m_y1 = -the_vector.m_y1;

        if(the_vector.m_y0 > the_vector.m_y1) {
            std::swap(the_vector.m_y0, the_vector.m_y1);
            std::swap(the_vector.m_x0, the_vector.m_x1);
        }


        //TODO: Finish Main Loop.

    }
}