//
// Created by lasagnaphil on 9/16/18.
//

#ifndef FLUID_SIM_UTILS_H
#define FLUID_SIM_UTILS_H

#include "Eigen/Core"

#define SQUARE(x) (x)*(x)
#define CUBE(x) (x)*(x)*(x)

double powi(double d, int i);

Eigen::Vector4d catmullRom(double a);

template <typename T>
inline T max(T a, T b) {
    return (a > b)? a : b;
}

template <typename T>
inline T min(T a, T b) {
    return (a < b)? a : b;
}

template <typename T>
inline T clamp(T value, T lower, T upper) {
    return max(lower, min(value, upper));
}

inline float radians(float deg) {
    return deg * M_PI / 180;
}

inline float degrees(float rad) {
    return rad * 180 / M_PI;
}
#endif //FLUID_SIM_UTILS_H
