//
// Created by lasagnaphil on 9/14/18.
//

#ifndef FLUID_SIM_ARRAY3D_H
#define FLUID_SIM_ARRAY3D_H

#include <cstddef>
#include "Eigen/Core"

template <typename T, size_t N1, size_t N2, size_t N3>
struct Array3D {
    Eigen::Matrix<T, Eigen::Dynamic, 1> data;

    Array3D() : data(N1*N2*N3, 1) {}

    T& operator()(size_t i, size_t j, size_t k) {
        return data(i*N2*N3 + j*N3 + k);
    }
    const T& operator()(size_t i, size_t j, size_t k) const {
        return data(i*N2*N3 + j*N3 + k);
    }

    Array3D<T, N1, N2, N3>& operator+=(const Array3D<T, N1, N2, N3>& other) {
        data += other.data;
        return *this;
    }

    Array3D<T, N1, N2, N3>& operator-=(const Array3D<T, N1, N2, N3>& other) {
        data -= other.data;
        return *this;
    }

    Array3D<T, N1, N2, N3>& operator*=(const Array3D<T, N1, N2, N3>& other) {
        data *= other.data;
        return *this;
    }

    Array3D<T, N1, N2, N3>& operator/=(const Array3D<T, N1, N2, N3>& other) {
        data /= other.data;
        return *this;
    }


    T innerProduct(const Array3D<T, N1, N2, N3>& other) const {
        return data.dot(other.data);
    }

    T infiniteNorm() const {
        return data.template lpNorm<Eigen::Infinity>();
    }
};

template <typename T, size_t N1, size_t N2, size_t N3>
Array3D<T, N1, N2, N3> operator+(const Array3D<T, N1, N2, N3>& a, const Array3D<T, N1, N2, N3>& b) {
    Array3D<T, N1, N2, N3> res;
    res.data = a.data + b.data;
    return res;
}
template <typename T, size_t N1, size_t N2, size_t N3>
Array3D<T, N1, N2, N3> operator-(const Array3D<T, N1, N2, N3>& a, const Array3D<T, N1, N2, N3>& b) {
    Array3D<T, N1, N2, N3> res;
    res.data = a.data - b.data;
    return res;
}
template <typename T, size_t N1, size_t N2, size_t N3>
Array3D<T, N1, N2, N3> operator*(const Array3D<T, N1, N2, N3>& a, const Array3D<T, N1, N2, N3>& b) {
    Array3D<T, N1, N2, N3> res;
    res.data = a.data - b.data;
    return res;
}
template <typename T, size_t N1, size_t N2, size_t N3>
Array3D<T, N1, N2, N3> operator/(const Array3D<T, N1, N2, N3>& a, const Array3D<T, N1, N2, N3>& b) {
    Array3D<T, N1, N2, N3> res;
    res.data = a.data / b.data;
    return res;
}
template <typename T, size_t N1, size_t N2, size_t N3>
Array3D<T, N1, N2, N3> operator*(const T& k, const Array3D<T, N1, N2, N3>& b) {
    Array3D<T, N1, N2, N3> res;
    res.data = k * b.data;
    return res;
}
#endif //FLUID_SIM_ARRAY3D_H
