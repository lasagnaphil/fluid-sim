//
// Created by lasagnaphil on 9/14/18.
//

#ifndef FLUID_SIM_ARRAY3D_H
#define FLUID_SIM_ARRAY3D_H

#include <cstddef>

template <typename T, size_t N1, size_t N2, size_t N3>
struct Array3D {
    T data[N1][N2][N3];

    T& operator()(size_t i, size_t j, size_t k) {
        return data[i][j][k];
    }
    const T& operator()(size_t i, size_t j, size_t k) const {
        return data[i][j][k];
    }
};

#endif //FLUID_SIM_ARRAY3D_H
