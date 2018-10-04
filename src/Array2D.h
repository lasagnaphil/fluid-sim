//
// Created by lasagnaphil on 10/2/18.
//

#ifndef FLUID_SIM_ARRAY2D_H
#define FLUID_SIM_ARRAY2D_H

#include <cstddef>
#include <math/Vector2.h>

template <typename T, size_t NX, size_t NY>
struct Array2D {
    T data[NY][NX] = {};

    void copyFrom(Array2D& arr) {
        memcpy(data, arr.data, sizeof(T)*NX*NY);
    }

    T& operator()(size_t i, size_t j) {
        return data[j][i];
    }
    const T& operator()(size_t i, size_t j) const {
        return data[j][i];
    }

    Array2D& operator+=(const Array2D& rhs) {
        for (size_t j = 0; j < NY + 1; j++)
            for (size_t i = 0; i < NX; i++)
                (*this)(i,j) += rhs(i,j);
        return *this;
    }

    // TODO: FMA this
    void setMultiplyAdd(Array2D& a, T b, const Array2D& c) {
        for (size_t j = 0; j < NY + 1; j++)
            for (size_t i = 0; i < NX; i++)
                (*this)(i,j) = a(i,j) + b * c(i,j);
    }

    T innerProduct(const Array2D& rhs) const {
        T result = {};
        for (size_t j = 0; j < NY + 1; j++)
            for (size_t i = 0; i < NX; i++)
                result += (*this)(i,j)*rhs(i,j);
        return result;
    }

    T infiniteNorm() const {
        T result = {};
        for (size_t j = 0; j < NY + 1; j++)
            for (size_t i = 0; i < NX; i++) {
                if (abs((*this)(i,j)) > result) result = abs((*this)(i,j));
            }
        return result;
    }

    T triCubic(Vector2<T> p) {
        int x = (int) p.x, y = (int) p.y;
        if (x < 0 || x >= NX || y < 0 || y >= NY) {
            return 0;
        }
        T dx = p.x - (T)x, dy = p.y - (T)y;
        T* pv = (T*)data + (x - 1) + (y - 1) * NX;

    #define CUBE(x)   ((x) * (x) * (x))
    #define SQR(x)    ((x) * (x))

        /* factors for Catmull-Rom interpolation */
        T u[4], v[4];
        T r[4];
        T vox = 0;

        u[0] = -0.5 * CUBE (dx) + SQR (dx) - 0.5 * dx;
        u[1] =  1.5 * CUBE (dx) - 2.5 * SQR (dx) + 1;
        u[2] = -1.5 * CUBE (dx) + 2 * SQR (dx) + 0.5 * dx;
        u[3] =  0.5 * CUBE (dx) - 0.5 * SQR (dx);

        v[0] = -0.5 * CUBE (dy) + SQR (dy) - 0.5 * dy;
        v[1] =  1.5 * CUBE (dy) - 2.5 * SQR (dy) + 1;
        v[2] = -1.5 * CUBE (dy) + 2 * SQR (dy) + 0.5 * dy;
        v[3] =  0.5 * CUBE (dy) - 0.5 * SQR (dy);

        for (int j = 0; j < 4; j++)
        {
            r[j] = 0;
            for (int i = 0; i < 4; i++)
            {
                r[j] += u[i] * *pv;
                pv++;
            }
            vox += v[j] * r[j];
            pv += NX - 4;
        }
        return (T)(vox < 0 ? 0.0 : vox);

    #undef CUBE
    #undef SQR
    }
};

#endif //FLUID_SIM_ARRAY2D_H
