//
// Created by lasagnaphil on 9/14/18.
//

#ifndef FLUID_SIM_ARRAY3D_H
#define FLUID_SIM_ARRAY3D_H

#include <cstddef>
#include <cmath>
#include <math/Vector3.h>

template <typename T, size_t NX, size_t NY, size_t NZ>
struct Array3D {
    T data[NZ][NY][NX] = {};
    T& operator()(size_t i, size_t j, size_t k) {
        return data[k][j][i];
    }
    const T& operator()(size_t i, size_t j, size_t k) const {
        return data[k][j][i];
    }

    Array3D& operator+=(const Array3D& rhs) {
        for (size_t k = 0; k < NZ; k++)
            for (size_t j = 0; j < NY + 1; j++)
                for (size_t i = 0; i < NX; i++)
                    (*this)(i,j,k) += rhs(i,j,k);
        return *this;
    }

    // TODO: FMA this
    void setMultiplyAdd(Array3D& a, T b, const Array3D& c) {
        for (size_t k = 0; k < NZ; k++)
            for (size_t j = 0; j < NY + 1; j++)
                for (size_t i = 0; i < NX; i++)
                    (*this)(i,j,k) = a(i,j,k) + b * c(i,j,k);
    }

    T innerProduct(const Array3D& rhs) const {
        T result = {};
        for (size_t k = 0; k < NZ; k++)
            for (size_t j = 0; j < NY + 1; j++)
                for (size_t i = 0; i < NX; i++)
                    result += (*this)(i,j,k)*rhs(i,j,k);
        return result;
    }

    T infiniteNorm() const {
        T result = {};
        for (size_t k = 0; k < NZ; k++)
            for (size_t j = 0; j < NY + 1; j++)
                for (size_t i = 0; i < NX; i++) {
                    if (abs((*this)(i,j,k)) > result) result = abs((*this)(i,j,k));
                }
        return result;
    }

    // from http://www.realtimerendering.com/resources/GraphicsGems/gemsv/ch3-3/tricubic.c
    T triCubic(Vector3<T> p)
    {
        int x = (int) p.x, y = (int) p.y, z = (int) p.z;
        if (x < 0 || x >= NX || y < 0 || y >= NY || z < 0 || z >= NZ)
            return (0);

        T dx = p.x - (T) x, dy = p.y - (T) y, dz = p.z - (T) z;
        T* pv = (T*)data + (x - 1) + (y - 1) * NX + (z - 1) * NX * NY;

    #define CUBE(x)   ((x) * (x) * (x))
    #define SQR(x)    ((x) * (x))

        /* factors for Catmull-Rom interpolation */
        T u[4], v[4], w[4];
        T r[4], q[4];
        T vox = 0;

        u[0] = -0.5f * CUBE (dx) + SQR (dx) - 0.5f * dx;
        u[1] =  1.5f * CUBE (dx) - 2.5f * SQR (dx) + 1;
        u[2] = -1.5f * CUBE (dx) + 2 * SQR (dx) + 0.5f * dx;
        u[3] =  0.5f * CUBE (dx) - 0.5f * SQR (dx);

        v[0] = -0.5f * CUBE (dy) + SQR (dy) - 0.5f * dy;
        v[1] =  1.5f * CUBE (dy) - 2.5f * SQR (dy) + 1;
        v[2] = -1.5f * CUBE (dy) + 2 * SQR (dy) + 0.5f * dy;
        v[3] =  0.5f * CUBE (dy) - 0.5f * SQR (dy);

        w[0] = -0.5f * CUBE (dz) + SQR (dz) - 0.5f * dz;
        w[1] =  1.5f * CUBE (dz) - 2.5f * SQR (dz) + 1;
        w[2] = -1.5f * CUBE (dz) + 2 * SQR (dz) + 0.5f * dz;
        w[3] =  0.5f * CUBE (dz) - 0.5f * SQR (dz);

        for (int k = 0; k < 4; k++)
        {
            q[k] = 0;
            for (int j = 0; j < 4; j++)
            {
                r[j] = 0;
                for (int i = 0; i < 4; i++)
                {
                    r[j] += u[i] * *pv;
                    pv++;
                }
                q[k] += v[j] * r[j];
                pv += NX - 4;
            }
            vox += w[k] * q[k];
            pv += NX * NY - 4 * NX;
        }
        return (T)(vox < 0 ? 0.0 : vox);

    #undef CUBE
    #undef SQR
    }
};

#endif //FLUID_SIM_ARRAY3D_H
