//
// Created by lasagnaphil on 10/2/18.
//

#ifndef FLUID_SIM_ARRAY2D_H
#define FLUID_SIM_ARRAY2D_H

#include <cstddef>
#include <mathfu/vector.h>
#include <Map.h>
#include <math/Utils.h>

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

    void reset() {
        memset(data, 0, sizeof(T)*NY*NX);
    }

    Array2D& operator+=(const Array2D& rhs) {
        for (size_t j = 0; j < NY; j++)
            for (size_t i = 0; i < NX; i++)
                (*this)(i,j) += rhs(i,j);
        return *this;
    }

    Array2D& operator/=(const Array2D& rhs) {
        for (size_t j = 0; j < NY; j++)
            for (size_t i = 0; i < NX; i++)
                (*this)(i,j) /= rhs(i,j);
        return *this;
    }

    void safeDivBy(const Array2D& rhs) {
        for (size_t j = 0; j < NY; j++) {
            for (size_t i = 0; i < NX; i++) {
                if (rhs(i,j) != 0.0)
                    (*this)(i,j) /= rhs(i,j);
                else
                    (*this)(i,j) = 0.0;
            }
        }
    }

    // TODO: FMA this
    void setMultiplyAdd(Array2D& a, T b, const Array2D& c) {
        for (size_t j = 0; j < NY; j++)
            for (size_t i = 0; i < NX; i++)
                (*this)(i,j) = a(i,j) + b * c(i,j);
    }

    T innerProduct(const Array2D& rhs) const {
        T result = {};
        for (size_t j = 0; j < NY; j++)
            for (size_t i = 0; i < NX; i++)
                result += (*this)(i,j)*rhs(i,j);
        return result;
    }

    T infiniteNorm() const {
        T result = {};
        for (size_t j = 0; j < NY; j++)
            for (size_t i = 0; i < NX; i++) {
                if (abs((*this)(i,j)) > result) result = abs((*this)(i,j));
            }
        return result;
    }

    T triCubic(mathfu::Vector<T, 2> p) {
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
            int yp = utils::clamp<int>(y+j-1, 0, NY-1);
            r[j] = 0;
            for (int i = 0; i < 4; i++)
            {
                int xp = utils::clamp<int>(x+i-1, 0, NX-1);
                r[j] += u[i] * data[yp][xp];
            }
            vox += v[j] * r[j];
        }
        return (T)(vox < 0 ? 0.0 : vox);

#undef CUBE
#undef SQR
    }

    void distribute(mathfu::Vector<T, 2> pos, T value) {
        int ui = (int) (pos.x);
        int uj = (int) (pos.y);
        auto disp = mathfu::Vector<T, 2>(pos.x - ui, pos.y - uj);
        T kx1 = (0.5 - disp.x) * (0.5 - disp.x);
        T kx2 = (0.75 - disp.x * disp.x);
        T kx3 = 0.75 - (1.0 - disp.x) * (1.0 - disp.x);
        T ky1 = (0.5 - disp.y) * (0.5 - disp.y);
        T ky2 = (0.75 - disp.y * disp.y);
        T ky3 = 0.75 - (1.0 - disp.y) * (1.0 - disp.y);
        if (disp.x < 0.5 && disp.y < 0.5) {
            (*this)(ui - 1, uj - 1) += kx1 * ky1 * value;
            (*this)(ui - 1, uj    ) += kx1 * ky2 * value;
            (*this)(ui - 1, uj + 1) += kx1 * ky3 * value;
            (*this)(ui    , uj - 1) += kx2 * ky1 * value;
            (*this)(ui    , uj    ) += kx2 * ky2 * value;
            (*this)(ui    , uj + 1) += kx2 * ky3 * value;
            (*this)(ui + 1, uj - 1) += kx3 * ky1 * value;
            (*this)(ui + 1, uj    ) += kx3 * ky2 * value;
            (*this)(ui + 1, uj + 1) += kx3 * ky3 * value;
        }
        else if (disp.x > 0.5 && disp.y < 0.5) {
            (*this)(ui    , uj - 1) += kx2 * ky1 * value;
            (*this)(ui    , uj    ) += kx2 * ky2 * value;
            (*this)(ui    , uj + 1) += kx2 * ky3 * value;
            (*this)(ui + 1, uj - 1) += kx3 * ky1 * value;
            (*this)(ui + 1, uj    ) += kx3 * ky2 * value;
            (*this)(ui + 1, uj + 1) += kx3 * ky3 * value;
            (*this)(ui + 2, uj - 1) += kx1 * ky1 * value;
            (*this)(ui + 2, uj    ) += kx1 * ky2 * value;
            (*this)(ui + 2, uj + 1) += kx1 * ky3 * value;
        }
        else if (disp.x < 0.5 && disp.y > 0.5) {
            (*this)(ui - 1, uj    ) += kx1 * ky2 * value;
            (*this)(ui - 1, uj + 1) += kx1 * ky3 * value;
            (*this)(ui - 1, uj + 2) += kx1 * ky1 * value;
            (*this)(ui    , uj    ) += kx2 * ky2 * value;
            (*this)(ui    , uj + 1) += kx2 * ky3 * value;
            (*this)(ui    , uj + 2) += kx2 * ky1 * value;
            (*this)(ui + 1, uj    ) += kx3 * ky2 * value;
            (*this)(ui + 1, uj + 1) += kx3 * ky3 * value;
            (*this)(ui + 1, uj + 2) += kx3 * ky1 * value;
        }
        else if (disp.x > 0.5 && disp.y > 0.5) {
            (*this)(ui    , uj    ) += kx2 * ky2 * value;
            (*this)(ui    , uj + 1) += kx2 * ky3 * value;
            (*this)(ui    , uj + 2) += kx2 * ky1 * value;
            (*this)(ui + 1, uj    ) += kx3 * ky2 * value;
            (*this)(ui + 1, uj + 1) += kx3 * ky3 * value;
            (*this)(ui + 1, uj + 2) += kx3 * ky1 * value;
            (*this)(ui + 2, uj    ) += kx1 * ky2 * value;
            (*this)(ui + 2, uj + 1) += kx1 * ky3 * value;
            (*this)(ui + 2, uj + 2) += kx1 * ky1 * value;
        }
    }

    T extract(mathfu::Vector<T, 2> pos) {
        T value = (T) 0;
        int ui = (int) (pos.x);
        int uj = (int) (pos.y);
        auto disp = mathfu::Vector<T, 2>(pos.x - ui, pos.y - uj);
        T kx1 = (0.5 - disp.x) * (0.5 - disp.x);
        T kx2 = (0.75 - disp.x * disp.x);
        T kx3 = 0.75 - (1.0 - disp.x) * (1.0 - disp.x);
        T ky1 = (0.5 - disp.y) * (0.5 - disp.y);
        T ky2 = (0.75 - disp.y * disp.y);
        T ky3 = 0.75 - (1.0 - disp.y) * (1.0 - disp.y);
        if (disp.x < 0.5 && disp.y < 0.5) {
            value += kx1 * ky1 * (*this)(ui - 1, uj - 1);
            value += kx1 * ky2 * (*this)(ui - 1, uj    );
            value += kx1 * ky3 * (*this)(ui - 1, uj + 1);
            value += kx2 * ky1 * (*this)(ui    , uj - 1);
            value += kx2 * ky2 * (*this)(ui    , uj    );
            value += kx2 * ky3 * (*this)(ui    , uj + 1);
            value += kx3 * ky1 * (*this)(ui + 1, uj - 1);
            value += kx3 * ky2 * (*this)(ui + 1, uj    );
            value += kx3 * ky3 * (*this)(ui + 1, uj + 1);
        }
        else if (disp.x > 0.5 && disp.y < 0.5) {
            value += kx2 * ky1 * (*this)(ui    , uj - 1);
            value += kx2 * ky2 * (*this)(ui    , uj    );
            value += kx2 * ky3 * (*this)(ui    , uj + 1);
            value += kx3 * ky1 * (*this)(ui + 1, uj - 1);
            value += kx3 * ky2 * (*this)(ui + 1, uj    );
            value += kx3 * ky3 * (*this)(ui + 1, uj + 1);
            value += kx1 * ky1 * (*this)(ui + 2, uj - 1);
            value += kx1 * ky2 * (*this)(ui + 2, uj    );
            value += kx1 * ky3 * (*this)(ui + 2, uj + 1);
        }
        else if (disp.x < 0.5 && disp.y > 0.5) {
            value += kx1 * ky2 * (*this)(ui - 1, uj    );
            value += kx1 * ky3 * (*this)(ui - 1, uj + 1);
            value += kx1 * ky1 * (*this)(ui - 1, uj + 2);
            value += kx2 * ky2 * (*this)(ui    , uj    );
            value += kx2 * ky3 * (*this)(ui    , uj + 1);
            value += kx2 * ky1 * (*this)(ui    , uj + 2);
            value += kx3 * ky2 * (*this)(ui + 1, uj    );
            value += kx3 * ky3 * (*this)(ui + 1, uj + 1);
            value += kx3 * ky1 * (*this)(ui + 1, uj + 2);
        }
        else if (disp.x > 0.5 && disp.y > 0.5) {
            value += kx2 * ky2 * (*this)(ui    , uj    );
            value += kx2 * ky3 * (*this)(ui    , uj + 1);
            value += kx2 * ky1 * (*this)(ui    , uj + 2);
            value += kx3 * ky2 * (*this)(ui + 1, uj    );
            value += kx3 * ky3 * (*this)(ui + 1, uj + 1);
            value += kx3 * ky1 * (*this)(ui + 1, uj + 2);
            value += kx1 * ky2 * (*this)(ui + 2, uj    );
            value += kx1 * ky3 * (*this)(ui + 2, uj + 1);
            value += kx1 * ky1 * (*this)(ui + 2, uj + 2);
        }
        return value;
    }
};

#endif //FLUID_SIM_ARRAY2D_H
