//
// Created by lasagnaphil on 9/13/18.
//

#ifndef FLUID_SIM_WATERSIM_H

#include <glad/glad.h>
#include <Eigen/Dense>
#include <cstddef>
#include <Vec.h>
#include <math/Vector3.h>
#include <math/Utils.h>

#include "WaterSimSettings.h"

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

template <size_t SIZEX, size_t SIZEY, size_t SIZEZ>
struct MACGrid3D {
    Array3D<double, SIZEX + 1, SIZEY, SIZEZ> u = {};
    Array3D<double, SIZEX, SIZEY + 1, SIZEZ> v = {};
    Array3D<double, SIZEX, SIZEY, SIZEZ + 1> w = {};
    double dx = 0.001;

    template <typename Fun>
    void iterateU(Fun f) {
        for (size_t k = 0; k < SIZEZ; k++)
            for (size_t j = 0; j < SIZEY; j++)
                for (size_t i = 0; i < SIZEX + 1; i++)
                    f(i, j, k);
    }
    template <typename Fun>
    void iterateV(Fun f) {
        for (size_t k = 0; k < SIZEZ; k++)
            for (size_t j = 0; j < SIZEY + 1; j++)
                for (size_t i = 0; i < SIZEX; i++)
                    f(i, j, k);
    }

    template <typename Fun>
    void iterateW(Fun f) {
        for (size_t k = 0; k < SIZEZ + 1; k++)
            for (size_t j = 0; j < SIZEY; j++)
                for (size_t i = 0; i < SIZEX; i++)
                    f(i, j, k);
    }

    template <typename Fun>
    void iterate(Fun f) {
        for (size_t k = 0; k < SIZEZ; k++)
            for (size_t j = 0; j < SIZEY; j++)
                for (size_t i = 0; i < SIZEX; i++)
                    f(i, j, k);
    }

    template <typename Fun>
    void iterateBackwards(Fun f) {
        for (size_t k = SIZEZ; k-- > 0;) {
            for (size_t j = SIZEY; j-- > 0;) {
                for (size_t i = SIZEX; i-- > 0;) {
                    f(i, j, k);
                }
            }
        }
    }

    Vector3d vel(size_t i, size_t j, size_t k) {
        return 0.5 * Vector3d::create(
                u(i+1,j,k) + u(i,j,k),
                v(i,j+1,k) + v(i,j,k),
                w(i,j,k+1) + w(i,j,k));
    }

    Vector3d velU(size_t i, size_t j, size_t k) {
        if (i > 0 && i < SIZEX)
            return Vector3d::create(
                    u(i,j,k),
                    0.25 * (v(i-1,j,k) + v(i-1,j+1,k) + v(i,j,k) + v(i,j+1,k)),
                    0.25 * (w(i-1,j,k) + w(i-1,j,k+1) + w(i,j,k) + w(i,j,k+1))
            );
        else if (i == 0)
            return Vector3d::create(
                    u(i,j,k),
                    0.25 * (v(i,j,k) + v(i,j+1,k)),
                    0.25 * (w(i,j,k) + w(i,j,k+1))
            );
        else
            return Vector3d::create(
                    u(i,j,k),
                    0.25 * (v(i-1,j,k) + v(i-1,j+1,k)),
                    0.25 * (w(i-1,j,k) + w(i-1,j,k+1))
            );
    }

    Vector3d velV(size_t i, size_t j, size_t k) {
        if (j > 0 && j < SIZEY)
            return Vector3d::create(
                    0.25 * (u(i,j-1,k) + u(i,j,k) + u(i+1,j-1,k) + u(i+1,j,k)),
                    v(i,j,k),
                    0.25 * (w(i,j-1,k) + w(i,j-1,k+1) + w(i,j,k) + w(i,j,k+1))
            );
        else if (j == 0)
            return Vector3d::create(
                    0.25 * (u(i,j,k) + u(i+1,j,k)),
                    v(i,j,k),
                    0.25 * (w(i,j,k) + w(i,j,k+1))
            );
        else
            return Vector3d::create(
                    0.25 * (u(i,j-1,k) + u(i+1,j-1,k)),
                    v(i,j,k),
                    0.25 * (w(i,j-1,k) + w(i,j-1,k+1))
            );
    }

    Vector3d velW(size_t i, size_t j, size_t k) {
        if (k > 0 && k < SIZEZ)
            return Vector3d::create(
                    0.25 * (u(i,j,k-1) + u(i+1,j,k-1) + u(i,j,k) + u(i+1,j,k)),
                    0.25 * (v(i,j,k-1) + v(i,j+1,k-1) + v(i,j,k) + v(i,j+1,k)),
                    w(i,j,k)
            );
        else if (k == 0)
            return Vector3d::create(
                    0.5 * (u(i,j,k) + u(i+1,j,k)),
                    0.5 * (v(i,j,k) + v(i,j+1,k)),
                    w(i,j,k)
            );
        else
            return Vector3d::create(
                    0.5 * (u(i,j,k-1) + u(i+1,j,k-1)),
                    0.5 * (v(i,j,k-1) + v(i,j+1,k-1)),
                    w(i,j,k)
            );
    }

    double velInterpU(Vector3d pos) {
        pos.y -= 0.5;
        pos.z -= 0.5;
        return u.triCubic(pos);
    }

    double velInterpV(Vector3d pos) {
        pos.z -= 0.5;
        pos.x -= 0.5;
        return v.triCubic(pos);
    }

    double velInterpW(Vector3d pos) {
        pos.x -= 0.5;
        pos.y -= 0.5;
        return w.triCubic(pos);
    }

    Vector3d velInterp(Vector3d pos) {
        return Vector3d::create(velInterpU(pos), velInterpV(pos), velInterpW(pos));
    }

    double velDiv(size_t i, size_t j, size_t k) {
        return (u(i+1,j,k) - u(i,j,k) + v(i,j+1,k) - v(i,j,k) + w(i,j,k+1) - w(i,j,k)) / dx;
    }
};

struct WaterSim3D {
    static constexpr int SIZEX = WaterSimSettings::Dim3D::SIZEX;
    static constexpr int SIZEY = WaterSimSettings::Dim3D::SIZEY;
    static constexpr int SIZEZ = WaterSimSettings::Dim3D::SIZEZ;

    template <typename T>
    using Grid3D = Array3D<T, SIZEX, SIZEY, SIZEZ>;

    enum class CellType : uint8_t {
        EMPTY, FLUID, SOLID
    };

    Array3D<double, SIZEX, SIZEY, SIZEZ> p = {};
    Array3D<CellType, SIZEX, SIZEY, SIZEZ> cell = {};
    MACGrid3D<SIZEX, SIZEY, SIZEZ> mac;

    Eigen::Vector3d gravity = {0, -9.8, 0};
    double rho = 1000.0;
    double dt = 1.0 / 60.0;

    void setup();

    void runFrame();

    void update();

    void debugPrint();

    void applyAdvection();

    void applyGravity();

    void applyProjection();

    Vector3d clampPos(const Vector3d& x);
};

#endif //FLUID_SIM_WATERSIM_H
