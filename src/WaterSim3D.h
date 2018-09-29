//
// Created by lasagnaphil on 9/13/18.
//

#ifndef FLUID_SIM_WATERSIM_H

#include <glad/glad.h>
#include <Eigen/Dense>
#include <cstddef>
#include "math/Vector3.h"
#include "math/Utils.h"

#include "WaterSimSettings.h"

extern Eigen::Matrix<double, 64, 64> CUBIC_INTERP_MAT;

template <typename T, size_t NX, size_t NY, size_t NZ>
struct Array3D {
    T data[NZ][NY][NX];
    T& operator()(size_t i, size_t j, size_t k) {
        return data[k][j][i];
    }
    const T& operator()(size_t i, size_t j, size_t k) const {
        return data[k][j][i];
    }

};

template <size_t SIZEX, size_t SIZEY, size_t SIZEZ>
struct MACGrid3D {
    Array3D<double, SIZEX + 1, SIZEY, SIZEZ> u = {};
    Array3D<double, SIZEX, SIZEY + 1, SIZEZ> v = {};
    Array3D<double, SIZEX, SIZEY, SIZEZ + 1> w = {};
    float dx = 0.001;

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
                    0.5 * (v(i,j,k) + v(i,j+1,k)),
                    0.5 * (w(i,j,k) + w(i,j,k+1))
            );
        else
            return Vector3d::create(
                    u(i,j,k),
                    0.5 * (v(i-1,j,k) + v(i-1,j+1,k)),
                    0.5 * (w(i-1,j,k) + w(i-1,j,k+1))
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
                    0.5 * (u(i,j,k) + u(i+1,j,k)),
                    v(i,j,k),
                    0.5 * (w(i,j,k) + w(i,j,k+1))
            );
        else
            return Vector3d::create(
                    0.5 * (u(i,j-1,k) + u(i+1,j-1,k)),
                    v(i,j,k),
                    0.5 * (w(i,j-1,k) + w(i,j-1,k+1))
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

    double velInterpU(const Vector3d& pos) {
        return utils::triCubic(pos, (double*)u.data, SIZEX + 1, SIZEY, SIZEZ);
    }

    double velInterpV(const Vector3d& pos) {
        return utils::triCubic(pos, (double*)v.data, SIZEX, SIZEY + 1, SIZEZ);
    }

    double velInterpW(const Vector3d& pos) {
        return utils::triCubic(pos, (double*)w.data, SIZEX, SIZEY, SIZEZ + 1);
    }

    Vector3d velInterp(const Vector3d& pos) {
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

    Grid3D<double> applyA(const Grid3D<double>& r,
                          const Grid3D<double>& Adiag, const Grid3D<double>& Aplusi, const Grid3D<double>& Aplusj, const Grid3D<double>& Aplusk);

    Grid3D<double> applyPreconditioner(const Grid3D<double>& r,
            const Grid3D<double>& precon, const Grid3D<double>& Aplusi, const Grid3D<double>& Aplusj, const Grid3D<double>& Aplusk);
};

#endif //FLUID_SIM_WATERSIM_H
