//
// Created by lasagnaphil on 9/13/18.
//

#ifndef FLUID_SIM_WATERSIM_H

#include <glad/glad.h>
#include <Eigen/Dense>
#include <cstddef>
#include "HandmadeMath.h"

#include "Array3D.h"
#include "Utils.h"
#include "WaterSimSettings.h"

extern Eigen::Matrix<double, 64, 64> CUBIC_INTERP_MAT;

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
    Array3D<double, SIZEX + 1, SIZEY, SIZEZ> u = {};
    Array3D<double, SIZEX, SIZEY + 1, SIZEZ> v = {};
    Array3D<double, SIZEX, SIZEY, SIZEZ + 1> w = {};

    Array3D<CellType, SIZEX, SIZEY, SIZEZ> cell = {};

    Eigen::Vector3d gravity = {0, -9.8, 0};
    double rho = 1000.0;
    double dx = 0.01;
    double dt = 1.0 / 60.0;

    void setup();

    void runFrame();

    void update();

    void debugPrint();

    void applyAdvection();

    void applyGravity();

    void applyProjection();

    Eigen::Vector3d clampPos(const Eigen::Vector3d& x);

    Grid3D<double> applyA(const Grid3D<double>& r,
                          const Grid3D<double>& Adiag, const Grid3D<double>& Aplusi, const Grid3D<double>& Aplusj, const Grid3D<double>& Aplusk);

    Grid3D<double> applyPreconditioner(const Grid3D<double>& r,
            const Grid3D<double>& precon, const Grid3D<double>& Aplusi, const Grid3D<double>& Aplusj, const Grid3D<double>& Aplusk);

    Eigen::Vector3d vel(size_t i, size_t j, size_t k);

    Eigen::Vector3d velU(size_t i, size_t j, size_t k);

    Eigen::Vector3d velV(size_t i, size_t j, size_t k);

    Eigen::Vector3d velW(size_t i, size_t j, size_t k);

    template <typename Fun>
    void iterateU(Fun f) {
        for (size_t i = 0; i < SIZEX + 1; i++)
            for (size_t j = 0; j < SIZEY; j++)
                for (size_t k = 0; k < SIZEZ; k++)
                    f(i, j, k);
    }
    template <typename Fun>
    void iterateV(Fun f) {
        for (size_t i = 0; i < SIZEX; i++)
            for (size_t j = 0; j < SIZEY + 1; j++)
                for (size_t k = 0; k < SIZEZ; k++)
                    f(i, j, k);
    }

    template <typename Fun>
    void iterateW(Fun f) {
        for (size_t i = 0; i < SIZEX; i++)
            for (size_t j = 0; j < SIZEY; j++)
                for (size_t k = 0; k < SIZEZ + 1; k++)
                    f(i, j, k);
    }

    template <typename Fun>
    void iterate(Fun f) {
        for (size_t i = 0; i < SIZEX; i++)
            for (size_t j = 0; j < SIZEY; j++)
                for (size_t k = 0; k < SIZEZ; k++)
                    f(i, j, k);
    }

    template <typename Fun>
    void iterateBackwards(Fun f) {
        for (size_t i = SIZEX; i-- > 0;) {
            for (size_t j = SIZEY; j-- > 0;) {
                for (size_t k = SIZEZ; k-- > 0;) {
                    f(i, j, k);
                }
            }
        }
    }

    template <typename Fun>
    double triCubicInterp(Eigen::Vector3d pos, Fun f) {
        Eigen::Matrix<double, 64, 1> interpValues;
        Eigen::Vector3i flr((int)floor(pos(0)), (int)floor(pos(1)), (int)floor(pos(2)));
        flr(0) = clamp(flr(0), 0, SIZEX - 1);
        flr(1) = clamp(flr(1), 0, SIZEY - 1);
        flr(2) = clamp(flr(2), 0, SIZEZ - 1);
        Eigen::Vector3d offset = pos - flr.cast<double>();
        if (offset(0) < 0 || offset(0) >= 1) offset(0) = 0;
        if (offset(1) < 0 || offset(1) >= 1) offset(1) = 0;
        if (offset(2) < 0 || offset(2) >= 1) offset(2) = 0;

        for (int k = 0; k < 4; k++) {
            for (int j = 0; j < 4; j++) {
                for (int i = 0; i < 4; i++) {
                    double gridVal = f(clamp(flr(0) + i - 1, 0, SIZEX - 1), clamp(flr(1) + j - 1, 0, SIZEY - 1), clamp(flr(2) + k - 1, 0, SIZEZ - 1));
                    interpValues(16 * i + 4 * j + k) = gridVal;
                }
            }
        }
        Eigen::Matrix<double, 64, 1> coeffs = CUBIC_INTERP_MAT * interpValues;
        double result;
        for (int k = 0; k < 4; k++) {
            for (int j = 0; j < 4; j++) {
                for (int i = 0; i < 4; i++) {
                    result += (powi(offset(0), i) * powi(offset(1), j) * powi(offset(2), k)) * coeffs(i * 16 + j * 4 + k);
                }
            }
        }
        return result;
    }

    inline double velInterpU(const Eigen::Vector3d& pos) {
        return triCubicInterp(pos, [this](size_t i, size_t j, size_t k) { return 0.5 * (u(i,j,k) + u(i+1,j,k)); });
    }

    inline double velInterpV(const Eigen::Vector3d& pos) {
        return triCubicInterp(pos, [this](size_t i, size_t j, size_t k) { return 0.5 * (v(i,j,k) + v(i,j+1,k)); });
    }

    inline double velInterpW(const Eigen::Vector3d& pos) {
        return triCubicInterp(pos, [this](size_t i, size_t j, size_t k) { return 0.5 * (w(i,j,k) + w(i,j,k+1)); });
    }

    inline Eigen::Vector3d velInterp(const Eigen::Vector3d& pos) {
        return Eigen::Vector3d(velInterpU(pos), velInterpV(pos), velInterpW(pos));
    }
};

#endif //FLUID_SIM_WATERSIM_H
