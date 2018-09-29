//
// Created by lasagnaphil on 2018-09-18.
//

#ifndef FLUID_SIM_WATERSIM2D_H
#define FLUID_SIM_WATERSIM2D_H

#include <cstdint>
#include <math/Utils.h>
#include "WaterSimSettings.h"
#include "math/Vector2.h"

template <typename T, size_t NX, size_t NY>
struct Array2D {
    T data[NY][NX];

    T& operator()(size_t i, size_t j) {
        return data[j][i];
    }
    const T& operator()(size_t i, size_t j) const {
        return data[j][i];
    }
};

template <size_t NX, size_t NY>
struct MACGrid2D {
    Array2D<double, NX + 1, NY> u;
    Array2D<double, NX, NY + 1> v;
    float dx = 0.001;

    template <typename Fun>
    void iterateU(Fun f) {
        for (size_t j = 0; j < NY; j++) {
            for (size_t i = 0; i < NX + 1; i++) {
                f(i, j);
            }
        }
    }

    template <typename Fun>
    void iterateV(Fun f) {
        for (size_t j = 0; j < NY + 1; j++) {
            for (size_t i = 0; i < NX; i++) {
                f(i, j);
            }
        }
    }

    template <typename Fun>
    void iterate(Fun f) {
        for (size_t j = 0; j < NY; j++) {
            for (size_t i = 0; i < NX; i++) {
                f(i, j);
            }
        }
    }

    Vector2d velU(size_t i, size_t j) {
        return {
            u(i,j),
            0.25 * (v(i-1,j) + v(i-1,j+1) + v(i,j) + v(i,j+1))
        };
    }

    Vector2d velV(size_t i, size_t j) {
        return {
            0.25 * (u(i,j-1) + u(i+1,j-1) + u(i,j) + u(i+1,j)),
            v(i,j),
        };
    }

    Vector2d vel(size_t i, size_t j) {
        return {
            0.5 * (u(i+1,j) + u(i,j)),
            0.5 * (v(i,j+1) + v(i,j))
        };
    }

    Vector2d velInterp(Vector2d pos) {
        // TODO: use cubic instead of linear interpolation
        size_t i = utils::clamp(floor(pos.x), 0.0, (double)NX - 1);
        size_t j = utils::clamp(floor(pos.y), 0.0, (double)NY - 1);
        Vector2d offset = pos - Vector2d::create(i, j);
        return (1 - offset.x) * (1 - offset.y) * vel(i,j)
            + offset.x * (1 - offset.y) * vel(i+1,j)
            + (1 - offset.x) * offset.y * vel(i,j+1)
            + offset.x * offset.y * vel(i+1,j+1);
    }

    double velDiv(size_t i, size_t j) {
        return u(i+1,j)-u(i,j) + v(i,j+1)-v(i,j);
    }
};

#define GET(arr, __i, __j) (arr(__j * SIZEX + __i))

struct WaterSim2D {
    static constexpr int SIZEX = WaterSimSettings::Dim2D::SIZEX;
    static constexpr int SIZEY = WaterSimSettings::Dim2D::SIZEY;

    enum class CellType : uint8_t {
        EMPTY, FLUID, SOLID
    };

    MACGrid2D<SIZEX, SIZEY> mac = {};
    Array2D<double, SIZEX, SIZEY> p = {};
    Array2D<CellType, SIZEX, SIZEY> cell = {};

    Vector2d gravity = {0, -9.8};
    double rho = 1000.0;
    double dt = 1.0 / 60.0;

    void setup();

    void runFrame();

    void update();

    void applyAdvection();

    void applyGravity();

    void applyProjection();

    Vector2d clampPos(Vector2d pos) {
        Vector2d clamped = {};
        clamped.x = utils::clamp(pos.x, 0.0, (double)SIZEX);
        clamped.y = utils::clamp(pos.y, 0.0, (double)SIZEY);
        return clamped;
    }
};


#endif //FLUID_SIM_WATERSIM2D_H
