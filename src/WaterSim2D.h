//
// Created by lasagnaphil on 2018-09-18.
//

#ifndef FLUID_SIM_WATERSIM2D_H
#define FLUID_SIM_WATERSIM2D_H

#include <cstdint>
#include <math/Utils.h>
#include "WaterSimSettings.h"
#include "math/Vector2.h"

#include "Array2D.h"
#include "MACGrid2D.h"

struct WaterSim2D {
    static constexpr int SIZEX = WaterSimSettings::Dim2D::SIZEX;
    static constexpr int SIZEY = WaterSimSettings::Dim2D::SIZEY;

    template <typename T>
    using Grid2D = Array2D<T, SIZEX, SIZEY>;

    enum class CellType : uint8_t {
        EMPTY, FLUID, SOLID
    };

    MACGrid2D<SIZEX, SIZEY> mac = {};
    Array2D<double, SIZEX, SIZEY> p = {};
    Array2D<CellType, SIZEX, SIZEY> cell = {};

    Vector2d gravity = {0, -9.8};
    double rho = 997.0;

    bool rendered = false;
    double currentTime = 0.0f;

    double dt = 0.005;
    double dx = 0.001;

    void setup();

    void runFrame();

    void update();

    void applyAdvection();

    void applyGravity();

    void applyProjection();

    void updateCells();

    Vector2d clampPos(Vector2d pos);

    double avgPressure();
    double avgPressureInFluid();
};


#endif //FLUID_SIM_WATERSIM2D_H
