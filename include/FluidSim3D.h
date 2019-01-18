//
// Created by lasagnaphil on 9/13/18.
//

#ifndef FLUID_SIM_FLUIDSIM_H

#include <stdint.h>
#include "FluidSimSettings.h"
#include "MACGrid3D.h"

struct FluidSim3D {
    static constexpr int SIZEX = FluidSimSettings::Dim3D::SIZEX;
    static constexpr int SIZEY = FluidSimSettings::Dim3D::SIZEY;
    static constexpr int SIZEZ = FluidSimSettings::Dim3D::SIZEZ;

    template <typename T>
    using Grid3D = Array3D<T, SIZEX, SIZEY, SIZEZ>;

    enum class CellType : uint8_t {
        EMPTY, FLUID, SOLID
    };

    Array3D<double, SIZEX, SIZEY, SIZEZ> p = {};
    Array3D<CellType, SIZEX, SIZEY, SIZEZ> cell = {};
    MACGrid3D<SIZEX, SIZEY, SIZEZ> mac;

    vec3d gravity = {0, -9.8, 0};
    double rho = 1000.0;
    double dt = 0.01;
    double dx = 0.001;

    bool rendered = false;
    enum Stage {
        READY = 0, ADVECTION, GRAVITY, PROJECTION
    };
    Stage stage = Stage::READY;

    void setup();

    void runFrame();

    void update();

    void debugPrint();

    void applyAdvection();

    void applyGravity();

    void applyProjection();

    void updateCells();

    vec3d clampPos(vec3d x);
};

#endif //FLUID_SIM_FLUIDSIM_H
