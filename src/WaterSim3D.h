//
// Created by lasagnaphil on 9/13/18.
//

#ifndef FLUID_SIM_WATERSIM_H

#include <glad/glad.h>

#include "WaterSimSettings.h"
#include "MACGrid3D.h"

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

    Vector3d gravity = {0, -9.8, 0};
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
