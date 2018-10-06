//
// Created by lasagnaphil on 2018-09-18.
//

#ifndef FLUID_SIM_WATERSIM2D_H
#define FLUID_SIM_WATERSIM2D_H

#include <cstdint>
#include <math/Utils.h>
#include <math/Vector2.h>
#include <StackVec.h>
#include <HandmadeMath.h>
#include "WaterSimSettings.h"

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
    Vec<Vector2d> particles = {};
    Array2D<double, SIZEX, SIZEY> phi = {};

    double gravity = -9.81;
    double rho = 997.0;

    bool rendered = false;
    double currentTime = 0.0f;

    double dt = 0.001;
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

    hmm_vec2 getGridCenter();

    void createLevelSet();
    void updateLevelSet();

        template <typename Fun>
    void iterateU(Fun f) const {
        for (size_t j = 0; j < SIZEY; j++) {
            for (size_t i = 0; i < SIZEX + 1; i++) {
                f(i, j);
            }
        }
    }

    template <typename Fun>
    void iterateV(Fun f) {
        for (size_t j = 0; j < SIZEY + 1; j++) {
            for (size_t i = 0; i < SIZEX; i++) {
                f(i, j);
            }
        }
    }

    template <typename Fun>
    void iterate(Fun f) {
        for (size_t j = 0; j < SIZEY; j++) {
            for (size_t i = 0; i < SIZEX; i++) {
                f(i, j);
            }
        }
    }

    template <typename Fun>
    void iterateBackwards(Fun f) {
        for (size_t j = SIZEY; j-- > 0;) {
            for (size_t i = SIZEX; i-- > 0;) {
                f(i, j);
            }
        }
    }

    template <typename Fun>
    void fastSweepIterate(Fun f) {
        // Sweep with four possible directions, two times (to make sure)
        for (int i = 0; i < 2; i++) {
            for (size_t j = 0; j < SIZEY; j++) {
                for (size_t i = 0; i < SIZEX; i++) {
                    f(i, j);
                }
            }
            for (size_t j = 0; j < SIZEY; j++) {
                for (size_t i = SIZEX; i-- > 0;) {
                    f(i, j);
                }
            }
            for (size_t j = SIZEY; j-- > 0;) {
                for (size_t i = 0; i < SIZEX; i++) {
                    f(i, j);
                }
            }
            for (size_t j = SIZEY; j-- > 0;) {
                for (size_t i = SIZEX; i-- > 0;) {
                    f(i, j);
                }
            }
        }
    }
};


#endif //FLUID_SIM_WATERSIM2D_H
