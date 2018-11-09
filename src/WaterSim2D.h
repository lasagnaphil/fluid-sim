//
// Created by lasagnaphil on 2018-09-18.
//

#ifndef FLUID_SIM_WATERSIM2D_H
#define FLUID_SIM_WATERSIM2D_H

#include <cstdint>
#include <StackVec.h>
#include <mathfu/glsl_mappings.h>
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
    Vec<mathfu::vec2d> particles = {};
    Array2D<double, SIZEX, SIZEY> phi = {};
    Vec<mathfu::vec2d> particleVels = {};

    double gravity = -9.81;
    double rho = 997.0;

    bool rendered = false;
    double currentTime = 0.0f;

    double dt = 0.0001;
    double dx = 0.001;
    double dr = 0.0009; // water particle radius

    enum class StageType {
        Init, CreateLevelSet, UpdateLevelSet, TransferVelocityToGrid,
        ApplySemiLagrangianAdvection, ApplyGravity, ApplyProjection,
        UpdateVelocity, UpdateParticleVelocities, ApplyAdvection
    };

    enum class SimMode {
        Eulerian, PIC
    };

    StageType stage = StageType::Init;
    SimMode mode = SimMode::PIC;

    void setup(double dt, double dx, double rho, double gravity);

    void runFrame();

    void update();

    void applySemiLagrangianAdvection();

    void transferVelocityToGrid();

    void applyGravity();

    void applyProjection();

    void updateVelocity();

    void updateParticleVelocities();

    void applyAdvection();

    mathfu::vec2d clampPos(mathfu::vec2d pos);

    double avgPressure();
    double avgPressureInFluid();

    mathfu::vec2d getGridCenter();

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
        for (int k = 0; k < 2; k++) {
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

    const char* printStage() {
        switch(stage) {
            case StageType::Init: return "Init";
            case StageType::CreateLevelSet: return "CreateLevelSet";
            case StageType::UpdateLevelSet: return "UpdateLevelSet";
            case StageType::TransferVelocityToGrid: return "TransferVelocityToGrid";
            case StageType::ApplySemiLagrangianAdvection: return "ApplySemiLagrangianAdvection";
            case StageType::ApplyGravity: return "ApplyGravity";
            case StageType::ApplyProjection: return "ApplyProjection";
            case StageType::UpdateVelocity: return "UpdateVelocity";
            case StageType::UpdateParticleVelocities: return "UpdateParticleVelocities";
            case StageType::ApplyAdvection: return "ApplyAdvection";
            default: return nullptr;
        }
    }
};


#endif //FLUID_SIM_WATERSIM2D_H
