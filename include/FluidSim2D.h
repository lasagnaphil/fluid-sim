//
// Created by lasagnaphil on 2018-09-18.
//

#ifndef FLUID_SIM_FLUIDSIM2D_H
#define FLUID_SIM_FLUIDSIM2D_H

#include <cstdint>
#include <StackVec.h>
#include <vec2d.h>
#include "Array2D.h"
#include "MACGrid2D.h"
#include "PerformanceCounter.h"

struct LevelSet {
    Array2D<double> phi;
    int sizeX;
    int sizeY;
    double dx;

    static LevelSet create(int sizeX, int sizeY, double dx) {
        return {
            Array2D<double>::create(sizeX, sizeY),
            sizeX,
            sizeY,
            dx
        };
    }

    void free() {
        phi.free();
    }

    void constructFromParticles(Vec<vec2d> particles, double dr);
    void redistance();

    template <typename Fun> void fastSweepIterate(Fun f);
};

enum FluidSimMode {
    FS_SEMILAGRANGIAN, FS_PICFLIP
};

enum FluidCellType : uint8_t {
    FS_EMPTY, FS_FLUID, FS_SOLID
};

struct FluidSim2DConfig {
    int sizeX;
    int sizeY;
    int particlesPerCellSqrt;

    double dt;
    double dx;
    double rho;
    double gravityX;
    double gravityY;

    FluidSimMode mode;
    double picFlipAlpha;

    FluidCellType* initialValues;
};

struct FluidSim2D {
    int sizeX;
    int sizeY;
    int particlesPerCellSqrt;
    int particlesPerCell;

    MACGrid2D mac;
    MACGrid2D newMac;
    Array2D<double> p;
    Array2D<FluidCellType> cell;
    Vec<vec2d> particles = {};
    Vec<vec2d> particleVels = {};

    LevelSet waterLevelSet;

    vec2d gravity = {0, -9.81};
    vec2d origGravity = {0, -9.81};
    double rho = 997.0;

    double picFlipAlpha;

    bool rendered = false;
    double currentTime = 0.0f;

    double dt = 0.0001;
    double dx = 0.001;
    double dr = 0.0009; // water particle radius

    enum class StageType {
        Init, CreateWaterLevelSet, TransferVelocityToGrid,
        ApplySemiLagrangianAdvection, ApplyGravity, CreateSolidLevelSet, ApplyProjection,
        UpdateVelocity, UpdateParticleVelocities, ApplyAdvection
    };
    int numStages;

    StageType stage = StageType::Init;

    FluidSimMode mode;

    PerformanceCounter perfCounter;

    double origWaterVolume = 0.0;
    double waterVolume = 0.0;
    Vec<double> waterVolumeData = {};

    double totalEnergy = 0.0;
    Vec<double> totalEnergyData = {};

    double particleTotalEnergy = 0.0;
    Vec<double> particleTotalEnergyData = {};

    static FluidSim2D create(const FluidSim2DConfig& config);

    void free();

    void runFrame();

    void update();

    void createWaterLevelSet();

    void applySemiLagrangianAdvection();

    void transferVelocityToGrid();

    void applyGravity();

    void createSolidLevelSet();

    void applyProjection();

    void updateVelocity();

    void updateParticleVelocities();

    void applyAdvection();

    double avgPressure();
    double avgPressureInFluid();
    double maxVelocity();

    vec2d getGridCenter();

    void saveStats();

    vec2d clampPos(vec2d to);

    template <typename Fun>
    void iterate(Fun f) {
        for (int j = 0; j < sizeY; j++) {
            for (int i = 0; i < sizeX; i++) {
                f(i, j);
            }
        }
    }

    const char* printStage() {
        switch(stage) {
            case StageType::Init: return "Init";
            case StageType::CreateWaterLevelSet: return "CreateWaterLevelSet";
            case StageType::TransferVelocityToGrid: return "TransferVelocityToGrid";
            case StageType::ApplySemiLagrangianAdvection: return "ApplySemiLagrangianAdvection";
            case StageType::ApplyGravity: return "ApplyGravity";
            case StageType::CreateSolidLevelSet: return "CreateSolidLevelSet";
            case StageType::ApplyProjection: return "ApplyProjection";
            case StageType::UpdateVelocity: return "UpdateVelocity";
            case StageType::UpdateParticleVelocities: return "UpdateParticleVelocities";
            case StageType::ApplyAdvection: return "ApplyAdvection";
            default: return nullptr;
        }
    }
};

template<typename Fun>
void LevelSet::fastSweepIterate(Fun f) {
    // Sweep with four possible directions, two times (to make sure)
    for (int k = 0; k < 4; k++) {
        for (int j = 0; j < sizeY; j++) {
            for (int i = 0; i < sizeX; i++) {
                f(i, j);
            }
        }
        for (int j = 0; j < sizeY; j++) {
            for (int i = sizeX; i-- > 0;) {
                f(i, j);
            }
        }
        for (int j = sizeY; j-- > 0; ) {
            for (int i = 0; i < sizeX; i++) {
                f(i, j);
            }
        }
        for (int j = sizeY; j-- > 0; ) {
            for (int i = sizeX; i-- > 0;) {
                f(i, j);
            }
        }
    }
}

#endif //FLUID_SIM_FLUIDSIM2D_H
