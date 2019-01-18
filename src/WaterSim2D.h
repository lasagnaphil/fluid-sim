//
// Created by lasagnaphil on 2018-09-18.
//

#ifndef FLUID_SIM_WATERSIM2D_H
#define FLUID_SIM_WATERSIM2D_H

#include <cstdint>
#include <StackVec.h>
#include <vec2d.h>
#include "WaterSimSettings.h"

#include "Array2D.h"
#include "MACGrid2D.h"
#include "PerformanceCounter.h"

struct LevelSet {
    Array2D<double> phi;
    size_t sizeX;
    size_t sizeY;
    double dx;

    static LevelSet create(size_t sizeX, size_t sizeY, double dx) {
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

struct WaterSim2D {
    static constexpr int PARTICLES_PER_CELL_SQRT = WaterSimSettings::Dim2D::PARTICLES_PER_CELL_SQRT;
#define SQR(x) ((x)*(x))
    static constexpr int PARTICLES_PER_CELL = SQR(WaterSimSettings::Dim2D::PARTICLES_PER_CELL_SQRT);
#undef SQR

    enum class CellType : uint8_t {
        EMPTY, FLUID, SOLID
    };

    size_t sizeX;
    size_t sizeY;

    MACGrid2D mac;
    MACGrid2D newMac;
    Array2D<double> p;
    Array2D<CellType> cell;
    Vec<vec2d> particles = {};
    Vec<vec2d> particleVels = {};

    LevelSet waterLevelSet;

    vec2d gravity = {0, -9.81};
    vec2d origGravity = {0, -9.81};
    double rho = 997.0;

    bool oscillateGravity = WaterSimSettings::Dim2D::OSCILLATE_GRAVITY;
    float oscillateGravityAmp = WaterSimSettings::Dim2D::OSCILLATE_GRAVITY_AMP;
    float oscillateGravityPeriod = WaterSimSettings::Dim2D::OSCILLATE_GRAVITY_PERIOD;
    float picFlipAlpha = WaterSimSettings::Dim2D::PIC_FLIP_INTERP_ALPHA;

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

    using SimMode = WaterSimSettings::SimMode;
    SimMode mode = WaterSimSettings::Dim2D::SIM_MODE;

    PerformanceCounter perfCounter;

    double origWaterVolume = 0.0;
    double waterVolume = 0.0;
    Vec<double> waterVolumeData = {};

    double totalEnergy = 0.0;
    Vec<double> totalEnergyData = {};

    double particleTotalEnergy = 0.0;
    Vec<double> particleTotalEnergyData = {};

    void setup(size_t nx, size_t ny, double dt, double dx, double dr, double rho, double gravity);

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
    void iterateU(Fun f) const {
        for (size_t j = 0; j < sizeY; j++) {
            for (size_t i = 0; i < sizeX + 1; i++) {
                f(i, j);
            }
        }
    }

    template <typename Fun>
    void iterateV(Fun f) {
        for (size_t j = 0; j < sizeY + 1; j++) {
            for (size_t i = 0; i < sizeX; i++) {
                f(i, j);
            }
        }
    }

    template <typename Fun>
    void iterate(Fun f) {
        for (size_t j = 0; j < sizeY; j++) {
            for (size_t i = 0; i < sizeX; i++) {
                f(i, j);
            }
        }
    }

    template <typename Fun>
    void iterateBackwards(Fun f) {
        for (int j = sizeY; j >= 0; j--) {
            for (size_t i = sizeX; i-- > 0;) {
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
        for (size_t j = 0; j < sizeY; j++) {
            for (size_t i = 0; i < sizeX; i++) {
                f(i, j);
            }
        }
        for (size_t j = 0; j < sizeY; j++) {
            for (size_t i = sizeX; i-- > 0;) {
                f(i, j);
            }
        }
        for (size_t j = sizeY; j-- > 0; ) {
            for (size_t i = 0; i < sizeX; i++) {
                f(i, j);
            }
        }
        for (size_t j = sizeY; j-- > 0; ) {
            for (size_t i = sizeX; i-- > 0;) {
                f(i, j);
            }
        }
    }
}

#endif //FLUID_SIM_WATERSIM2D_H
