//
//
// Created by lasagnaphil on 2018-09-18.
//

#include <ctime>
#include <Defer.h>
#include <Queue.h>
#include <math/Utils.h>
#include <log.h>
#include <File.h>
#include "WaterSim2D.h"
#include "InputManager.h"

#define USE_GHOST_PRESSURE
#define USE_LEVEL_SET
#define USE_RESEEDING

using namespace mathfu;

inline double randf(float max) {
    return ((double)rand()/(double)RAND_MAX) * max;
}

void WaterSim2D::setup(double dt, double dx, double dr, double rho, double gravity) {

    if (mode == SimMode::SemiLagrangian) {
        numStages = 7;
    }
    else if (mode == SimMode::PIC) {
        numStages = 8;
    }

    perfCounter = PerformanceCounter::create(numStages);
    this->dt = dt;
    this->dx = mac.dx = dx;
    this->dr = dr;
    this->rho = rho;
    this->gravity = {0, gravity};
    this->origGravity = this->gravity;

    size_t fluidCount = 0;

    int initState = WaterSimSettings::Dim2D::INIT_STATE;
    if (initState == 0) {
        iterate([&](size_t i, size_t j) {
            if (i == 0 || i == SIZEX - 1 || j == 0 || j == SIZEX - 1) {
                cell(i, j) = CellType::SOLID;
            }
            else if (i < SIZEX/2) {
                if (j <= i/2 + SIZEY/4) {
                    cell(i, j) = CellType::FLUID;
                    fluidCount++;
                }
                else {
                    cell(i, j) = CellType::EMPTY;
                }
            }
            else {
                if (j <= 3*SIZEY/4 - i/2) {
                    cell(i, j) = CellType::FLUID;
                    fluidCount++;
                }
                else {
                    cell(i, j) = CellType::EMPTY;
                }
            }
        });
    }
    else if (initState == 1) {
        iterate([&](size_t i, size_t j) {
            if (i == 0 || i == SIZEX - 1 ||
                j == 0 || j == SIZEY - 1) {
                cell(i, j) = CellType::SOLID;
            }
            else if (i + j < SIZEY * 3 / 4) {
                cell(i, j) = CellType::FLUID;
                fluidCount++;
            }
            else {
                cell(i, j) = CellType::EMPTY;
            }
        });
    }
    else if (initState == 2) {
        iterate([&](size_t i, size_t j) {
            if(i == 0 || i == SIZEX - 1 || j == 0 || j == SIZEX - 1) {
                cell(i, j) = CellType::SOLID;
            }
            else if (i >= SIZEX/4 && i < 3*SIZEX/4 && j >= 1*SIZEY/8 && j < 5*SIZEY/8) {
                cell(i, j) = CellType::FLUID;
                fluidCount++;
            }
            else {
                cell(i, j) = CellType::EMPTY;
            }
        });
    }

    particles.reserve(PARTICLES_PER_CELL*fluidCount);
    iterate([&](size_t i, size_t j) {
        if (cell(i,j) == CellType::FLUID) {
            for (int k = 0; k < PARTICLES_PER_CELL; k++) {
                constexpr float dist = 1.0f / PARTICLES_PER_CELL_SQRT;
                int x = k % PARTICLES_PER_CELL_SQRT;
                int y = k / PARTICLES_PER_CELL_SQRT;
                particles.push(vec2d(((double) i + dist * x + randf(dist)) * dx, ((double) j + dist * y + randf(dist)) * dx));
            }
        }
    });

    particleVels.resize(PARTICLES_PER_CELL*fluidCount);
    for (int i = 0; i < PARTICLES_PER_CELL*fluidCount; i++) {
        particleVels[i] = vec2d(0.0, 0.0);
    }

    waterVolume = fluidCount * dx * dx;
    origWaterVolume = waterVolume;
    waterLevelSet = LevelSet::create(dx);
}

void WaterSim2D::free() {
    perfCounter.free();
    waterVolumeData.free();
}

void WaterSim2D::runFrame() {
#define STOPWATCH(__CODE) \
    perfCounter.beginStage(); \
    __CODE; \
    perfCounter.endStage();

    if (mode == SimMode::SemiLagrangian) {
        stage = StageType::CreateWaterLevelSet;
        STOPWATCH(createWaterLevelSet());
        stage = StageType::ApplySemiLagrangianAdvection;
        STOPWATCH(applySemiLagrangianAdvection());
        stage = StageType::ApplyGravity;
        STOPWATCH(applyGravity());
        stage = StageType::CreateSolidLevelSet;
        STOPWATCH(createSolidLevelSet());
        stage = StageType::ApplyProjection;
        STOPWATCH(applyProjection());
        stage = StageType::UpdateVelocity;
        STOPWATCH(updateVelocity());
        stage = StageType::ApplyAdvection;
        STOPWATCH(applyAdvection());
    }
    else if (mode == SimMode::PIC) {
        stage = StageType::CreateWaterLevelSet;
        STOPWATCH(createWaterLevelSet());
        stage = StageType::TransferVelocityToGrid;
        STOPWATCH(transferVelocityToGrid());
        stage = StageType::ApplyGravity;
        STOPWATCH(applyGravity());
        stage = StageType::CreateSolidLevelSet;
        STOPWATCH(createSolidLevelSet());
        stage = StageType::ApplyProjection;
        STOPWATCH(applyProjection());
        stage = StageType::UpdateVelocity;
        STOPWATCH(updateVelocity());
        stage = StageType::UpdateParticleVelocities;
        STOPWATCH(updateParticleVelocities());
        stage = StageType::ApplyAdvection;
        STOPWATCH(applyAdvection());
    }
    rendered = false;
    currentTime += dt;
    perfCounter.endFrame();
#undef STOPWATCH
}

void WaterSim2D::update() {
    auto inputMgr = InputManager::get();
    /*
    if (inputMgr->isKeyEntered(SDL_SCANCODE_RETURN)) {
        if (mode == SimMode::SemiLagrangian) {
            if (stage == StageType::Init || stage == StageType::ApplyAdvection) {
                stage = StageType::CreateLevelSet;
                createLevelSet();
            } else if (stage == StageType::CreateLevelSet) {
                stage = StageType::UpdateLevelSet;
                updateLevelSet();
            } else if (stage == StageType::UpdateLevelSet) {
                stage = StageType::ApplySemiLagrangianAdvection;
                applySemiLagrangianAdvection();
            } else if (stage == StageType::ApplySemiLagrangianAdvection) {
                stage = StageType::ApplyGravity;
                applyGravity();
            } else if (stage == StageType::ApplyGravity) {
                stage = StageType::ApplyProjection;
                applyProjection();
                updateVelocity();
            } else if (stage == StageType::ApplyProjection) {
                stage = StageType::UpdateVelocity;
                updateVelocity();
            } else if (stage == StageType::UpdateVelocity) {
                stage = StageType::ApplyAdvection;
                applyAdvection();
                currentTime += dt;
            }
            rendered = false;
        }
        else if (mode == SimMode::PIC) {
            if (stage == StageType::Init || stage == StageType::ApplyAdvection) {
                stage = StageType::CreateLevelSet;
                createLevelSet();
            } else if (stage == StageType::CreateLevelSet) {
                stage = StageType::UpdateLevelSet;
                updateLevelSet();
            } else if (stage == StageType::UpdateLevelSet) {
                stage = StageType::TransferVelocityToGrid;
                transferVelocityToGrid();
            } else if (stage == StageType::TransferVelocityToGrid) {
                stage = StageType::ApplyGravity;
                applyGravity();
            } else if (stage == StageType::ApplyGravity) {
                stage = StageType::ApplyProjection;
                applyProjection();
            } else if (stage == StageType::ApplyProjection) {
                stage = StageType::UpdateVelocity;
                updateVelocity();
            } else if (stage == StageType::UpdateVelocity) {
                stage = StageType::UpdateParticleVelocities;
                updateParticleVelocities();
            } else if (stage == StageType::UpdateParticleVelocities) {
                stage = StageType::ApplyAdvection;
                applyAdvection();
                currentTime += dt;
            }
            rendered = false;
        }
    }
     */
    runFrame();
    /*
    if (inputMgr->isKeyEntered(SDL_SCANCODE_RETURN)) {
        runFrame();
    }
     */
}

void WaterSim2D::transferVelocityToGrid() {
    mac.u.reset();
    mac.v.reset();
    auto udiv = new Array2D<double, SIZEX+1, SIZEY>();
    auto vdiv = new Array2D<double, SIZEX, SIZEY+1>();
    for (int e = 0; e < particles.size; e++) {
        auto xp = particles[e];
        auto vp = particleVels[e];
        auto upos = vec2d(xp.x / dx, xp.y / dx - 0.5);
        mac.u.linearDistribute(upos, vp.x);
        udiv->linearDistribute(upos, 1.0);
        auto vpos = vec2d(xp.x / dx - 0.5, xp.y / dx);
        mac.v.linearDistribute(vpos, vp.y);
        vdiv->linearDistribute(vpos, 1.0);
    }
#pragma omp parallel for
    for (size_t j = 0; j < SIZEY; j++) {
        for (size_t i = 0; i < SIZEX + 1; i++) {
            if ((*udiv)(i, j) > 0) mac.u(i, j) /= (*udiv)(i, j);
        }
    }
#pragma omp parallel for
    for (size_t j = 0; j < SIZEY + 1; j++) {
        for (size_t i = 0; i < SIZEX; i++) {
            if ((*vdiv)(i, j) > 0) mac.v(i, j) /= (*vdiv)(i, j);
        }
    }

    auto uintFlag = new Array2D<uint32_t, SIZEX+1, SIZEY>();
    auto vintFlag = new Array2D<uint32_t, SIZEX, SIZEY+1>();

#pragma omp parallel for
    for (size_t j = 0; j < SIZEY; j++) {
        for (size_t i = 0; i < SIZEX + 1; i++) {
            (*uintFlag)(i,j) = mac.u(i,j) == 0.0? UINT32_MAX : 0;
        }
    }
#pragma omp parallel for
    for (size_t j = 0; j < SIZEY + 1; j++) {
        for (size_t i = 0; i < SIZEX; i++) {
            (*vintFlag)(i,j) = mac.v(i,j) == 0.0? UINT32_MAX : 0;
        }
    }
    mac.u.extrapolate(*uintFlag);
    mac.v.extrapolate(*vintFlag);
    delete uintFlag;
    delete vintFlag;
    delete udiv;
    delete vdiv;
}

void WaterSim2D::applySemiLagrangianAdvection() {
    double C = 5;

#pragma omp parallel for
    for (int j = 0; j < SIZEY; j++) {
        for (int i = 0; i < SIZEX + 1; i++) {
            vec2d x_p = vec2d((double)i, ((double)j + 0.5)) * dx;
            auto k1 = mac.velInterp(x_p);
            auto k2 = mac.velInterp(x_p - 0.5*dt*k1);
            auto k3 = mac.velInterp(x_p - 0.75*dt*k2);
            auto x_p_new = x_p - ((2./9.)*dt*k1 + (3./9.)*dt*k2 + (4./9.)*dt*k3);
            x_p_new = clampPos(x_p, x_p_new);
            mac.u(i,j) = mac.velInterpU(x_p_new);
        }
    }

#pragma omp parallel for
    for (int j = 0; j < SIZEY + 1; j++) {
        for (int i = 0; i < SIZEX; i++) {
            vec2d x_p = vec2d((double)i + 0.5, ((double)j)) * dx;
            auto k1 = mac.velInterp(x_p);
            auto k2 = mac.velInterp(x_p - 0.5*dt*k1);
            auto k3 = mac.velInterp(x_p - 0.75*dt*k2);
            auto x_p_new = x_p - ((2./9.)*dt*k1 + (3./9.)*dt*k2 + (4./9.)*dt*k3);
            x_p_new = clampPos(x_p, x_p_new);
            mac.v(i,j) = mac.velInterpV(x_p_new);
        }
    }

}

void WaterSim2D::applyGravity() {
    static float t = 0.0f;
    t += dt;
    if (oscillateGravity) {
        gravity.x = oscillateGravityAmp * sin(2*M_PI* t / oscillateGravityPeriod);
    }
#pragma omp parallel for
    for (int j = 0; j < SIZEY; j++) {
        for (int i = 0; i < SIZEX + 1; i++) {
            mac.u(i,j) += dt * gravity.x;
        }
    }
#pragma omp parallel for
    for (int j = 0; j < SIZEY + 1; j++) {
        for (int i = 0; i < SIZEX; i++) {
            mac.v(i,j) += dt * gravity.y;
        }
    }
}

void WaterSim2D::applyProjection() {
    auto gridStack = Vec<Grid2D<double>>::create(9);

    auto& Adiag = gridStack.newItem();
    auto& Ax = gridStack.newItem();
    auto& Ay = gridStack.newItem();

    // calculate lhs (matrix A)
    double scaleA = dt / (rho * dx * dx);
#ifdef USE_GHOST_PRESSURE
#pragma omp parallel for
    for (size_t j = 0; j < SIZEY; j++) {
        for (size_t i = 0; i < SIZEX; i++) {
            if (cell(i,j) == CellType::FLUID) {
                if (cell(i-1,j) == CellType::FLUID) {
                    Adiag(i,j) += scaleA;
                }
                else if (cell(i-1,j) == CellType::EMPTY) {
                    double phi1 = waterLevelSet.phi(i-1, j);
                    double phi0 = waterLevelSet.phi(i, j);
                    Adiag(i,j) -= scaleA * utils::max(phi1 / phi0, -1e3);
                }
                if (cell(i+1,j) == CellType::FLUID) {
                    Adiag(i,j) += scaleA;
                    Ax(i,j) = -scaleA;
                }
                else if (cell(i+1,j) == CellType::EMPTY) {
                    double phi1 = waterLevelSet.phi(i+1, j);
                    double phi0 = waterLevelSet.phi(i, j);
                    Adiag(i,j) += scaleA * (1 - utils::max(phi1 / phi0, -1e3));
                }
                if (cell(i,j-1) == CellType::FLUID) {
                    Adiag(i,j) += scaleA;
                }
                else if (cell(i,j-1) == CellType::EMPTY) {
                    double phi1 = waterLevelSet.phi(i, j-1);
                    double phi0 = waterLevelSet.phi(i, j);
                    Adiag(i,j) -= scaleA * utils::max(phi1 / phi0, -1e3);
                }
                if (cell(i,j+1) == CellType::FLUID) {
                    Adiag(i,j) += scaleA;
                    Ay(i,j) = -scaleA;
                }
                else if (cell(i,j+1) == CellType::EMPTY) {
                    double phi1 = waterLevelSet.phi(i, j+1);
                    double phi0 = waterLevelSet.phi(i, j);
                    Adiag(i,j) += scaleA * (1 - utils::max(phi1 / phi0, -1e3));
                }
            }
        }
    }
#else
#pragma omp parallel for
    for (size_t j = 0; j < SIZEY; j++) {
        for (size_t i = 0; i < SIZEX; i++) {
            if (cell(i,j) == CellType::FLUID) {
                if (cell(i-1,j) == CellType::FLUID) {
                    Adiag(i,j) += scaleA;
                }
                if (cell(i+1,j) == CellType::FLUID) {
                    Adiag(i,j) += scaleA;
                    Ax(i,j) = -scaleA;
                }
                else if (cell(i+1,j) == CellType::EMPTY) {
                    Adiag(i,j) += scaleA;
                }
                if (cell(i,j-1) == CellType::FLUID) {
                    Adiag(i,j) += scaleA;
                }
                if (cell(i,j+1) == CellType::FLUID) {
                    Adiag(i,j) += scaleA;
                    Ay(i,j) = -scaleA;
                }
                else if (cell(i,j+1) == CellType::EMPTY) {
                    Adiag(i,j) += scaleA;
                }
            }
        }
    }
#endif

    auto& rhs = gridStack.newItem();

    // calculate rhs
    {
        double scale = 1.0 / dx;
#pragma omp parallel for
        for (size_t j = 0; j < SIZEY; j++) {
            for (size_t i = 0; i < SIZEX; i++) {
                if (cell(i,j) == CellType::FLUID) {
                    rhs(i,j) = -scale * (mac.u(i+1,j)-mac.u(i,j)+mac.v(i,j+1)-mac.v(i,j));
                    // modify rhs to account for solid velocities
                    // TODO: use usolid, vsolid, wsolid instead of 0
                    if (cell(i-1,j) == CellType::SOLID) {
                        rhs(i,j) -= scale * (mac.u(i,j) - 0);
                    }
                    if (cell(i+1,j) == CellType::SOLID) {
                        rhs(i,j) += scale * (mac.u(i+1,j) - 0);
                    }
                    if (cell(i,j-1) == CellType::SOLID) {
                        rhs(i,j) -= scale * (mac.v(i,j) - 0);
                    }
                    if (cell(i,j+1) == CellType::SOLID) {
                        rhs(i,j) += scale * (mac.v(i,j+1) - 0);
                    }
                }
            }
        }
    }

#define SQUARE(x) (x)*(x)
    // find preconditioner
    auto& precon = gridStack.newItem();
    {
        constexpr double tau = 0.999;
        constexpr double sigma = 0.25;
        double e;
#pragma omp parallel for
        for (size_t j = 1; j < SIZEY; j++) {
            for (size_t i = 1; i < SIZEX; i++) {
                if (cell(i, j) == CellType::FLUID) {
                    e = Adiag(i,j)
                        - SQUARE(Ax(i-1,j) * precon(i-1,j))
                        - SQUARE(Ay(i,j-1) * precon(i,j-1))
                        - tau * (Ax(i-1,j)*Ay(i-1,j)*SQUARE(precon(i-1,j)) + Ay(i,j-1)*Ax(i,j-1)*SQUARE(precon(i,j-1)));
                    if (e < sigma * Adiag(i,j)) {
                        e = Adiag(i,j);
                    }
                    precon(i,j) = 1 / sqrt(e);
                }
            }
        }
    }
#undef SQUARE

    auto& r = gridStack.newItem();
    auto& z = gridStack.newItem();
    auto& s = gridStack.newItem();

    auto applyPreconditioner = [&]() {
        // apply preconditioner
        auto& q = gridStack.newItem();
        for (size_t j = 1; j < SIZEY; j++) {
            for (size_t i = 1; i < SIZEX; i++) {
                if (cell(i,j) == CellType::FLUID) {
                    double t = r(i,j)
                            - Ax(i-1,j) * precon(i-1,j) * q(i-1,j)
                            - Ay(i,j-1) * precon(i,j-1) * q(i,j-1);
                    q(i,j) = t * precon(i,j);
                }
            }
        }
        for (size_t j = SIZEY - 1; j-- > 0; ) {
            for (size_t i = SIZEX - 1; i-- > 0; ) {
                if (cell(i,j) == CellType::FLUID) {
                    double t = q(i,j)
                            - Ax(i,j) * precon(i,j) * z(i+1,j)
                            - Ay(i,j) * precon(i,j) * z(i,j+1);
                    z(i,j) = t * precon(i,j);
                }
            }
        }
        gridStack.pop();
    };

    // use PCG algorithm to solve the linear equation
    p.reset();
    r.copyFrom(rhs);
    applyPreconditioner();
    s.copyFrom(z);
    double sigma = z.innerProduct(r);
    int maxIters = 200;
    int iter = 0;
    while (iter < maxIters) {
        // apply A
#pragma omp parallel for
        for (size_t j = 0; j < SIZEY; j++) {
            for (size_t i = 0; i < SIZEX; i++) {
                if (cell(i, j) == CellType::FLUID) {
                    z(i, j) = Adiag(i, j) * s(i, j)
                              + Ax(i - 1, j) * s(i - 1, j)
                              + Ax(i, j) * s(i + 1, j)
                              + Ay(i, j - 1) * s(i, j - 1)
                              + Ay(i, j) * s(i, j + 1);
                }
            }
        }

        // Note: this code is needed to handle when rhs = 0 (No boundary conditions) -> p = 0 everywhere
        double rhsNorm = rhs.infiniteNorm();
        if (rhsNorm <= 1e-12) { break; }

        double alpha = sigma / z.innerProduct(s);
        p.setMultiplyAdd(p, alpha, s);
        r.setMultiplyAdd(r, -alpha, z);
        if (r.infiniteNorm() <= 1e-12 * rhsNorm) { break; }

        applyPreconditioner();

        double sigma_new = z.innerProduct(r);
        double beta = sigma_new / sigma;
        s.setMultiplyAdd(z, beta, s);
        sigma = sigma_new;

        iter++;
    }
    if (iter == maxIters) {
        log_error("Maximum iteration limit exceeded!");
    }

    gridStack.free();
}

void WaterSim2D::updateVelocity() {
    // update velocity using the solved pressure
    auto du = new Array2D<uint32_t, SIZEX+1, SIZEY>;
    defer {delete du;};
    auto dv = new Array2D<uint32_t, SIZEX, SIZEY+1>;
    defer {delete dv;};
    newMac = mac;
    {
        double scale = dt / (rho * dx);
#pragma omp parallel for
        for (size_t j = 0; j < SIZEY; j++) {
            for (size_t i = 0; i < SIZEX; i++) {
                if ((i > 0 && cell(i-1,j) == CellType::FLUID) || cell(i,j) == CellType::FLUID) {
                    if ((i == 0 || cell(i-1,j) == CellType::SOLID) || cell(i,j) == CellType::SOLID) {
                        newMac.u(i,j) = 0; // usolid(i,j);
                    }
#ifdef USE_GHOST_PRESSURE
                    else if (i > 0 && cell(i-1,j) == CellType::EMPTY) {
                        // account for ghost pressures
                        double phi1 = waterLevelSet.phi(i-1, j);
                        double phi0 = waterLevelSet.phi(i, j);
                        newMac.u(i,j) -= scale * (1 - utils::max(phi1/phi0, -1e3)) * p(i,j);
                    }
                    else if (cell(i,j) == CellType::EMPTY) {
                        // account for ghost pressures
                        double phi1 = waterLevelSet.phi(i, j);
                        double phi0 = waterLevelSet.phi(i-1, j);
                        newMac.u(i,j) -= scale * (utils::max(phi1/phi0, -1e3) - 1) * p(i-1,j);
                    }
#endif
                    else if (i > 0) {
                        newMac.u(i,j) -= scale * (p(i,j) - p(i-1,j));
                    }
                    else {
                        newMac.u(i,j) = 0; // usolid(0,j)
                    }
                }
                else {
                    // mark as unknown?
                    (*du)(i,j) = UINT32_MAX;
                }
                if ((j > 0 && cell(i,j-1) == CellType::FLUID) || cell(i,j) == CellType::FLUID) {
                    if ((j == 0 || cell(i,j-1) == CellType::SOLID) || cell(i,j) == CellType::SOLID) {
                        newMac.v(i,j) = 0; // vsolid(i,0)
                    }
#ifdef USE_GHOST_PRESSURE
                    else if (j > 0 && cell(i,j-1) == CellType::EMPTY) {
                        // account for ghost pressures
                        double phi1 = waterLevelSet.phi(i, j-1);
                        double phi0 = waterLevelSet.phi(i, j);
                        newMac.v(i,j) -= scale * (1 - utils::max(phi1/phi0, -1e3)) * p(i,j);
                    }
                    else if (cell(i,j) == CellType::EMPTY) {
                        // account for ghost pressures
                        double phi1 = waterLevelSet.phi(i, j);
                        double phi0 = waterLevelSet.phi(i, j-1);
                        newMac.v(i,j) -= scale * (utils::max(phi1/phi0, -1e3) - 1) * p(i,j-1);
                    }
#endif
                    else if (j > 0) {
                        newMac.v(i,j) -= scale * (p(i,j) - p(i,j-1));
                    }
                    else {
                        newMac.v(i,j) = 0; // vsolid(i,0)
                    }
                }
                else {
                    // mark as unknown?
                    (*dv)(i,j) = UINT32_MAX;
                }
            }
        }
    }

    newMac.u.extrapolate(*du);
    newMac.v.extrapolate(*dv);

    if (mode == SimMode::SemiLagrangian) {
        mac = newMac;
    }
}

void WaterSim2D::updateParticleVelocities() {
    // PIC update
    Array2D<double, SIZEX+1, SIZEY> uDiff = newMac.u - mac.u;
    Array2D<double, SIZEX, SIZEY+1> vDiff = newMac.v - mac.v;

#pragma omp parallel for
    for (int e = 0; e < particles.size; e++) {
        vec2d upos = vec2d(particles[e].x / dx, particles[e].y / dx - 0.5);
        vec2d vpos = vec2d(particles[e].x / dx - 0.5, particles[e].y / dx);
        vec2d picVel = {newMac.u.linearExtract(upos), newMac.v.linearExtract(vpos)};
        vec2d flipVel = particleVels[e] + vec2d {uDiff.linearExtract(upos), vDiff.linearExtract(vpos)};
        particleVels[e] = (double)picFlipAlpha * picVel + (1 - (double)picFlipAlpha) * flipVel;
    }

    mac = newMac;
}

void WaterSim2D::applyAdvection() {
    // Change timestep to apply CFL condition
    double cmax = 0.0;
    int emax = -1;
    for (int e = 0; e < particles.size; e++) {
        vec2d vel = particleVels[e];
        double c = (vel.x + vel.y) * dt / dx;
        if (c > cmax) {
            cmax = c;
            emax = e;
        }
    }
    if (cmax > 5.0) {
        log_warn("Particle velocity violates CFL condition! (C=%f, vel=%f, pos=(%f, %f))",
                cmax, particles[emax].Length(), particles[emax].x, particles[emax].y);
    }


    #pragma omp parallel for
    for (int i = 0; i < particles.size; i++) {
        auto& pos = particles[i];
        auto k1 = mac.velInterp(pos);
        auto k2 = mac.velInterp(pos + 0.5*dt*k1);
        auto k3 = mac.velInterp(pos + 0.75*dt*k2);

        auto nextPos = pos + (2./9.)*dt*k1 + (3./9.)*dt*k2 + (4./9.)*dt*k3;
        nextPos = clampPos(pos, nextPos);

        if (isnan(pos.x) || isnan(pos.y)) {
            log_error("Advected position is NaN! pos=(%d, %d), idx=%d", pos.x, pos.y, i);
            exit(1);
        }
        pos = nextPos;
    }
}


double WaterSim2D::avgPressure() {
    double avgP = 0.0f;
    iterate([&](size_t i, size_t j) {
        avgP += p(i,j);
    });
    avgP /= (SIZEX*SIZEY);
    return avgP;
}

double WaterSim2D::avgPressureInFluid() {
    double avgP = 0.0f;
    size_t count = 0;
    iterate([&](size_t i, size_t j) {
        if (cell(i,j) == CellType::FLUID) {
            avgP += p(i,j);
            count++;
        }
    });
    avgP /= count;
    return avgP;
}

double WaterSim2D::maxVelocity() {
    double maxVel = 0.0f;
    iterate([&](size_t i, size_t j) {
        if (cell(i,j) == CellType::FLUID) {
            double vel = mac.velInterp(vec2d(i,j)*dx).Length();
            if (vel > maxVel) maxVel = vel;
        }
    });
    return maxVel;
}

vec2d WaterSim2D::getGridCenter() {
    return vec2d(SIZEX * dx / 2, SIZEY * dx / 2);
}


mathfu::vec2d WaterSim2D::clampPos(mathfu::vec2d from, mathfu::vec2d to) {
    double offset = 1e-3;
    vec2d clamped = {
        utils::clamp<double>(to.x, (1.0 + offset) * dx, (SIZEX-1.0 - offset)*dx),
        utils::clamp<double>(to.y, (1.0 + offset) * dx, (SIZEY-1.0 - offset)*dx)
    };
    return clamped;
    /*
    if (to.x != from.x) {
        if (to.x < dx) {
            double newX = (1 + 1e-6)*dx;
            to.y = from.y + (to.y - from.y) * (newX - from.x) / (to.x - from.x);
            to.x = newX;
        }
        else if (to.x >= (SIZEX-1)*dx) {
            double newX = (SIZEX-1 - 1e-6)*dx;
            to.y = from.y + (to.y - from.y) * (newX - from.x) / (to.x - from.x);
            to.x = newX;
        }
    }
    if (to.y != from.y) {
        if (to.y < dx) {
            double newY = (1 + 1e-6)*dx;
            to.x = from.x + (to.x - from.x) * (newY - from.y) / (to.y - from.y);
            to.y = newY;
        }
        else if (to.y >= (SIZEY-1)*dx) {
            double newY = (SIZEY-1 - 1e-6)*dx;
            to.x = from.x + (to.x - from.x) * (newY - from.y) / (to.y - from.y);
            to.y = newY;
        }
    }
     */
    return to;
}

void WaterSim2D::createWaterLevelSet() {
#ifdef USE_LEVEL_SET
    waterLevelSet.constructFromParticles(particles, dr);
    waterLevelSet.redistance();
    for (size_t j = 0; j < SIZEY; j++) {
        for (size_t i = 0; i < SIZEX; i++) {
            if (cell(i, j) != CellType::SOLID) {
                double phi = waterLevelSet.phi(i, j);
                cell(i, j) = phi < 0.0? CellType::FLUID : CellType::EMPTY;
            }
        }
    }
#else

#pragma omp parallel for
    for (size_t j = 0; j < SIZEY; j++) {
        for (size_t i = 0; i < SIZEX; i++) {
            if(cell(i,j) == CellType::FLUID) {
                cell(i,j) = CellType::EMPTY;
            }
        }
    }

#pragma omp parallel for
    for (int i = 0; i < particles.size; i++) {
        auto pos = particles[i];
        int x = (int)pos.x; int y = (int)pos.y;
        if (cell(x, y) == CellType::EMPTY) {
            cell(x, y) = CellType::FLUID;
        }
    }

    // Correction to eliminate "bubbles" inside fluids (and on wall boundaries)
#pragma omp parallel for
    for (size_t i = 1; i < SIZEX - 1; i++) {
        for (size_t j = 1; j < SIZEY - 1; j++) {
            if (cell(i, j) == CellType::EMPTY) {
                int fluidCount = 0;
                int solidCount = 0;
                for (auto neighbor : {cell(i-1, j), cell(i+1, j), cell(i,j-1), cell(i,j+1)}) {
                    if (neighbor == CellType::FLUID) {
                        fluidCount++;
                    }
                    else if (neighbor == CellType::SOLID) {
                        solidCount++;
                    }
                }
                if (fluidCount == 4 || (fluidCount == 3 && solidCount == 1)) {
                    cell(i, j) = CellType::FLUID;
                }
            }
        }
    }

#endif

    waterVolume = 0.0f;
    totalEnergy = 0.0f;
    particleTotalEnergy = 0.0f;
    // compute water volume
    for (size_t j = 0; j < SIZEY; j++) {
        for (size_t i = 0; i < SIZEX; i++) {
            if (cell(i, j) == CellType::FLUID) {
                waterVolume += dx * dx;
                auto vel = mac.velInterp(vec2d{i*dx,j*dx});
                totalEnergy += 0.5 * (rho * dx * dx) * vel.LengthSquared();
                totalEnergy -= (rho * dx * dx) * (gravity.x * (i*dx) + gravity.y * (j*dx));
            }
        }
    }
    int particlesPerCellSqrt = WaterSimSettings::Dim2D::PARTICLES_PER_CELL_SQRT;
    for (int e = 0; e < particles.size; e++) {
        auto pos = particles[e];
        auto vel = particleVels[e];
        particleTotalEnergy += 0.5 * (rho * dx * dx / (particlesPerCellSqrt * particlesPerCellSqrt)) * vel.LengthSquared();
        particleTotalEnergy -= (rho * dx * dx / (particlesPerCellSqrt * particlesPerCellSqrt)) * (gravity.x * pos.x + gravity.y * pos.y);
    }
    waterVolumeData.push(waterVolume);
    totalEnergyData.push(totalEnergy);
    particleTotalEnergyData.push(particleTotalEnergy);
}

void WaterSim2D::createSolidLevelSet() {
    // TODO
}

void WaterSim2D::saveStats() {
    perfCounter.saveToFile("perf.csv");
    File file = File::open("conservation.csv", "w+").unwrap();
    std::string buf;
    buf += "Total Volume, Total Energy, Total Energy (Particle) \n";
    for (int i = 0; i < waterVolumeData.size; i++) {
        auto str = String::fmt("%f, %f, %f\n", waterVolumeData[i], totalEnergyData[i], particleTotalEnergyData[i]);
        buf += str.data();
        str.free();
    }
    file.writeAll(buf.c_str());
    file.close();
}

void LevelSet::constructFromParticles(Vec<mathfu::vec2d> particles, double dr) {
    auto* t = new Array2D<size_t, SIZEX, SIZEY>();
    defer {delete t;};

#pragma omp parallel for
    for (size_t j = 0; j < SIZEY; j++) {
        for (size_t i = 0; i < SIZEX; i++) {
            phi(i,j) = HUGE_VAL;
            (*t)(i, j) = (size_t) -1;
        }
    }

#pragma omp parallel for
    for (size_t e = 0; e < particles.size; e++) {
        auto pos = particles[e];
        int x = (int)(pos.x / dx), y = (int)(pos.y / dx);
        double d = sqrt((pos.x - x*dx)*(pos.x - x*dx) + (pos.y - y*dx)*(pos.y - y*dx)) - dr;
        if (d < phi(x,y)) {
            phi(x,y) = d;
            (*t)(x,y) = e;
        }
    }

    fastSweepIterate([&](size_t i, size_t j) {
        const size_t i_list[] = {i-1, i+1, i, i};
        const size_t j_list[] = {j, j, j-1, j+1};
        for (int k = 0; k < 4; k++) {
            if (i_list[k] < 0 || i_list[k] >= SIZEX || j_list[k] < 0 || j_list[k] >= SIZEY) {
                continue;
            }
            size_t e = (*t)(i_list[k],j_list[k]);
            if (e != -1) {
                auto pos = particles[e];
                double d = sqrt((pos.x - i*dx)*(pos.x - i*dx) + (pos.y - j*dx)*(pos.y - j*dx)) - dr;
                if (d < phi(i,j)) {
                    phi(i,j) = d;
                    (*t)(i,j) = e;
                }
            }
        }
    });

}

void LevelSet::redistance() {
    auto isSurface = new Array2D<bool, SIZEX, SIZEY>();
    memset(isSurface->data, false, SIZEX*SIZEY);
    defer {delete isSurface;};
    auto oldPhi = new Array2D<double, SIZEX, SIZEY>();
    defer {delete oldPhi;};

    oldPhi->copyFrom(phi);
    for (size_t j = 0; j < SIZEY - 1; j++) {
        for (size_t i = 0; i < SIZEX - 1; i++) {
            using utils::sgn;
            double phi0 = (*oldPhi)(i,j);
            double phi1 = (*oldPhi)(i+1,j);
            double phi2 = (*oldPhi)(i,j+1);
            if (phi0 * phi1 < 0) {
                if (abs(phi0 - phi1) > dx) {
                    double theta0 = phi0 / (phi0 - phi1);
                    double theta1 = phi1 / (phi0 - phi1);
                    double newPhi0 = utils::sgn(phi0) * theta0 * dx;
                    if (abs(newPhi0) < abs(phi0)) phi(i,j) = phi0;
                    double newPhi1 = utils::sgn(phi1) * theta1 * dx;
                    if (abs(newPhi1) < abs(phi1)) phi(i+1,j) = phi1;
                }
                (*isSurface)(i,j) = true;
                (*isSurface)(i+1,j) = true;
            }
            if (phi0 * phi2 < 0) {
                if (abs(phi0 - phi2) > dx) {
                    double theta0 = phi0 / (phi0 - phi2);
                    double theta2 = phi2 / (phi0 - phi2);
                    double newPhi0 = utils::sgn(phi0) * theta0 * dx;
                    if (abs(newPhi0) < abs(phi0)) phi(i,j) = phi0;
                    double newPhi2 = utils::sgn(phi2) * theta2 * dx;
                    if (abs(newPhi2) < abs(phi2)) phi(i,j+1) = phi2;
                }
                (*isSurface)(i,j) = true;
                (*isSurface)(i,j+1) = true;
            }
        }
    }

    for (size_t j = 0; j < SIZEY; j++) {
        for (size_t i = 0; i < SIZEX; i++) {
            if (!(*isSurface)(i,j) && phi(i,j) < 0) {
                phi(i,j) = -HUGE_VAL;
            }
        }
    }

    using utils::sgn;
    for (int k = 0; k < 4; k++) {
        for (size_t j = 1; j < SIZEY; j++) {
            for (size_t i = 1; i < SIZEX; i++) {
                if (phi(i,j) >= 0) continue;
                double phi0 = utils::min(abs(phi(i-1,j)), abs(phi(i,j-1)));
                double phi1 = utils::max(abs(phi(i-1,j)), abs(phi(i,j-1)));
                double d = phi0 + dx;
                if (d > phi1) {
                    d = 0.5 * (phi0 + phi1 + sqrt(2*dx*dx - (phi1 - phi0)*(phi1 - phi0)));
                }
                if (d < -phi(i,j)) {
                    phi(i,j) = -d;
                }
            }
        }
        for (size_t j = 1; j < SIZEY; j++) {
            for (size_t i = SIZEX - 1; i-- > 0;) {
                if (phi(i,j) >= 0) continue;
                double phi0 = utils::min(abs(phi(i+1,j)), abs(phi(i,j-1)));
                double phi1 = utils::max(abs(phi(i+1,j)), abs(phi(i,j-1)));
                double d = phi0 + dx;
                if (d > phi1) {
                    d = 0.5 * (phi0 + phi1 + sqrt(2*dx*dx - (phi1 - phi0)*(phi1 - phi0)));
                }
                if (d < -phi(i,j)) {
                    phi(i,j) = -d;
                }
            }
        }
        for (size_t j = SIZEY - 1; j-- > 0;) {
            for (size_t i = 1; i < SIZEX; i++) {
                if (phi(i,j) >= 0) continue;
                double phi0 = utils::min(abs(phi(i-1,j)), abs(phi(i,j+1)));
                double phi1 = utils::max(abs(phi(i-1,j)), abs(phi(i,j+1)));
                double d = phi0 + dx;
                if (d > phi1) {
                    d = 0.5 * (phi0 + phi1 + sqrt(2*dx*dx - (phi1 - phi0)*(phi1 - phi0)));
                }
                if (d < -phi(i,j)) {
                    phi(i,j) = -d;
                }
            }
        }
        for (size_t j = SIZEY - 1; j-- > 0;) {
            for (size_t i = SIZEX - 1; i-- > 0;) {
                if (phi(i,j) >= 0) continue;
                double phi0 = utils::min(abs(phi(i+1,j)), abs(phi(i,j+1)));
                double phi1 = utils::max(abs(phi(i+1,j)), abs(phi(i,j+1)));
                double d = phi0 + dx;
                if (d > phi1) {
                    d = 0.5 * (phi0 + phi1 + sqrt(2*dx*dx - (phi1 - phi0)*(phi1 - phi0)));
                }
                if (d < -phi(i,j)) {
                    phi(i,j) = -d;
                }
            }
        }
    }

    // Run smoothing kernel two times to remove any holes
    for (int k = 0; k < 2; k++) {
        oldPhi->copyFrom(phi);
    #pragma omp parallel for
        for (size_t j = 1; j < SIZEY-1; j++) {
            for (size_t i = 1; i < SIZEX-1; i++) {
                size_t im = i == 0? 0 : i-1;
                size_t jm = j == 0? 0 : j-1;
                size_t ip = i == SIZEX-1? SIZEX-1 : i+1;
                size_t jp = j == SIZEY-1? SIZEY-1 : j+1;
                double avg = 0.25 * ((*oldPhi)(im,j) + (*oldPhi)(ip,j) + (*oldPhi)(i,jm) + (*oldPhi)(i,jp));
                if (avg < (*oldPhi)(i,j))
                    phi(i,j) = avg;
            }
        }
    }
}
