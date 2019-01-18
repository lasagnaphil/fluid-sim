//
//
// Created by lasagnaphil on 2018-09-18.
//

#include <ctime>
#include <vec2dx4.h>
#include <math_utils.h>
#include <Defer.h>
#include <Queue.h>
#include <log.h>
#include <File.h>
#include "FluidSim2D.h"
#include "InputManager.h"

#define USE_GHOST_PRESSURE
#define USE_LEVEL_SET
#define USE_RESEEDING

inline double randf(float max) {
    return ((double)rand()/(double)RAND_MAX) * max;
}

FluidSim2D FluidSim2D::create(const FluidSim2DConfig& config) {
    FluidSim2D sim;

    sim.sizeX = config.sizeX;
    sim.sizeY = config.sizeY;
    sim.particlesPerCellSqrt = config.particlesPerCellSqrt;
    sim.particlesPerCell = sim.particlesPerCellSqrt * sim.particlesPerCellSqrt;
    sim.dt = config.dt;
    sim.dx = config.dx;
    sim.dr = 0.9 * config.dx;
    sim.rho = config.rho;
    sim.gravity = {config.gravityX, config.gravityY};
    sim.origGravity = sim.gravity;

    sim.mode = config.mode;
    sim.picFlipAlpha = config.picFlipAlpha;
    sim.numStages = sim.mode == FS_SEMILAGRANGIAN? 7 : 8;

    sim.mac = MACGrid2D::create(sim.sizeX, sim.sizeY, sim.dx);
    sim.newMac = MACGrid2D::create(sim.sizeX, sim.sizeY, sim.dx);
    sim.p = Array2D<double>::create(sim.sizeX, sim.sizeY);
    sim.cell = Array2D<FluidCellType>::create(sim.sizeX, sim.sizeY);
    sim.perfCounter = PerformanceCounter::create(sim.numStages);

    memcpy(sim.cell.data, config.initialValues, sizeof(uint8_t)*sim.sizeX*sim.sizeY);

    size_t fluidCount = 0;

    sim.particles = Vec<vec2d>::create(sim.particlesPerCell*fluidCount);
    float dist = 1.0f / sim.particlesPerCellSqrt;
    for (int j = 0; j < sim.sizeY; j++) {
        for (int i = 0; i < sim.sizeX; i++) {
            if (sim.cell(i,j) == FS_FLUID) {
                fluidCount++;
                for (int k = 0; k < sim.particlesPerCell; k++) {
                    int x = k % sim.particlesPerCellSqrt;
                    int y = k / sim.particlesPerCellSqrt;
                    sim.particles.push(vec2d {((double) i + dist * x + randf(dist)) * sim.dx, ((double) j + dist * y + randf(dist)) * sim.dx});
                }
            }
        }
    }

    sim.particleVels = Vec<vec2d>::emptyWithSize(sim.particlesPerCell*fluidCount);
    for (int i = 0; i < sim.particlesPerCell*fluidCount; i++) {
        sim.particleVels[i] = vec2d {0.0, 0.0};
    }

    sim.waterVolume = fluidCount * sim.dx * sim.dx;
    sim.origWaterVolume = sim.waterVolume;
    sim.waterLevelSet = LevelSet::create(sim.sizeX, sim.sizeY, sim.dx);

    return sim;
}

void FluidSim2D::free() {
    mac.free();
    newMac.free();
    p.free();
    cell.free();
    particles.free();
    particleVels.free();

    waterLevelSet.free();

    perfCounter.free();
    waterVolumeData.free();
    totalEnergyData.free();
    particleTotalEnergyData.free();
}

void FluidSim2D::runFrame() {
#define STOPWATCH(__CODE) \
    perfCounter.beginStage(); \
    __CODE; \
    perfCounter.endStage();

    if (mode == FS_SEMILAGRANGIAN) {
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
    else if (mode == FS_PICFLIP) {
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

void FluidSim2D::update() {
    runFrame();
}

void FluidSim2D::transferVelocityToGrid() {
    mac.u.reset();
    mac.v.reset();
    auto udiv = Array2D<double>::create(sizeX+1, sizeY);
    auto vdiv = Array2D<double>::create(sizeX, sizeY+1);
    defer {
        udiv.free(); vdiv.free();
    };
#if 1
    for (int e = 0; e < particles.size; e++) {
        auto xp = particles[e];
        auto vp = particleVels[e];
        auto upos = vec2d {xp.x / dx, xp.y / dx - 0.5};
        mac.u.linearDistribute(upos, vp.x);
        udiv.linearDistribute(upos, 1.0);
        auto vpos = vec2d {xp.x / dx - 0.5, xp.y / dx};
        mac.v.linearDistribute(vpos, vp.y);
        vdiv.linearDistribute(vpos, 1.0);
    }
#else
    for (int e = 0; e < aml::max<int>(4, particles.size+3); e += 4) {
        auto xp = vec2dx4::load(&particles[e]);
        auto vp = vec2dx4::load(&particleVels[e]);
        auto upos = vec2dx4 {xp.x / dx, xp.y / dx - 0.5};
        mac.u.linearDistribute(upos, vp.x);
        udiv->linearDistribute(upos, vec4d::make(1.0));
        auto vpos = vec2dx4 {xp.x / dx - 0.5, xp.y / dx};
        mac.v.linearDistribute(vpos, vp.y);
        vdiv->linearDistribute(vpos, vec4d::make(1.0));
    }
#endif
    for (int j = 0; j < sizeY; j++) {
        for (int i = 0; i < sizeX + 1; i++) {
            if (udiv(i, j) > 0) mac.u(i, j) /= udiv(i, j);
        }
    }
    for (int j = 0; j < sizeY + 1; j++) {
        for (int i = 0; i < sizeX; i++) {
            if (vdiv(i, j) > 0) mac.v(i, j) /= vdiv(i, j);
        }
    }

    auto uintFlag = Array2D<uint32_t>::create(sizeX+1, sizeY);
    auto vintFlag = Array2D<uint32_t>::create(sizeX, sizeY+1);
    defer {
        uintFlag.free(); vintFlag.free();
    };

    for (int j = 0; j < sizeY; j++) {
        for (int i = 0; i < sizeX + 1; i++) {
            uintFlag(i,j) = mac.u(i,j) == 0.0? UINT32_MAX : 0;
        }
    }
    for (int j = 0; j < sizeY + 1; j++) {
        for (int i = 0; i < sizeX; i++) {
            vintFlag(i,j) = mac.v(i,j) == 0.0? UINT32_MAX : 0;
        }
    }
    mac.u.extrapolate(uintFlag);
    mac.v.extrapolate(vintFlag);
}

void FluidSim2D::applySemiLagrangianAdvection() {
    double C = 5;

#pragma omp parallel for
    for (int j = 0; j < sizeY; j++) {
        for (int i = 0; i < sizeX + 1; i++) {
            vec2d x_p = vec2d {(double)i, ((double)j + 0.5)} * dx;
            auto k1 = mac.velInterp(x_p);
            auto k2 = mac.velInterp(x_p - 0.5*dt*k1);
            auto k3 = mac.velInterp(x_p - 0.75*dt*k2);
            auto x_p_new = x_p - ((2./9.)*dt*k1 + (3./9.)*dt*k2 + (4./9.)*dt*k3);
            x_p_new = clampPos(x_p_new);
            mac.u(i,j) = mac.velInterpU(x_p_new);
        }
    }

#pragma omp parallel for
    for (int j = 0; j < sizeY + 1; j++) {
        for (int i = 0; i < sizeX; i++) {
            vec2d x_p = vec2d {(double)i + 0.5, ((double)j)} * dx;
            auto k1 = mac.velInterp(x_p);
            auto k2 = mac.velInterp(x_p - 0.5*dt*k1);
            auto k3 = mac.velInterp(x_p - 0.75*dt*k2);
            auto x_p_new = x_p - ((2./9.)*dt*k1 + (3./9.)*dt*k2 + (4./9.)*dt*k3);
            x_p_new = clampPos(x_p_new);
            mac.v(i,j) = mac.velInterpV(x_p_new);
        }
    }

}

void FluidSim2D::applyGravity() {
#pragma omp parallel for
    for (int j = 0; j < sizeY; j++) {
        for (int i = 0; i < sizeX + 1; i++) {
            mac.u(i,j) += dt * gravity.x;
        }
    }
#pragma omp parallel for
    for (int j = 0; j < sizeY + 1; j++) {
        for (int i = 0; i < sizeX; i++) {
            mac.v(i,j) += dt * gravity.y;
        }
    }
}

void FluidSim2D::applyProjection() {
    auto Adiag = Array2D<double>::create(sizeX, sizeY);
    auto Ax = Array2D<double>::create(sizeX, sizeY);
    auto Ay = Array2D<double>::create(sizeX, sizeY);
    defer {
        Adiag.free(); Ax.free(); Ay.free();
    };

    // calculate lhs (matrix A)
    double scaleA = dt / (rho * dx * dx);
#ifdef USE_GHOST_PRESSURE
#pragma omp parallel for
    for (int j = 0; j < sizeY; j++) {
        for (int i = 0; i < sizeX; i++) {
            if (cell(i,j) == FS_FLUID) {
                if (cell(i-1,j) == FS_FLUID) {
                    Adiag(i,j) += scaleA;
                }
                else if (cell(i-1,j) == FS_EMPTY) {
                    double phi1 = waterLevelSet.phi(i-1, j);
                    double phi0 = waterLevelSet.phi(i, j);
                    Adiag(i,j) -= scaleA * aml::max(phi1 / phi0, -1e3);
                }
                if (cell(i+1,j) == FS_FLUID) {
                    Adiag(i,j) += scaleA;
                    Ax(i,j) = -scaleA;
                }
                else if (cell(i+1,j) == FS_EMPTY) {
                    double phi1 = waterLevelSet.phi(i+1, j);
                    double phi0 = waterLevelSet.phi(i, j);
                    Adiag(i,j) += scaleA * (1 - aml::max(phi1 / phi0, -1e3));
                }
                if (cell(i,j-1) == FS_FLUID) {
                    Adiag(i,j) += scaleA;
                }
                else if (cell(i,j-1) == FS_EMPTY) {
                    double phi1 = waterLevelSet.phi(i, j-1);
                    double phi0 = waterLevelSet.phi(i, j);
                    Adiag(i,j) -= scaleA * aml::max(phi1 / phi0, -1e3);
                }
                if (cell(i,j+1) == FS_FLUID) {
                    Adiag(i,j) += scaleA;
                    Ay(i,j) = -scaleA;
                }
                else if (cell(i,j+1) == FS_EMPTY) {
                    double phi1 = waterLevelSet.phi(i, j+1);
                    double phi0 = waterLevelSet.phi(i, j);
                    Adiag(i,j) += scaleA * (1 - aml::max(phi1 / phi0, -1e3));
                }
            }
        }
    }
#else
#pragma omp parallel for
    for (int j = 0; j < sizeY; j++) {
        for (int i = 0; i < sizeX; i++) {
            if (cell(i,j) == FS_FLUID) {
                if (cell(i-1,j) == FS_FLUID) {
                    Adiag(i,j) += scaleA;
                }
                if (cell(i+1,j) == FS_FLUID) {
                    Adiag(i,j) += scaleA;
                    Ax(i,j) = -scaleA;
                }
                else if (cell(i+1,j) == FS_EMPTY) {
                    Adiag(i,j) += scaleA;
                }
                if (cell(i,j-1) == FS_FLUID) {
                    Adiag(i,j) += scaleA;
                }
                if (cell(i,j+1) == FS_FLUID) {
                    Adiag(i,j) += scaleA;
                    Ay(i,j) = -scaleA;
                }
                else if (cell(i,j+1) == FS_EMPTY) {
                    Adiag(i,j) += scaleA;
                }
            }
        }
    }
#endif

    auto rhs = Array2D<double>::create(sizeX, sizeY);
    defer {rhs.free();};

    // calculate rhs
    {
        double scale = 1.0 / dx;
#pragma omp parallel for
        for (int j = 0; j < sizeY; j++) {
            for (int i = 0; i < sizeX; i++) {
                if (cell(i,j) == FS_FLUID) {
                    rhs(i,j) = -scale * (mac.u(i+1,j)-mac.u(i,j)+mac.v(i,j+1)-mac.v(i,j));
                    // modify rhs to account for solid velocities
                    // TODO: use usolid, vsolid, wsolid instead of 0
                    if (cell(i-1,j) == FS_SOLID) {
                        rhs(i,j) -= scale * (mac.u(i,j) - 0);
                    }
                    if (cell(i+1,j) == FS_SOLID) {
                        rhs(i,j) += scale * (mac.u(i+1,j) - 0);
                    }
                    if (cell(i,j-1) == FS_SOLID) {
                        rhs(i,j) -= scale * (mac.v(i,j) - 0);
                    }
                    if (cell(i,j+1) == FS_SOLID) {
                        rhs(i,j) += scale * (mac.v(i,j+1) - 0);
                    }
                }
            }
        }
    }

#define SQUARE(x) (x)*(x)
    // find preconditioner
    auto precon = Array2D<double>::create(sizeX, sizeY);
    defer {precon.free();};
    {
        constexpr double tau = 0.999;
        constexpr double sigma = 0.25;
        double e;
#pragma omp parallel for
        for (int j = 1; j < sizeY; j++) {
            for (int i = 1; i < sizeX; i++) {
                if (cell(i, j) == FS_FLUID) {
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

    auto r = Array2D<double>::create(sizeX, sizeY);
    auto z = Array2D<double>::create(sizeX, sizeY);
    auto s = Array2D<double>::create(sizeX, sizeY);
    defer {
        r.free(); z.free(); s.free();
    };

    auto applyPreconditioner = [&]() {
        // apply preconditioner
        auto q = Array2D<double>::create(sizeX, sizeY);
        for (int j = 1; j < sizeY; j++) {
            for (int i = 1; i < sizeX; i++) {
                if (cell(i,j) == FS_FLUID) {
                    double t = r(i,j)
                            - Ax(i-1,j) * precon(i-1,j) * q(i-1,j)
                            - Ay(i,j-1) * precon(i,j-1) * q(i,j-1);
                    q(i,j) = t * precon(i,j);
                }
            }
        }
        for (int j = sizeY - 1; j-- > 0; ) {
            for (int i = sizeX - 1; i-- > 0; ) {
                if (cell(i,j) == FS_FLUID) {
                    double t = q(i,j)
                            - Ax(i,j) * precon(i,j) * z(i+1,j)
                            - Ay(i,j) * precon(i,j) * z(i,j+1);
                    z(i,j) = t * precon(i,j);
                }
            }
        }
        q.free();
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
        for (int j = 0; j < sizeY; j++) {
            for (int i = 0; i < sizeX; i++) {
                if (cell(i, j) == FS_FLUID) {
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
}

void FluidSim2D::updateVelocity() {
    // update velocity using the solved pressure
    auto du = Array2D<uint32_t>::create(sizeX+1, sizeY);
    defer {du.free();};
    auto dv = Array2D<uint32_t>::create(sizeX, sizeY+1);
    defer {dv.free();};

    newMac.copyFrom(mac);
    {
        double scale = dt / (rho * dx);
#pragma omp parallel for
        for (int j = 0; j < sizeY; j++) {
            for (int i = 0; i < sizeX; i++) {
                if ((i > 0 && cell(i-1,j) == FS_FLUID) || cell(i,j) == FS_FLUID) {
                    if ((i == 0 || cell(i-1,j) == FS_SOLID) || cell(i,j) == FS_SOLID) {
                        newMac.u(i,j) = 0; // usolid(i,j);
                    }
#ifdef USE_GHOST_PRESSURE
                    else if (i > 0 && cell(i-1,j) == FS_EMPTY) {
                        // account for ghost pressures
                        double phi1 = waterLevelSet.phi(i-1, j);
                        double phi0 = waterLevelSet.phi(i, j);
                        newMac.u(i,j) -= scale * (1 - aml::max(phi1/phi0, -1e3)) * p(i,j);
                    }
                    else if (cell(i,j) == FS_EMPTY) {
                        // account for ghost pressures
                        double phi1 = waterLevelSet.phi(i, j);
                        double phi0 = waterLevelSet.phi(i-1, j);
                        newMac.u(i,j) -= scale * (aml::max(phi1/phi0, -1e3) - 1) * p(i-1,j);
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
                    du(i,j) = UINT32_MAX;
                }
                if ((j > 0 && cell(i,j-1) == FS_FLUID) || cell(i,j) == FS_FLUID) {
                    if ((j == 0 || cell(i,j-1) == FS_SOLID) || cell(i,j) == FS_SOLID) {
                        newMac.v(i,j) = 0; // vsolid(i,0)
                    }
#ifdef USE_GHOST_PRESSURE
                    else if (j > 0 && cell(i,j-1) == FS_EMPTY) {
                        // account for ghost pressures
                        double phi1 = waterLevelSet.phi(i, j-1);
                        double phi0 = waterLevelSet.phi(i, j);
                        newMac.v(i,j) -= scale * (1 - aml::max(phi1/phi0, -1e3)) * p(i,j);
                    }
                    else if (cell(i,j) == FS_EMPTY) {
                        // account for ghost pressures
                        double phi1 = waterLevelSet.phi(i, j);
                        double phi0 = waterLevelSet.phi(i, j-1);
                        newMac.v(i,j) -= scale * (aml::max(phi1/phi0, -1e3) - 1) * p(i,j-1);
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
                    dv(i,j) = UINT32_MAX;
                }
            }
        }
    }

    newMac.u.extrapolate(du);
    newMac.v.extrapolate(dv);

    if (mode == FS_SEMILAGRANGIAN) {
        mac.copyFrom(newMac);
    }
}

void FluidSim2D::updateParticleVelocities() {
    // PIC update
    Array2D<double> uDiff = newMac.u - mac.u;
    Array2D<double> vDiff = newMac.v - mac.v;
    defer {uDiff.free(); vDiff.free();};

#pragma omp parallel for
    for (int e = 0; e < particles.size; e++) {
        auto upos = vec2d {particles[e].x / dx, particles[e].y / dx - 0.5};
        auto vpos = vec2d {particles[e].x / dx - 0.5, particles[e].y / dx};
        auto picVel = vec2d {newMac.u.linearExtract(upos), newMac.v.linearExtract(vpos)};
        auto flipVel = particleVels[e] + vec2d {uDiff.linearExtract(upos), vDiff.linearExtract(vpos)};
        particleVels[e] = (double)picFlipAlpha * picVel + (1 - (double)picFlipAlpha) * flipVel;
    }

    mac.copyFrom(newMac);
}

void FluidSim2D::applyAdvection() {
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
                cmax, aml::norm(particles[emax]), particles[emax].x, particles[emax].y);
    }


    #pragma omp parallel for
    for (int i = 0; i < particles.size; i++) {
        auto& pos = particles[i];
        auto k1 = mac.velInterp(pos);
        auto k2 = mac.velInterp(pos + 0.5*dt*k1);
        auto k3 = mac.velInterp(pos + 0.75*dt*k2);

        auto nextPos = pos + (2./9.)*dt*k1 + (3./9.)*dt*k2 + (4./9.)*dt*k3;
        nextPos = clampPos(nextPos);

        if (isnan(pos.x) || isnan(pos.y)) {
            log_error("Advected position is NaN! pos=(%d, %d), idx=%d", pos.x, pos.y, i);
            exit(1);
        }
        pos = nextPos;
    }
}


double FluidSim2D::avgPressure() {
    double avgP = 0.0f;
    iterate([&](int i, int j) {
        avgP += p(i,j);
    });
    avgP /= (sizeX*sizeY);
    return avgP;
}

double FluidSim2D::avgPressureInFluid() {
    double avgP = 0.0f;
    size_t count = 0;
    iterate([&](int i, int j) {
        if (cell(i,j) == FS_FLUID) {
            avgP += p(i,j);
            count++;
        }
    });
    avgP /= count;
    return avgP;
}

double FluidSim2D::maxVelocity() {
    double maxVel = 0.0f;
    iterate([&](int i, int j) {
        if (cell(i,j) == FS_FLUID) {
            double vel = aml::norm(mac.velInterp(vec2d{(double)i,(double)j}*dx));
            if (vel > maxVel) maxVel = vel;
        }
    });
    return maxVel;
}

vec2d FluidSim2D::getGridCenter() {
    return vec2d {sizeX * dx / 2, sizeY * dx / 2};
}


vec2d FluidSim2D::clampPos(vec2d to) {
    constexpr double offset = 1e-3;
    return vec2d {
        aml::clamp<double>(to.x, (1.0 + offset) * dx, (sizeX-1.0 - offset)*dx),
        aml::clamp<double>(to.y, (1.0 + offset) * dx, (sizeY-1.0 - offset)*dx)
    };
}

void FluidSim2D::createWaterLevelSet() {
#ifdef USE_LEVEL_SET
    waterLevelSet.constructFromParticles(particles, dr);
    waterLevelSet.redistance();
    for (int j = 0; j < sizeY; j++) {
        for (int i = 0; i < sizeX; i++) {
            if (cell(i, j) != FS_SOLID) {
                double phi = waterLevelSet.phi(i, j);
                cell(i, j) = phi < 0.0? FS_FLUID : FS_EMPTY;
            }
        }
    }
#else

#pragma omp parallel for
    for (int j = 0; j < sizeY; j++) {
        for (int i = 0; i < sizeX; i++) {
            if(cell(i,j) == FS_FLUID) {
                cell(i,j) = FS_EMPTY;
            }
        }
    }

#pragma omp parallel for
    for (int i = 0; i < particles.size; i++) {
        auto pos = particles[i];
        int x = (int)pos.x; int y = (int)pos.y;
        if (cell(x, y) == FS_EMPTY) {
            cell(x, y) = FS_FLUID;
        }
    }

    // Correction to eliminate "bubbles" inside fluids (and on wall boundaries)
#pragma omp parallel for
    for (int i = 1; i < sizeX - 1; i++) {
        for (int j = 1; j < sizeY - 1; j++) {
            if (cell(i, j) == FS_EMPTY) {
                int fluidCount = 0;
                int solidCount = 0;
                for (auto neighbor : {cell(i-1, j), cell(i+1, j), cell(i,j-1), cell(i,j+1)}) {
                    if (neighbor == FS_FLUID) {
                        fluidCount++;
                    }
                    else if (neighbor == FS_SOLID) {
                        solidCount++;
                    }
                }
                if (fluidCount == 4 || (fluidCount == 3 && solidCount == 1)) {
                    cell(i, j) = FS_FLUID;
                }
            }
        }
    }

#endif

    waterVolume = 0.0f;
    totalEnergy = 0.0f;
    particleTotalEnergy = 0.0f;
    // compute water volume
    for (int j = 0; j < sizeY; j++) {
        for (int i = 0; i < sizeX; i++) {
            if (cell(i, j) == FS_FLUID) {
                waterVolume += dx * dx;
                auto vel = mac.velInterp(vec2d{i*dx,j*dx});
                totalEnergy += 0.5 * (rho * dx * dx) * aml::normsq(vel);
                totalEnergy -= (rho * dx * dx) * (gravity.x * (i*dx) + gravity.y * (j*dx));
            }
        }
    }
    for (int e = 0; e < particles.size; e++) {
        auto pos = particles[e];
        auto vel = particleVels[e];
        particleTotalEnergy += 0.5 * (rho * dx * dx / (particlesPerCellSqrt * particlesPerCellSqrt)) * aml::normsq(vel);
        particleTotalEnergy -= (rho * dx * dx / (particlesPerCellSqrt * particlesPerCellSqrt)) * (gravity.x * pos.x + gravity.y * pos.y);
    }
    waterVolumeData.push(waterVolume);
    totalEnergyData.push(totalEnergy);
    particleTotalEnergyData.push(particleTotalEnergy);
}

void FluidSim2D::createSolidLevelSet() {
    // TODO
}

void FluidSim2D::saveStats() {
    perfCounter.saveToFile("perf.csv");
    File file = File::open("conservation.csv", "w+").unwrap();
    std::string buf;
    buf += "Total Volume, Total Energy, Total Energy (Particle) \n";
    for (int i = 0; i < waterVolumeData.size; i++) {
        auto str = String::fmt("%f, %f, %f\n", waterVolumeData[i], totalEnergyData[i], particleTotalEnergyData[i]).unwrap();
        buf += str.data();
        str.free();
    }
    file.writeAll(buf.c_str());
    file.close();
}

void LevelSet::constructFromParticles(Vec<vec2d> particles, double dr) {
    auto t = Array2D<size_t>::create(sizeX, sizeY);
    defer {t.free();};

#pragma omp parallel for
    for (int j = 0; j < sizeY; j++) {
        for (int i = 0; i < sizeX; i++) {
            phi(i,j) = HUGE_VAL;
            t(i, j) = (size_t) -1;
        }
    }

#pragma omp parallel for
    for (size_t e = 0; e < particles.size; e++) {
        auto pos = particles[e];
        int x = (int)(pos.x / dx), y = (int)(pos.y / dx);
        double d = sqrt((pos.x - x*dx)*(pos.x - x*dx) + (pos.y - y*dx)*(pos.y - y*dx)) - dr;
        if (d < phi(x,y)) {
            phi(x,y) = d;
            t(x,y) = e;
        }
    }

    fastSweepIterate([&](int i, int j) {
        const int i_list[] = {i-1, i+1, i, i};
        const int j_list[] = {j, j, j-1, j+1};
        for (int k = 0; k < 4; k++) {
            if (i_list[k] < 0 || i_list[k] >= sizeX || j_list[k] < 0 || j_list[k] >= sizeY) {
                continue;
            }
            size_t e = t(i_list[k],j_list[k]);
            if (e != -1) {
                auto pos = particles[e];
                double d = sqrt((pos.x - i*dx)*(pos.x - i*dx) + (pos.y - j*dx)*(pos.y - j*dx)) - dr;
                if (d < phi(i,j)) {
                    phi(i,j) = d;
                    t(i,j) = e;
                }
            }
        }
    });

}

void LevelSet::redistance() {
    auto isSurface = Array2D<bool>::create(sizeX, sizeY);
    memset(isSurface.data, false, sizeX*sizeY);
    auto oldPhi = Array2D<double>::create(sizeX, sizeY);
    defer {isSurface.free(); oldPhi.free();};

    oldPhi.copyFrom(phi);
    for (int j = 0; j < sizeY - 1; j++) {
        for (int i = 0; i < sizeX - 1; i++) {
            using aml::sgn;
            double phi0 = oldPhi(i,j);
            double phi1 = oldPhi(i+1,j);
            double phi2 = oldPhi(i,j+1);
            if (phi0 * phi1 < 0) {
                if (abs(phi0 - phi1) > dx) {
                    double theta0 = phi0 / (phi0 - phi1);
                    double theta1 = phi1 / (phi0 - phi1);
                    double newPhi0 = aml::sgn(phi0) * theta0 * dx;
                    if (abs(newPhi0) < abs(phi0)) phi(i,j) = phi0;
                    double newPhi1 = aml::sgn(phi1) * theta1 * dx;
                    if (abs(newPhi1) < abs(phi1)) phi(i+1,j) = phi1;
                }
                isSurface(i,j) = true;
                isSurface(i+1,j) = true;
            }
            if (phi0 * phi2 < 0) {
                if (abs(phi0 - phi2) > dx) {
                    double theta0 = phi0 / (phi0 - phi2);
                    double theta2 = phi2 / (phi0 - phi2);
                    double newPhi0 = aml::sgn(phi0) * theta0 * dx;
                    if (abs(newPhi0) < abs(phi0)) phi(i,j) = phi0;
                    double newPhi2 = aml::sgn(phi2) * theta2 * dx;
                    if (abs(newPhi2) < abs(phi2)) phi(i,j+1) = phi2;
                }
                isSurface(i,j) = true;
                isSurface(i,j+1) = true;
            }
        }
    }

    for (int j = 0; j < sizeY; j++) {
        for (int i = 0; i < sizeX; i++) {
            if (!isSurface(i,j) && phi(i,j) < 0) {
                phi(i,j) = -HUGE_VAL;
            }
        }
    }

    using aml::sgn;
    for (int k = 0; k < 4; k++) {
        for (int j = 1; j < sizeY; j++) {
            for (int i = 1; i < sizeX; i++) {
                if (phi(i,j) >= 0) continue;
                double phi0 = aml::min(abs(phi(i-1,j)), abs(phi(i,j-1)));
                double phi1 = aml::max(abs(phi(i-1,j)), abs(phi(i,j-1)));
                double d = phi0 + dx;
                if (d > phi1) {
                    d = 0.5 * (phi0 + phi1 + sqrt(2*dx*dx - (phi1 - phi0)*(phi1 - phi0)));
                }
                if (d < -phi(i,j)) {
                    phi(i,j) = -d;
                }
            }
        }
        for (int j = 1; j < sizeY; j++) {
            for (int i = sizeX - 1; i-- > 0;) {
                if (phi(i,j) >= 0) continue;
                double phi0 = aml::min(abs(phi(i+1,j)), abs(phi(i,j-1)));
                double phi1 = aml::max(abs(phi(i+1,j)), abs(phi(i,j-1)));
                double d = phi0 + dx;
                if (d > phi1) {
                    d = 0.5 * (phi0 + phi1 + sqrt(2*dx*dx - (phi1 - phi0)*(phi1 - phi0)));
                }
                if (d < -phi(i,j)) {
                    phi(i,j) = -d;
                }
            }
        }
        for (int j = sizeY - 1; j-- > 0;) {
            for (int i = 1; i < sizeX; i++) {
                if (phi(i,j) >= 0) continue;
                double phi0 = aml::min(abs(phi(i-1,j)), abs(phi(i,j+1)));
                double phi1 = aml::max(abs(phi(i-1,j)), abs(phi(i,j+1)));
                double d = phi0 + dx;
                if (d > phi1) {
                    d = 0.5 * (phi0 + phi1 + sqrt(2*dx*dx - (phi1 - phi0)*(phi1 - phi0)));
                }
                if (d < -phi(i,j)) {
                    phi(i,j) = -d;
                }
            }
        }
        for (int j = sizeY - 1; j-- > 0;) {
            for (int i = sizeX - 1; i-- > 0;) {
                if (phi(i,j) >= 0) continue;
                double phi0 = aml::min(abs(phi(i+1,j)), abs(phi(i,j+1)));
                double phi1 = aml::max(abs(phi(i+1,j)), abs(phi(i,j+1)));
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
        oldPhi.copyFrom(phi);
    #pragma omp parallel for
        for (int j = 1; j < sizeY-1; j++) {
            for (int i = 1; i < sizeX-1; i++) {
                int im = i == 0? 0 : i-1;
                int jm = j == 0? 0 : j-1;
                int ip = i == sizeX-1? sizeX-1 : i+1;
                int jp = j == sizeY-1? sizeY-1 : j+1;
                double avg = 0.25 * (oldPhi(im,j) + oldPhi(ip,j) + oldPhi(i,jm) + oldPhi(i,jp));
                if (avg < oldPhi(i,j))
                    phi(i,j) = avg;
            }
        }
    }
}
