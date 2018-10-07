//
// Created by lasagnaphil on 2018-09-18.
//

#include <Defer.h>
#include <ctime>
#include "WaterSim2D.h"
#include "InputManager.h"

inline double randf() {
    return ((double)rand()/(double)RAND_MAX) * 0.5;
}

void WaterSim2D::setup() {
    mac.dx = dx;
    size_t fluidCount = 0;
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
    particles.reserve(fluidCount);
    iterate([&](size_t i, size_t j) {
        if (cell(i,j) == CellType::FLUID) {
            particles.push(Vector2d::create((double)i + randf(), (double)j + randf()) * dx);
            particles.push(Vector2d::create((double)i + 0.5 + randf(), (double)j + randf()) * dx);
            particles.push(Vector2d::create((double)i + randf(), (double)j + 0.5 + randf()) * dx);
            particles.push(Vector2d::create((double)i + 0.5 + randf(), (double)j + 0.5 + randf()) * dx);
        }
    });

    createLevelSet();
}

Vector2d WaterSim2D::clampPos(Vector2d pos) {
    Vector2d clamped = {};
    clamped.x = utils::clamp(pos.x, 0.0, (double)SIZEX * dx - 1e-10);
    clamped.y = utils::clamp(pos.y, 0.0, (double)SIZEY * dx - 1e-10);
    return clamped;
}

void WaterSim2D::runFrame() {
    stage = StageType::CreateLevelSet;
    createLevelSet();
    stage = StageType::UpdateLevelSet;
    updateLevelSet();
    stage = StageType::ApplyAdvection;
    applyAdvection();
    stage = StageType::ApplyGravity;
    applyGravity();
    stage = StageType::ApplyProjection;
    applyProjection();
    stage = StageType::UpdateCells;
    updateCells();
    rendered = false;
    currentTime += dt;
}

void WaterSim2D::update() {
    /*
    auto inputMgr = InputManager::get();
    if (inputMgr->isKeyEntered(SDL_SCANCODE_RETURN)) {
        if (stage == StageType::Init) {
            stage = StageType::CreateLevelSet;
            createLevelSet();
        }
        else if (stage == StageType::CreateLevelSet) {
            stage = StageType::UpdateLevelSet;
            updateLevelSet();
        }
        else if (stage == StageType::UpdateLevelSet) {
            stage = StageType::ApplyAdvection;
            applyAdvection();
        }
        else if (stage == StageType::ApplyAdvection) {
            stage = StageType::ApplyGravity;
            applyGravity();
        }
        else if (stage == StageType::ApplyGravity) {
            stage = StageType::ApplyProjection;
            applyProjection();
        }
        else if (stage == StageType::ApplyProjection) {
            stage = StageType::UpdateCells;
            updateCells();
            currentTime += dt;
        }
        else if (stage == StageType::UpdateCells) {
            stage = StageType::CreateLevelSet;
            createLevelSet();
        }
        rendered = false;
    }
     */
    runFrame();
}

void WaterSim2D::applyAdvection() {
    double C = 5;
    iterateU([&](size_t i, size_t j) {
        Vector2d x_p = Vector2d::create((double)i, ((double)j + 0.5)) * dx;
        double tau = 0;
        bool finished = false;
        while (!finished) {
            auto k1 = mac.velInterp(x_p);
            double dtau = C * dx / (k1.normalize() + 10e-37);
            if (tau + dtau >= dt) {
                dtau = dt - tau;
                finished = true;
            }
            else if (tau + 2 * dtau >= dt) {
                dtau = 0.5 * (dt - tau);
            }
            auto k2 = mac.velInterp(x_p - 0.5*dtau*k1);
            auto k3 = mac.velInterp(x_p - 0.75*dtau*k2);
            x_p -= (2./9.)*dtau*k1 + (3./9.)*dtau*k2 + (4./9.)*dtau*k3;
            tau += dtau;
        }
        mac.u(i,j) = mac.velInterpU(x_p);
    });
    iterateV([&](size_t i, size_t j) {
        Vector2d x_p = Vector2d::create((double)i + 0.5, ((double)j)) * dx;
        double tau = 0;
        bool finished = false;
        while (!finished) {
            auto k1 = mac.velInterp(x_p);
            double dtau = C * dx / (k1.normalize() + 10e-37);
            if (tau + dtau >= dt) {
                dtau = dt - tau;
                finished = true;
            }
            else if (tau + 2 * dtau >= dt) {
                dtau = 0.5 * (dt - tau);
            }
            auto k2 = mac.velInterp(x_p - 0.5*dtau*k1);
            auto k3 = mac.velInterp(x_p - 0.75*dtau*k2);
            x_p -= (2./9.)*dtau*k1 + (3./9.)*dtau*k2 + (4./9.)*dtau*k3;
            tau += dtau;
        }
        mac.v(i,j) = mac.velInterpV(x_p);
    });
}

void WaterSim2D::applyGravity() {
    iterateV([&](size_t i, size_t j) {
        mac.v(i,j) += dt * gravity;
    });
}

void WaterSim2D::applyProjection() {
    auto gridStack = Vec<Grid2D<double>>::create(9);

    auto& Adiag = gridStack.newItem();
    auto& Ax = gridStack.newItem();
    auto& Ay = gridStack.newItem();

    // calculate lhs (matrix A)
    double scaleA = dt / (rho * dx * dx);
    iterate([&](size_t i, size_t j) {
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
    });

    auto& rhs = gridStack.newItem();

    // calculate rhs
    {
        double scale = 1.0 / dx;
        iterate([&](size_t i, size_t j) {
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
        });
    }

#define SQUARE(x) (x)*(x)
    // find preconditioner
    auto& precon = gridStack.newItem();
    {
        constexpr double tau = 0.97;
        constexpr double sigma = 0.25;
        double e;
        iterate([&](size_t i, size_t j) {
            if (cell(i, j) == CellType::FLUID) {
                e = Adiag(i,j)
                    - (i > 0? SQUARE(Ax(i-1,j) * precon(i-1,j)) : 0)
                    - (j > 0? SQUARE(Ay(i,j-1) * precon(i,j-1)) : 0)
                    - tau * ((i > 0? Ax(i-1,j)*Ay(i-1,j)*SQUARE(precon(i-1,j)) : 0)
                             + (j > 0? Ay(i,j-1)*Ax(i,j-1)*SQUARE(precon(i,j-1)) : 0));
                if (e < sigma * Adiag(i,j)) {
                    e = Adiag(i,j);
                }
                precon(i,j) = 1 / sqrt(e);
            }
        });
    }
#undef SQUARE

    auto& r = gridStack.newItem();
    auto& z = gridStack.newItem();
    auto& s = gridStack.newItem();

    auto applyPreconditioner = [&]() {
        // apply preconditioner
        auto& q = gridStack.newItem();
        iterate([&](size_t i, size_t j) {
            if (cell(i,j) == CellType::FLUID) {
                double t = r(i,j) - (i > 0? Ax(i-1,j) * precon(i-1,j) * q(i-1,j) : 0)
                           - (j > 0? Ay(i,j-1) * precon(i,j-1) * q(i,j-1) : 0);
                q(i,j) = t * precon(i,j);
            }
        });
        iterateBackwards([&](size_t i, size_t j) {
            if (cell(i,j) == CellType::FLUID) {
                double t = q(i,j) - (i < SIZEX - 1? Ax(i,j) * precon(i,j) * z(i+1,j) : 0)
                           - (j < SIZEY - 1? Ay(i,j) * precon(i,j) * z(i,j+1) : 0);
                z(i,j) = t * precon(i,j);
            }
        });
        gridStack.pop();
    };

    // use PCG algorithm to solve the linear equation
    r = rhs;
    applyPreconditioner();
    s = z;
    double sigma = z.innerProduct(r);
    int maxIters = 200;
    int iter = 0;
    while (iter < maxIters) {
        // apply A
        iterate([&](size_t i, size_t j) {
            if (cell(i,j) == CellType::FLUID) {
                z(i, j) = Adiag(i, j) * s(i, j)
                          + (i > 0 ? Ax(i - 1, j) * s(i - 1, j) : 0)
                          + (i < SIZEX - 1 ? Ax(i, j) * s(i + 1, j) : 0)
                          + (j > 0 ? Ay(i, j - 1) * s(i, j - 1) : 0)
                          + (j < SIZEY - 1 ? Ay(i, j) * s(i, j + 1) : 0);
            }
        });

        double alpha = sigma / z.innerProduct(s);
        p.setMultiplyAdd(p, alpha, s);
        r.setMultiplyAdd(r, -alpha, z);
        if (r.infiniteNorm() <= 1e-6) { break; }

        applyPreconditioner();

        double sigma_new = z.innerProduct(r);
        double beta = sigma_new / sigma;
        s.setMultiplyAdd(z, beta, s);
        sigma = sigma_new;

        iter++;
    }

    // set pressure of empty cells to zero
    iterate([&](size_t i, size_t j) {
        if (cell(i,j) == CellType::EMPTY) {
            p(i,j) = 0;
        }
    });

    // update velocity using the solved pressure
    {
        double scale = dt / (rho * dx);
        for (size_t j = 1; j < SIZEY; j++) {
            for (size_t i = 1; i < SIZEX; i++) {
                if (cell(i-1,j) == CellType::FLUID || cell(i,j) == CellType::FLUID) {
                    if (cell(i-1,j) == CellType::SOLID || cell(i,j) == CellType::SOLID) {
                        mac.u(i,j) = 0; // usolid(i,j);
                    }
                    else if (cell(i-1,j) == CellType::EMPTY) {
                        // account for ghost pressures
                        mac.u(i,j) -= scale * utils::min((phi(i,j) - phi(i-1,j)) / phi(i,j), 1e3) * p(i,j);
                    }
                    else if (cell(i,j) == CellType::EMPTY) {
                        // account for ghost pressures
                        mac.u(i,j) -= scale * utils::min((phi(i,j) - phi(i-1,j)) / phi(i-1,j), 1e3) * p(i-1,j);
                    }
                    else {
                        mac.u(i,j) -= scale * (p(i,j) - p(i-1,j));
                    }
                }
                else {
                    // mark as unknown?
                    // mac.u(i,j) = 0;
                }
                if ((cell(i,j-1) == CellType::FLUID || cell(i,j) == CellType::FLUID)) {
                    if (cell(i,j-1) == CellType::SOLID || cell(i,j) == CellType::SOLID) {
                        mac.v(i,j) = 0;
                    }
                    else if (cell(i,j-1) == CellType::EMPTY) {
                        // account for ghost pressures
                        mac.v(i,j) -= scale * utils::min((phi(i,j) - phi(i,j-1)) / phi(i,j), 1e3) * p(i,j);
                    }
                    else if (cell(i,j) == CellType::EMPTY) {
                        // account for ghost pressures
                        mac.v(i,j) -= scale * utils::min((phi(i,j) - phi(i,j-1)) / phi(i,j-1), 1e3) * p(i,j-1);
                    }
                    else {
                        mac.v(i,j) -= scale * (p(i,j) - p(i,j-1));
                    }
                }
                else {
                    // mark as unknown?
                    // mac.v(i,j) = 0;
                }
            }
        }
        iterate([&](size_t i, size_t j) {

        });
    }

    gridStack.free();
}

void WaterSim2D::updateCells() {
    srand(time(NULL));
    Grid2D<CellType>* oldCell = new Grid2D<CellType>();
    defer {delete oldCell;};
    oldCell->copyFrom(cell);
    iterate([&](size_t i, size_t j) {
        if(cell(i,j) == CellType::FLUID) {
            cell(i,j) = CellType::EMPTY;
        }
    });
    for (int i = 0; i < particles.size; i++) {
        auto& pos = particles[i];
        auto k1 = mac.velInterp(pos);
        auto k2 = mac.velInterp(pos + 0.5*dt*k1);
        auto k3 = mac.velInterp(pos + 0.75*dt*k2);
        pos = clampPos(pos + (2./9.)*dt*k1 + (3./9.)*dt*k2 + (4./9.)*dt*k3);
        int x = (int)(pos.x/dx), y = (int)(pos.y/dx);
        if (cell(x,y) == CellType::EMPTY) {
            cell(x,y) = CellType::FLUID;
        }
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

hmm_vec2 WaterSim2D::getGridCenter() {
    return HMM_Vec2(SIZEX * dx / 2, SIZEY * dx / 2);
}

void WaterSim2D::createLevelSet() {
    auto* t = new Grid2D<size_t>();
    defer {delete t;};

    iterate([&](size_t i, size_t j) {
        phi(i,j) = HUGE_VAL;
    });
    iterate([&](size_t i, size_t j) {
        (*t)(i,j) = (size_t)-1;
    });
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
                if (e != (size_t)-1) {
                    double d = sqrt((pos.x - i*dx)*(pos.x - i*dx) + (pos.y - j*dx)*(pos.y - j*dx)) - dr;
                    if (d < phi(i,j)) {
                        phi(i,j) = d;
                        (*t)(i,j) = e;
                    }
                }
            }
        }
    });

}

// TODO: Still buggy
void WaterSim2D::updateLevelSet() {
    auto isSurface = new Grid2D<bool>();
    memset(isSurface->data, false, SIZEX*SIZEY);
    defer {delete isSurface;};
    for (size_t j = 0; j < SIZEY - 1; j++) {
        for (size_t i = 0; i < SIZEX - 1; i++) {
            if (phi(i,j) * phi(i+1,j) < 0.0) {
                double theta1 = phi(i,j) / (phi(i,j) - phi(i+1,j));
                double theta2 = phi(i+1,j) / (phi(i+1,j) - phi(i,j));
                phi(i,j) = utils::sgn(phi(i,j)) * theta1 * dx;
                phi(i+1,j) = utils::sgn(phi(i+1,j)) * theta2 * dx;
                (*isSurface)(i,j) = true;
                (*isSurface)(i+1,j) = true;
            }
            if (phi(i,j) * phi(i,j+1) < 0.0) {
                double theta1 = phi(i,j) / (phi(i,j) - phi(i,j+1));
                double theta2 = phi(i,j+1) / (phi(i,j+1) - phi(i,j));
                phi(i,j) = utils::sgn(phi(i,j)) * theta1 * dx;
                phi(i,j+1) = utils::sgn(phi(i,j+1)) * theta2 * dx;
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

    for (int i = 0; i < 2; i++) {
        for (size_t j = 0; j < SIZEY - 1; j++) {
            for (size_t i = 0; i < SIZEX - 1; i++) {
                if (phi(i,j) >= 0) continue;
                double phi0 = utils::min(abs(phi(i+1,j)), abs(phi(i,j+1)));
                double phi1 = utils::max(abs(phi(i+1,j)), abs(phi(i,j+1)));
                double d = phi0 + dx;
                if (d > phi1) {
                    d = 0.5 * (phi0 + phi1 + sqrt(2*dx*dx - (phi1 - phi0)*(phi1 - phi0)));
                }
                if (d < abs(phi(i,j))) {
                    phi(i,j) = utils::sgn(phi(i,j)) * d;
                }
            }
        }
        for (size_t j = 0; j < SIZEY - 1; j++) {
            for (size_t i = SIZEX; i-- > 1;) {
                if (phi(i,j) >= 0) continue;
                double phi0 = utils::min(abs(phi(i-1,j)), abs(phi(i,j+1)));
                double phi1 = utils::max(abs(phi(i-1,j)), abs(phi(i,j+1)));
                double d = phi0 + dx;
                if (d > phi1) {
                    d = 0.5 * (phi0 + phi1 + sqrt(2*dx*dx - (phi1 - phi0)*(phi1 - phi0)));
                }
                if (d < abs(phi(i,j))) {
                    phi(i,j) = utils::sgn(phi(i,j)) * d;
                }
            }
        }
        for (size_t j = SIZEY; j-- > 1;) {
            for (size_t i = 0; i < SIZEX - 1; i++) {
                if (phi(i,j) >= 0) continue;
                double phi0 = utils::min(abs(phi(i+1,j)), abs(phi(i,j-1)));
                double phi1 = utils::max(abs(phi(i+1,j)), abs(phi(i,j-1)));
                double d = phi0 + dx;
                if (d > phi1) {
                    d = 0.5 * (phi0 + phi1 + sqrt(2*dx*dx - (phi1 - phi0)*(phi1 - phi0)));
                }
                if (d < abs(phi(i,j))) {
                    phi(i,j) = utils::sgn(phi(i,j)) * d;
                }
            }
        }
        for (size_t j = SIZEY; j-- > 1;) {
            for (size_t i = SIZEX; i-- > 1;) {
                if (phi(i,j) >= 0) continue;
                double phi0 = utils::min(abs(phi(i-1,j)), abs(phi(i,j-1)));
                double phi1 = utils::max(abs(phi(i-1,j)), abs(phi(i,j-1)));
                double d = phi0 + dx;
                if (d > phi1) {
                    d = 0.5 * (phi0 + phi1 + sqrt(2*dx*dx - (phi1 - phi0)*(phi1 - phi0)));
                }
                if (d < abs(phi(i,j))) {
                    phi(i,j) = utils::sgn(phi(i,j)) * d;
                }
            }
        }
    }

    auto oldPhi = new Grid2D<double>();
    defer {delete oldPhi;};

    oldPhi->copyFrom(phi);
    for (size_t j = 0; j < SIZEY; j++) {
        for (size_t i = 0; i < SIZEX; i++) {
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
