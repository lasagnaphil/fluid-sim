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
    mac.iterate([&](size_t i, size_t j) {
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
    mac.iterate([&](size_t i, size_t j) {
        if (cell(i,j) == CellType::FLUID) {
            particles.push(Vector2d::create((double)i + randf(), (double)j + randf()) * dx);
            particles.push(Vector2d::create((double)i + 0.5 + randf(), (double)j + randf()) * dx);
            particles.push(Vector2d::create((double)i + randf(), (double)j + 0.5 + randf()) * dx);
            particles.push(Vector2d::create((double)i + 0.5 + randf(), (double)j + 0.5 + randf()) * dx);
        }
    });
}

Vector2d WaterSim2D::clampPos(Vector2d pos) {
    Vector2d clamped = {};
    clamped.x = utils::clamp(pos.x, 0.0, (double)SIZEX * dx - 1e-10);
    clamped.y = utils::clamp(pos.y, 0.0, (double)SIZEY * dx - 1e-10);
    return clamped;
}

void WaterSim2D::runFrame() {
    applyAdvection();
    applyGravity();
    applyProjection();
    updateCells();
    rendered = false;
    currentTime += dt;
}

void WaterSim2D::update() {
    /*
    static int nextStage = 0;
    auto inputMgr = InputManager::get();
    if (inputMgr->isKeyEntered(SDL_SCANCODE_RETURN)) {
        if (nextStage == 0) {
            applyAdvection();
        }
        else if (nextStage == 1) {
            applyGravity();
        }
        else if (nextStage == 2) {
            applyProjection();
        }
        else if (nextStage == 3) {
            updateCells();
        }
        nextStage = (nextStage + 1) % 4;
        rendered = false;
    }
     */
    runFrame();
}

void WaterSim2D::applyAdvection() {
    double C = 5;
    mac.iterateU([&](size_t i, size_t j) {
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
    mac.iterateV([&](size_t i, size_t j) {
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
    mac.iterateV([&](size_t i, size_t j) {
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
    mac.iterate([&](size_t i, size_t j) {
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
        mac.iterate([&](size_t i, size_t j) {
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
        mac.iterate([&](size_t i, size_t j) {
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
        mac.iterate([&](size_t i, size_t j) {
            if (cell(i,j) == CellType::FLUID) {
                double t = r(i,j) - (i > 0? Ax(i-1,j) * precon(i-1,j) * q(i-1,j) : 0)
                           - (j > 0? Ay(i,j-1) * precon(i,j-1) * q(i,j-1) : 0);
                q(i,j) = t * precon(i,j);
            }
        });
        mac.iterateBackwards([&](size_t i, size_t j) {
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
        mac.iterate([&](size_t i, size_t j) {
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

    // update velocity using the solved pressure
    {
        double scale = dt / (rho * dx);
        mac.iterate([&](size_t i, size_t j) {
            if (i > 0 && (cell(i-1,j) == CellType::FLUID || cell(i,j) == CellType::FLUID)) {
                if (cell(i-1,j) == CellType::SOLID || cell(i,j) == CellType::SOLID)
                    mac.u(i,j) = 0; // usolid(i,j);
                else
                    mac.u(i,j) -= scale * (p(i,j) - p(i-1,j));
            }
            else {
                // mark as unknown?
                // mac.u(i,j) = 0;
            }
            if (j > 0 && (cell(i,j-1) == CellType::FLUID || cell(i,j) == CellType::FLUID)) {
                if (cell(i,j-1) == CellType::SOLID || cell(i,j) == CellType::SOLID)
                    mac.v(i,j) = 0;
                else
                    mac.v(i,j) -= scale * (p(i,j) - p(i,j-1));
            }
            else {
                // mark as unknown?
                // mac.v(i,j) = 0;
            }
        });
    }

    gridStack.free();
}

void WaterSim2D::updateCells() {
    srand(time(NULL));
    Grid2D<CellType>* oldCell = new Grid2D<CellType>();
    defer {delete oldCell;};
    oldCell->copyFrom(cell);
    mac.iterate([&](size_t i, size_t j) {
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
    // set pressure of empty cells to zero
    mac.iterate([&](size_t i, size_t j) {
        if (cell(i,j) == CellType::EMPTY) {
            p(i,j) = 0;
        }
    });
}

double WaterSim2D::avgPressure() {
    double avgP = 0.0f;
    mac.iterate([&](size_t i, size_t j) {
        avgP += p(i,j);
    });
    avgP /= (SIZEX*SIZEY);
    return avgP;
}

double WaterSim2D::avgPressureInFluid() {
    double avgP = 0.0f;
    size_t count = 0;
    mac.iterate([&](size_t i, size_t j) {
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

