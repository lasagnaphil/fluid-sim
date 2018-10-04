//
// Created by lasagnaphil on 2018-09-18.
//

#include <Defer.h>
#include <ctime>
#include "WaterSim2D.h"
#include "InputManager.h"

void WaterSim2D::setup() {
    mac.iterate([&](size_t i, size_t j) {
        if (i == 0 || i == SIZEX - 1 ||
            j == 0 || j == SIZEY - 1) {
            cell(i, j) = CellType::SOLID;
        }
        else if (i + j < SIZEY * 3 / 4) {
            cell(i, j) = CellType::FLUID;
        }
        else {
            cell(i, j) = CellType::EMPTY;
        }
    });
}

Vector2d WaterSim2D::clampPos(Vector2d pos) {
    Vector2d clamped = {};
    clamped.x = utils::clamp(pos.x, 0.0, (double)SIZEX);
    clamped.y = utils::clamp(pos.y, 0.0, (double)SIZEY);
    return clamped;
}

void WaterSim2D::runFrame() {
    applyAdvection();
    applyGravity();
    applyProjection();
    updateCells();
    rendered = false;
    currentFrame++;
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
        nextStage = (nextStage + 1) % 3;
        rendered = false;
    }
    */
    runFrame();
}

void WaterSim2D::applyAdvection() {
    mac.iterateU([&](size_t i, size_t j) {
        Vector2d u_pos = Vector2d::create((double)i, (double)j + 0.5);
        Vector2d x_mid = u_pos - 0.5 * dt * mac.velU(i, j);
        Vector2d x_mid_cl = clampPos(x_mid);
        Vector2d x_p = u_pos - dt * mac.velInterp(x_mid_cl);
        mac.u(i,j) = mac.velInterpU(x_p);
    });
    mac.iterateV([&](size_t i, size_t j) {
        Vector2d v_pos = Vector2d::create((double)i + 0.5, (double)j);
        Vector2d x_mid = v_pos - 0.5 * dt * mac.velV(i, j);
        Vector2d x_mid_cl = clampPos(x_mid);
        Vector2d x_p = v_pos - dt * mac.velInterp(x_mid_cl);
        mac.v(i,j) = mac.velInterpV(x_p);
    });
}

void WaterSim2D::applyGravity() {
    mac.iterateU([&](size_t i, size_t j) {
        mac.u(i,j) += dt * gravity.x;
    });
    mac.iterateV([&](size_t i, size_t j) {
        mac.v(i,j) += dt * gravity.y;
    });
}

void WaterSim2D::applyProjection() {
    auto gridStack = Vec<Grid2D<double>>::create(9);

    auto& Adiag = gridStack.newItem();
    auto& Ax = gridStack.newItem();
    auto& Ay = gridStack.newItem();

    // calculate lhs (matrix A)
    double scaleA = dt / (rho * mac.dx * mac.dx);
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
        double scale = 1 / mac.dx;
        mac.iterate([&](size_t i, size_t j) {
            if (cell(i,j) == CellType::FLUID) {
                rhs(i,j) = -mac.velDiv(i,j);
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
    int maxIters = 10;
    int iter = 0;
    while (iter < maxIters) {
        // apply A
        mac.iterate([&](size_t i, size_t j) {
            z(i,j) = Adiag(i,j)*r(i,j)
                       + (i > 0? Ax(i-1,j)*r(i-1,j) : 0)
                       + (i < SIZEX - 1? Ax(i,j)*r(i+1,j) : 0)
                       + (j > 0? Ay(i,j-1)*r(i,j-1) : 0)
                       + (j < SIZEY - 1? Ay(i,j)*r(i,j+1) : 0);
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
        double scale = dt / (rho * mac.dx);
        mac.iterate([&](size_t i, size_t j) {
            if (i > 0 && (cell(i-1,j) == CellType::FLUID || cell(i,j) == CellType::FLUID)) {
                if (cell(i-1,j) == CellType::SOLID || cell(i,j) == CellType::SOLID)
                    mac.u(i,j) = 0; // usolid(i,j);
                else
                    mac.u(i,j) -= scale * (p(i,j) - p(i-1,j));
            }
            else {
                // mark as unknown?
                mac.u(i,j) = 0;
            }
            if (j > 0 && (cell(i,j-1) == CellType::FLUID || cell(i,j) == CellType::FLUID)) {
                if (cell(i,j-1) == CellType::SOLID || cell(i,j) == CellType::SOLID)
                    mac.v(i,j) = 0;
                else
                    mac.v(i,j) -= scale * (p(i,j) - p(i,j-1));
            }
            else {
                // mark as unknown?
                mac.v(i,j) = 0;
            }
        });
    }

    gridStack.free();
}

inline double randf() {
    return ((double)rand()/(double)RAND_MAX) * 0.5;
}

void WaterSim2D::updateCells() {
    srand(time(NULL));
    Grid2D<CellType>* oldCell = new Grid2D<CellType>();
    defer {delete oldCell;};
    oldCell->copyFrom(cell);
    mac.iterate([&](size_t i, size_t j) {
        if(cell(i,j) == CellType::FLUID) {
            cell(i,j) == CellType::EMPTY;
        }
    });
    mac.iterate([&](size_t i, size_t j) {
        if ((*oldCell)(i,j) == CellType::FLUID) {
            Vector2d particles[4];
            particles[0] = Vector2d::create((double)i + randf(), (double)j + randf());
            particles[1] = Vector2d::create((double)i + 0.5 + randf(), (double)j + randf());
            particles[2] = Vector2d::create((double)i + randf(), (double)j + 0.5 + randf());
            particles[3] = Vector2d::create((double)i + 0.5 + randf(), (double)j + 0.5 + randf());
            for (int l = 0; l < 4; l++) {
                auto& pos = particles[l];
                pos = clampPos(pos + dt * mac.velInterp(pos));
                int x = (int)pos.x, y = (int)pos.y;
                if (cell(x,y) == CellType::EMPTY) {
                    cell(x,y) = CellType::FLUID;
                }
            }
        }
    });
}

