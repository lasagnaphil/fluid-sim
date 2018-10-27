//
// Created by lasagnaphil on 9/13/18.
//
#include <time.h>
#include "WaterSim3D.h"
#include "FirstPersonCamera.h"
#include "InputManager.h"
#include "Defer.h"
#include "log.h"

using namespace mathfu;

void WaterSim3D::setup() {
    mac.iterate([&](size_t i, size_t j, size_t k) {
        if (i == 0 || i == SIZEX - 1 ||
            j == 0 || j == SIZEY - 1 ||
            k == 0 || k == SIZEZ - 1) {
            cell(i, j, k) = CellType::SOLID;
        }
        else if ((double)i + j < SIZEY * 3 / 4) {
            cell(i, j, k) = CellType::FLUID;
        }
        else {
            cell(i, j, k) = CellType::EMPTY;
        }
    });
}

vec3d WaterSim3D::clampPos(vec3d x) {
    vec3d clamped;
    clamped.x = utils::clamp<double>(x.x, 0.0, SIZEX);
    clamped.y = utils::clamp<double>(x.y, 0.0, SIZEY);
    clamped.z = utils::clamp<double>(x.z, 0.0, SIZEZ);
    return clamped;
}

void WaterSim3D::runFrame() {
    applyAdvection();
    applyGravity();
    applyProjection();
    updateCells();
    rendered = false;
}

void WaterSim3D::update() {
    static int nextStage = 0;
    auto inputMgr = InputManager::get();
    if (inputMgr->isKeyEntered(SDL_SCANCODE_RETURN)) {
        /*
        if (nextStage == 0) {
            applyAdvection();
            stage = Stage::ADVECTION;
        }
        else if (nextStage == 1) {
            applyGravity();
            stage = Stage::GRAVITY;
        }
        else if (nextStage == 2) {
            applyProjection();
            stage = Stage::PROJECTION;
        }
        nextStage = (nextStage + 1) % 3;
        rendered = false;
         */
        runFrame();
    }
}

void WaterSim3D::debugPrint() {
    log_info("Printing water sim states: ");
    for (size_t i = 0; i < SIZEX; i++) {
        for (size_t j = 0; j < SIZEY; j++) {
            for (size_t k = 0; k < SIZEZ; k++) {
                vec3d vec = mac.vel(i,j,k);
                printf("(%10f, %10f, %10f)\n", vec.x, vec.y, vec.z);
            }
        }
    }
}

void WaterSim3D::applyAdvection() {
    mac.iterateU([&](size_t i, size_t j, size_t k) {
        vec3d u_pos = vec3d(i, (double)j + 0.5, (double)k + 0.5);
        vec3d x_mid = u_pos - 0.5 * dt * mac.velU(i, j, k);
        vec3d x_mid_cl = clampPos(x_mid);
        vec3d x_p = u_pos - dt * mac.velInterp(x_mid_cl);
        mac.u(i,j,k) = mac.velInterpU(x_p);
    });
    mac.iterateV([&](size_t i, size_t j, size_t k) {
        vec3d v_pos = vec3d((double)i + 0.5, j, (double)k + 0.5);
        vec3d x_mid = v_pos - 0.5 * dt * mac.velV(i, j, k);
        vec3d x_mid_cl = clampPos(x_mid);
        vec3d x_p = v_pos - dt * mac.velInterp(x_mid_cl);
        mac.v(i,j,k) = mac.velInterpV(x_p);
    });
    mac.iterateW([&](size_t i, size_t j, size_t k) {
        vec3d w_pos = vec3d((double)i + 0.5, (double)j + 0.5, k);
        vec3d x_mid = w_pos - 0.5 * dt * mac.velW(i, j, k);
        vec3d x_mid_cl = clampPos(x_mid);
        vec3d x_p = w_pos - dt * mac.velInterp(x_mid_cl);
        mac.w(i,j,k) = mac.velInterpW(x_p);
    });
}

void WaterSim3D::applyGravity() {
    mac.iterateU([&](size_t i, size_t j, size_t k) {
        mac.u(i,j,k) += dt * gravity.x;
    });
    mac.iterateV([&](size_t i, size_t j, size_t k) {
        mac.v(i,j,k) += dt * gravity.y;
    });
    mac.iterateW([&](size_t i, size_t j, size_t k) {
        mac.w(i,j,k) += dt * gravity.z;
    });
}

void WaterSim3D::applyProjection() {
    auto gridStack = Vec<Grid3D<double>>::create(10);

    auto& Adiag = gridStack.newItem();
    auto& Ax = gridStack.newItem();
    auto& Ay = gridStack.newItem();
    auto& Az = gridStack.newItem();

    // calculate lhs (matrix A)
    double scaleA = dt / (rho * dx * dx);
    mac.iterate([&](size_t i, size_t j, size_t k) {
        // note: if index is out of bounds, regard the cell as EMPTY
        if (cell(i,j,k) == CellType::FLUID) {
            if (cell(i-1,j,k) == CellType::FLUID) {
                Adiag(i,j,k) += scaleA;
            }
            if (cell(i+1,j,k) == CellType::FLUID) {
                Adiag(i,j,k) += scaleA;
                Ax(i,j,k) = -scaleA;
            }
            else if (cell(i+1,j,k) == CellType::EMPTY) {
                Adiag(i,j,k) += scaleA;
            }
            if (cell(i,j-1,k) == CellType::FLUID) {
                Adiag(i,j,k) += scaleA;
            }
            if (cell(i,j+1,k) == CellType::FLUID) {
                Adiag(i,j,k) += scaleA;
                Ay(i,j,k) = -scaleA;
            }
            else if (cell(i,j+1,k) == CellType::EMPTY) {
                Adiag(i,j,k) += scaleA;
            }
            if (cell(i,j,k-1) == CellType::FLUID) {
                Adiag(i,j,k) += scaleA;
            }
            if (cell(i,j,k+1) == CellType::FLUID) {
                Adiag(i,j,k) += scaleA;
                Az(i,j,k) = -scaleA;
            }
            else if (cell(i,j,k+1) == CellType::EMPTY) {
                Adiag(i,j,k) += scaleA;
            }
        }
    });

    auto& rhs = gridStack.newItem();

    // calculate rhs
    {
        double scale = 1 / dx;
        mac.iterate([&](size_t i, size_t j, size_t k) {
            if (cell(i,j,k) == CellType::FLUID) {
                rhs(i,j,k) = -mac.velDiv(i,j,k);
                // modify rhs to account for solid velocities
                // TODO: use usolid, vsolid, wsolid instead of 0
                if (cell(i-1,j,k) == CellType::SOLID) {
                    rhs(i,j,k) -= scale * (mac.u(i,j,k) - 0);
                }
                if (cell(i+1,j,k) == CellType::SOLID) {
                    rhs(i,j,k) += scale * (mac.u(i+1,j,k) - 0);
                }
                if (cell(i,j-1,k) == CellType::SOLID) {
                    rhs(i,j,k) -= scale * (mac.v(i,j,k) - 0);
                }
                if (cell(i,j+1,k) == CellType::SOLID) {
                    rhs(i,j,k) += scale * (mac.v(i,j+1,k) - 0);
                }
                if (cell(i,j,k-1) == CellType::SOLID) {
                    rhs(i,j,k) -= scale * (mac.w(i,j,k) - 0);
                }
                if (cell(i,j,k+1) == CellType::SOLID) {
                    rhs(i,j,k) += scale * (mac.w(i,j,k+1) - 0);
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
        mac.iterate([&](size_t i, size_t j, size_t k) {
            if (cell(i, j, k) == CellType::FLUID) {
                e = Adiag(i,j,k)
                    - (i > 0? SQUARE(Ax(i-1,j,k) * precon(i-1,j,k)) : 0)
                    - (j > 0? SQUARE(Ay(i,j-1,k) * precon(i,j-1,k)) : 0)
                    - (k > 0? SQUARE(Az(i,j,k-1) * precon(i,j,k-1)) : 0)
                    - tau * ((i > 0? Ax(i-1,j,k)*(Ay(i-1,j,k) + Az(i-1,j,k))*SQUARE(precon(i-1,j,k)) : 0)
                             + (j > 0? Ay(i,j-1,k)*(Az(i,j-1,k) + Ax(i,j-1,k))*SQUARE(precon(i,j-1,k)) : 0)
                             + (k > 0? Az(i,j,k-1)*(Ax(i,j,k-1) + Ay(i,j,k-1))*SQUARE(precon(i,j,k-1)) : 0));
                if (e < sigma * Adiag(i,j,k)) {
                    e = Adiag(i,j,k);
                }
                precon(i,j,k) = 1 / sqrt(e);
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
        mac.iterate([&](size_t i, size_t j, size_t k) {
            if (cell(i,j,k) == CellType::FLUID) {
                double t = r(i,j,k) - (i > 0? Ax(i-1,j,k) * precon(i-1,j,k) * q(i-1,j,k) : 0)
                           - (j > 0? Ay(i,j-1,k) * precon(i,j-1,k) * q(i,j-1,k) : 0)
                           - (k > 0? Az(i,j,k-1) * precon(i,j,k-1) * q(i,j,k-1) : 0);
                q(i,j,k) = t * precon(i,j,k);
            }
        });
        mac.iterateBackwards([&](size_t i, size_t j, size_t k) {
            if (cell(i,j,k) == CellType::FLUID) {
                double t = q(i,j,k) - (i < SIZEX - 1? Ax(i,j,k) * precon(i,j,k) * z(i+1,j,k) : 0)
                           - (j < SIZEY - 1? Ay(i,j,k) * precon(i,j,k) * z(i,j+1,k) : 0)
                           - (k < SIZEZ - 1? Az(i,j,k) * precon(i,j,k) * z(i,j,k+1) : 0);
                z(i,j,k) = t * precon(i,j,k);
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
        mac.iterate([&](size_t i, size_t j, size_t k) {
            z(i,j,k) = Adiag(i,j,k)*r(i,j,k)
                       + (i > 0? Ax(i-1,j,k)*r(i-1,j,k) : 0)
                       + (i < SIZEX - 1? Ax(i,j,k)*r(i+1,j,k) : 0)
                       + (j > 0? Ay(i,j-1,k)*r(i,j-1,k) : 0)
                       + (j < SIZEY - 1? Ay(i,j,k)*r(i,j+1,k) : 0)
                       + (k > 0? Az(i,j,k-1)*r(i,j,k-1) : 0)
                       + (k < SIZEZ - 1? Az(i,j,k)*r(i,j,k+1) : 0);
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
        mac.iterate([&](size_t i, size_t j, size_t k) {
            if (i > 0 && (cell(i-1,j,k) == CellType::FLUID || cell(i,j,k) == CellType::FLUID)) {
                if (cell(i-1,j,k) == CellType::SOLID || cell(i,j,k) == CellType::SOLID)
                    mac.u(i,j,k) = 0; // usolid(i,j,k);
                else
                    mac.u(i,j,k) -= scale * (p(i,j,k) - p(i-1,j,k));
            }
            else {
                // mark as unknown?
                mac.u(i,j,k) = 0;
            }
            if (j > 0 && (cell(i,j-1,k) == CellType::FLUID || cell(i,j,k) == CellType::FLUID)) {
                if (cell(i,j-1,k) == CellType::SOLID || cell(i,j,k) == CellType::SOLID)
                    mac.v(i,j,k) = 0;
                else
                    mac.v(i,j,k) -= scale * (p(i,j,k) - p(i,j-1,k));
            }
            else {
                // mark as unknown?
                mac.v(i,j,k) = 0;
            }
            if (k > 0 && (cell(i,j,k-1) == CellType::FLUID || cell(i,j,k) == CellType::FLUID)) {
                if (cell(i,j,k-1) == CellType::SOLID || cell(i,j,k) == CellType::SOLID)
                    mac.w(i,j,k) = 0;
                else
                    mac.w(i,j,k) -= scale * (p(i,j,k) - p(i,j,k-1));
            }
            else {
                // mark as unknown?
                mac.w(i,j,k) = 0;
            }
        });
    }

    gridStack.free();
}

inline double randf() {
    return ((double)rand()/(double)RAND_MAX) * 0.5;
}

void WaterSim3D::updateCells() {
    srand(time(NULL));
    Grid3D<CellType>* oldCell = new Grid3D<CellType>();
    defer {delete oldCell;};
    oldCell->copyFrom(cell);
    mac.iterate([&](size_t i, size_t j, size_t k) {
        if (cell(i,j,k) == CellType::FLUID) {
            cell(i,j,k) = CellType::EMPTY;
        }
    });
    mac.iterate([&](size_t i, size_t j, size_t k) {
        if ((*oldCell)(i,j,k) == CellType::FLUID) {
            vec3d particles[8];
            particles[0] = vec3d((double)i + randf(), (double)j + randf(), (double)k + randf());
            particles[1] = vec3d((double)i + 0.5 + randf(), (double)j + randf(), (double)k + randf());
            particles[2] = vec3d((double)i + randf(), (double)j + 0.5 + randf(), (double)k + randf());
            particles[3] = vec3d((double)i + 0.5 + randf(), (double)j + 0.5 + randf(), (double)k + randf());
            particles[4] = vec3d((double)i + randf(), (double)j + randf(), (double)k + 0.5 + randf());
            particles[5] = vec3d((double)i + 0.5 + randf(), (double)j + randf(), (double)k + 0.5 + randf());
            particles[6] = vec3d((double)i + randf(), (double)j + 0.5 + randf(), (double)k + 0.5 + randf());
            particles[7] = vec3d((double)i + 0.5 + randf(), (double)j + 0.5 + randf(), (double)k + 0.5 + randf());
            for (int l = 0; l < 8; l++) {
                auto& pos = particles[l];
                pos = clampPos(pos + dt * mac.velInterp(pos));
                int x = (int)pos.x, y = (int)pos.y, z = (int)pos.z;
                if (cell(x,y,z) == CellType::EMPTY) {
                    cell(x,y,z) = CellType::FLUID;
                }
            }
        }
    });
}
