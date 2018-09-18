//
// Created by lasagnaphil on 9/13/18.
//
#include "WaterSim.h"
#include "FirstPersonCamera.h"
#include "InputManager.h"
#include "Eigen/SparseCore"
#include "Eigen/IterativeLinearSolvers"

void WaterSim::setup() {
    iterate([&](size_t i, size_t j, size_t k) {
        /*
        if (i == 0 || i == 1 || i == SIZEX - 2 || i == SIZEX - 1 ||
            j == 0 || j == 1 || j == SIZEY - 2 || j == SIZEY - 1 ||
            k == 0 || k == 1 || k == SIZEZ - 2 || k == SIZEZ - 1) {
            */
        if (i == 0 || i == SIZEX - 1 ||
            j == 0 || j == SIZEY - 1 ||
            k == 0 || k == SIZEZ - 1) {
            cell(i, j, k) = CellType::SOLID;
        }
        else if (i + j < SIZEY * 3 / 4) {
            cell(i, j, k) = CellType::FLUID;
        }
        else {
            cell(i, j, k) = CellType::EMPTY;
        }
    });
}

Eigen::Vector3d WaterSim::clampPos(const Eigen::Vector3d& x) {
    Eigen::Vector3d clamped;
    clamped(0) = clamp(x(0), 0.0, (double)SIZEX);
    clamped(1) = clamp(x(1), 0.0, (double)SIZEY);
    clamped(2) = clamp(x(2), 0.0, (double)SIZEZ);
    return clamped;
}

void WaterSim::runFrame() {
    applyAdvection();
    applyGravity();
    applyProjection();
}

void WaterSim::update() {
    static int stage = 0;
    auto inputMgr = InputManager::get();
    if (inputMgr->isKeyEntered(SDL_SCANCODE_RETURN)) {
        if (stage == 0)
            applyAdvection();
        else if (stage == 1)
            applyGravity();
        else if (stage == 2)
            applyProjection();
        stage = (stage + 1) % 3;
    }
}

void WaterSim::debugPrint() {
    log_info("Printing water sim states: ");
    for (size_t i = 0; i < SIZEX; i++) {
        for (size_t j = 0; j < SIZEY; j++) {
            for (size_t k = 0; k < SIZEZ; k++) {
                Eigen::Vector3d vec = vel(i,j,k);
                printf("(%10f, %10f, %10f)\n", vec(0), vec(1), vec(2));
            }
        }
    }
}

void WaterSim::applyAdvection() {
    using Eigen::Vector3d;
    iterateU([&](size_t i, size_t j, size_t k) {
        Vector3d u_pos = Vector3d((double)i - 0.5, j, k);
        Vector3d x_mid = u_pos - 0.5 * dt * velU(i, j, k);
        Vector3d x_mid_cl = clampPos(x_mid);
        Vector3d x_p = u_pos - dt * velInterp(x_mid_cl);
        double vInterp = velInterpU(x_p);
        u(i,j,k) = vInterp;
    });
    iterateV([&](size_t i, size_t j, size_t k) {
        Vector3d v_pos = Vector3d(i, (double)j - 0.5, k);
        Vector3d x_mid = v_pos - 0.5 * dt * velV(i, j, k);
        Vector3d x_mid_cl = clampPos(x_mid);
        Vector3d x_p = v_pos - dt * velInterp(x_mid_cl);
        v(i,j,k) = velInterpV(x_p);
    });
    iterateW([&](size_t i, size_t j, size_t k) {
        Vector3d w_pos = Vector3d(i, j, (double)k - 0.5);
        Vector3d x_mid = w_pos - 0.5 * dt * velW(i, j, k);
        Vector3d x_mid_cl = clampPos(x_mid);
        Vector3d x_p = w_pos - dt * velInterp(x_mid_cl);
        w(i,j,k) = velInterpW(x_p);
    });
}

void WaterSim::applyGravity() {
    iterateU([&](size_t i, size_t j, size_t k) {
        u(i,j,k) += dt * gravity(0);
    });
    iterateV([&](size_t i, size_t j, size_t k) {
        v(i,j,k) += dt * gravity(1);
    });
    iterateW([&](size_t i, size_t j, size_t k) {
        w(i,j,k) += dt * gravity(2);
    });
}

void WaterSim::applyProjection() {
    using Eigen::Vector3d;
    Eigen::SparseMatrix<double> A(SIZEX*SIZEY*SIZEZ, SIZEX*SIZEY*SIZEZ);

#define A_DIAG(__i,__j,__k) A.insert(__i*SIZEY*SIZEZ + __j*SIZEZ + __k, __i*SIZEY*SIZEZ + __j*SIZEZ + __k)
#define A_PLUSI(__i,__j,__k) A.insert((__i+1)*SIZEY*SIZEZ + __j*SIZEZ + __k, __i*SIZEY*SIZEZ + __j*SIZEZ + __k)
#define A_PLUSJ(__i,__j,__k) A.insert(__i*SIZEY*SIZEZ + (__j+1)*SIZEZ + __k, __i*SIZEY*SIZEZ + __j*SIZEZ + __k)
#define A_PLUSK(__i,__j,__k) A.insert(__i*SIZEY*SIZEZ + __j*SIZEZ + (__k+1), __i*SIZEY*SIZEZ + __j*SIZEZ + __k)
#define A_MINUSI(__i,__j,__k) A.insert((__i+1)*SIZEY*SIZEZ + __j*SIZEZ + __k, __i*SIZEY*SIZEZ + __j*SIZEZ + __k)
#define A_MINUSJ(__i,__j,__k) A.insert(__i*SIZEY*SIZEZ + (__j+1)*SIZEZ + __k, __i*SIZEY*SIZEZ + __j*SIZEZ + __k)
#define A_MINUSK(__i,__j,__k) A.insert(__i*SIZEY*SIZEZ + __j*SIZEZ + (__k+1), __i*SIZEY*SIZEZ + __j*SIZEZ + __k)

    /*
    Grid3D<double> Adiag = {};
    Grid3D<double> Aplusi = {};
    Grid3D<double> Aplusj = {};
    Grid3D<double> Aplusk =DIAG {};
     */
    Grid3D<double> rhs = {};

    // calculate lhs (matrix A)
    double scaleA = dt / (rho * dx * dx);
    iterate([&](size_t i, size_t j, size_t k) {
        // note: if index is out of bounds, regard the cell as EMPTY
        if (cell(i,j,k) == CellType::FLUID) {
            if (cell(i-1,j,k) == CellType::FLUID) {
                A_DIAG(i,j,k) += scaleA;
            }
            if (cell(i+1,j,k) == CellType::FLUID) {
                A_DIAG(i,j,k) += scaleA;
                A_PLUSI(i,j,k) -scaleA;
                A_MINUSI(i,j,k) = -scaleA;
            }
            else if (cell(i+1,j,k) == CellType::EMPTY) {
                A_DIAG(i,j,k) += scaleA;
            }
            if (cell(i,j-1,k) == CellType::FLUID) {
                A_DIAG(i,j,k) += scaleA;
            }
            if (cell(i,j+1,k) == CellType::FLUID) {
                A_DIAG(i,j,k) += scaleA;
                A_PLUSI(i,j,k) = -scaleA;
                A_MINUSI(i,j,k) = -scaleA;
            }
            else if (cell(i+1,j,k) == CellType::EMPTY) {
                A_DIAG(i,j,k) += scaleA;
            }
            if (cell(i,j,k-1) == CellType::FLUID) {
                A_DIAG(i,j,k) += scaleA;
            }
            if (cell(i,j,k+1) == CellType::FLUID) {
                A_DIAG(i,j,k) += scaleA;
                A_PLUSK(i,j,k) = -scaleA;
                A_MINUSK(i,j,k) = -scaleA;
            }
            else if (cell(i,j,k+1) == CellType::EMPTY) {
                A_DIAG(i,j,k) += scaleA;
            }
        }
    });

    // calculate rhs
    {
        double scale = 1 / dx;
        iterate([&](size_t i, size_t j, size_t k) {
            if (cell(i,j,k) == CellType::FLUID) {
                rhs(i, j, k) =
                        -(u(i+1,j,k) - u(i,j,k) + v(i,j+1,k) - v(i,j,k) + w(i,j,k+1) - w(i,j,k)) / dx;
                // modify rhs to account for solid velocities
                // TODO: use usolid, vsolid, wsolid instead of 0
                if (cell(i-1,j,k) == CellType::SOLID) {
                    rhs(i,j,k) -= scale * (u(i,j,k) - 0);
                }
                if (cell(i+1,j,k) == CellType::SOLID) {
                    rhs(i,j,k) += scale * (u(i+1,j,k) - 0);
                }
                if (cell(i,j-1,k) == CellType::SOLID) {
                    rhs(i,j,k) -= scale * (v(i,j,k) - 0);
                }
                if (cell(i,j+1,k) == CellType::SOLID) {
                    rhs(i,j,k) += scale * (v(i,j+1,k) - 0);
                }
                if (cell(i,j,k-1) == CellType::SOLID) {
                    rhs(i,j,k) -= scale * (w(i,j,k) - 0);
                }
                if (cell(i,j,k+1) == CellType::SOLID) {
                    rhs(i,j,k) += scale * (w(i,j,k+1) - 0);
                }
            }

        });
    }
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower|Eigen::Upper> cg;
    cg.compute(A);
    p.data = cg.solve(rhs.data);

    /*
    // find preconditioner
    Grid3D<double> precon = {};
    {
        constexpr double tau = 0.97;
        constexpr double sigma = 0.25;
        double e;
        iterate([&](size_t i, size_t j, size_t k) {
            if (cell(i, j, k) == CellType::FLUID) {
                e = Adiag(i,j,k)
                    - (i > 0? SQUARE(Aplusi(i-1,j,k) * precon(i-1,j,k)) : 0)
                    - (j > 0? SQUARE(Aplusj(i,j-1,k) * precon(i,j-1,k)) : 0)
                    - (k > 0? SQUARE(Aplusk(i,j,k-1) * precon(i,j,k-1)) : 0)
                    - tau * ((i > 0? Aplusi(i-1,j,k)*(Aplusj(i-1,j,k) + Aplusk(i-1,j,k))*SQUARE(precon(i-1,j,k)) : 0)
                             + (j > 0? Aplusj(i,j-1,k)*(Aplusi(i,j-1,k) + Aplusk(i,j-1,k))*SQUARE(precon(i,j-1,k)) : 0)
                             + (k > 0? Aplusk(i,j,k-1)*(Aplusi(i,j,k-1) + Aplusj(i,j,k-1))*SQUARE(precon(i,j,k-1)) : 0));
                if (e < sigma * Adiag(i,j,k)) {
                    e = Adiag(i,j,k);
                }
                precon(i,j,k) = 1 / sqrt(e);
            }
        });
    }

    // use PCG algorithm to solve the linear equation
    Grid3D<double> r = rhs;
    Grid3D<double> z = applyPreconditioner(r, precon, Aplusi, Aplusj, Aplusk);
    Grid3D<double> s = z;
    double sigma = z.innerProduct(r);
    int maxIters = 10;
    int iter = 0;
    while (iter < maxIters) {
        z = applyA(s, Adiag, Aplusi, Aplusj, Aplusk);
        double alpha = sigma / z.innerProduct(s);
        p += alpha * s;
        r -= alpha * z;
        if (r.infiniteNorm() <= 1e-6) { break; }
        z = applyPreconditioner(r, precon, Aplusi, Aplusj, Aplusk);
        double sigma_new = z.innerProduct(r);
        double beta = sigma_new / sigma;
        s = z + beta * s;
        sigma = sigma_new;

        iter++;
    }
     */

    // update velocity using the solved pressure
    {
        double scale = dt / (rho * dx);
        iterate([&](size_t i, size_t j, size_t k) {
            if (i > 0 && (cell(i-1,j,k) == CellType::FLUID || cell(i,j,k) == CellType::FLUID)) {
                if (cell(i-1,j,k) == CellType::SOLID || cell(i,j,k) == CellType::SOLID)
                    u(i,j,k) = 0; // usolid(i,j,k);
                else
                    u(i,j,k) -= scale * (p(i,j,k) - p(i-1,j,k));
            }
            else {
                // mark as unknown?
                u(i,j,k) = 0;
            }
            if (j > 0 && (cell(i,j-1,k) == CellType::FLUID || cell(i,j,k) == CellType::FLUID)) {
                if (cell(i,j-1,k) == CellType::SOLID || cell(i,j,k) == CellType::SOLID)
                    v(i,j,k) = 0;
                else
                    v(i,j,k) -= scale * (p(i,j,k) - p(i,j-1,k));
            }
            else {
                // mark as unknown?
                v(i,j,k) = 0;
            }
            if (k > 0 && (cell(i,j,k-1) == CellType::FLUID || cell(i,j,k) == CellType::FLUID)) {
                if (cell(i,j,k-1) == CellType::SOLID || cell(i,j,k) == CellType::SOLID)
                    w(i,j,k) = 0;
                else
                    w(i,j,k) -= scale * (p(i,j,k) - p(i,j,k-1));
            }
            else {
                // mark as unknown?
                w(i,j,k) = 0;
            }
        });
    }
}

WaterSim::Grid3D<double> WaterSim::applyA(const WaterSim::Grid3D<double> &r, const WaterSim::Grid3D<double> &Adiag,
                                          const WaterSim::Grid3D<double> &Aplusi,
                                          const WaterSim::Grid3D<double> &Aplusj,
                                          const WaterSim::Grid3D<double> &Aplusk) {
    Grid3D<double> z;
    iterate([&](size_t i, size_t j, size_t k) {
        z(i,j,k) = Adiag(i,j,k)*r(i,j,k)
                   + (i > 0? Aplusi(i-1,j,k)*r(i-1,j,k) : 0)
                   + (i < SIZEX - 1? Aplusi(i,j,k)*r(i+1,j,k) : 0)
                   + (j > 0? Aplusj(i,j-1,k)*r(i,j-1,k) : 0)
                   + (j < SIZEY - 1? Aplusj(i,j,k)*r(i,j+1,k) : 0)
                   + (k > 0? Aplusk(i,j,k-1)*r(i,j,k-1) : 0)
                   + (k < SIZEZ - 1? Aplusk(i,j,k)*r(i,j,k+1) : 0);
    });
    return z;
}

WaterSim::Grid3D<double>
WaterSim::applyPreconditioner(const WaterSim::Grid3D<double> &r, const WaterSim::Grid3D<double> &precon,
                              const WaterSim::Grid3D<double> &Aplusi, const WaterSim::Grid3D<double> &Aplusj,
                              const WaterSim::Grid3D<double> &Aplusk) {
    Grid3D<double> q = {};
    iterate([&](size_t i, size_t j, size_t k) {
        if (cell(i,j,k) == CellType::FLUID) {
            double t = r(i,j,k) - (i > 0? Aplusi(i-1,j,k) * precon(i-1,j,k) * q(i-1,j,k) : 0)
                       - (j > 0? Aplusj(i,j-1,k) * precon(i,j-1,k) * q(i,j-1,k) : 0)
                       - (k > 0? Aplusk(i,j,k-1) * precon(i,j,k-1) * q(i,j,k-1) : 0);
            q(i,j,k) = t * precon(i,j,k);
        }
    });
    Grid3D<double> z = {};
    iterateBackwards([&](size_t i, size_t j, size_t k) {
        if (cell(i,j,k) == CellType::FLUID) {
            double t = q(i,j,k) - (i < SIZEX - 1? Aplusi(i,j,k) * precon(i,j,k) * z(i+1,j,k) : 0)
                       - (j < SIZEY - 1? Aplusi(i,j,k) * precon(i,j,k) * z(i,j+1,k) : 0)
                       - (k < SIZEZ - 1? Aplusi(i,j,k) * precon(i,j,k) * z(i,j,k+1) : 0);
            z(i,j,k) = t * precon(i,j,k);
        }
    });
    return z;
}

#undef A_DIAG
#undef A_PLUSI
#undef A_PLUSJ
#undef A_PLUSK
#undef A_MINUSI
#undef A_MINUSJ
#undef A_MINUSK

Eigen::Vector3d WaterSim::vel(size_t i, size_t j, size_t k) {
    return 0.5 * Eigen::Vector3d(
            u(i+1,j,k) + u(i,j,k),
            v(i,j+1,k) + v(i,j,k),
            w(i,j,k+1) + w(i,j,k));
}

Eigen::Vector3d WaterSim::velU(size_t i, size_t j, size_t k) {
    if (i > 0 && i < SIZEX)
        return Eigen::Vector3d(
                u(i,j,k),
                0.25 * (v(i-1,j,k) + v(i-1,j+1,k) + v(i,j,k) + v(i,j+1,k)),
                0.25 * (w(i-1,j,k) + w(i-1,j,k+1) + w(i,j,k) + w(i,j,k+1))
        );
    else if (i == 0)
        return Eigen::Vector3d(
                u(i,j,k),
                0.5 * (v(i,j,k) + v(i,j+1,k)),
                0.5 * (w(i,j,k) + w(i,j,k+1))
        );
    else
        return Eigen::Vector3d(
                u(i,j,k),
                0.5 * (v(i-1,j,k) + v(i-1,j+1,k)),
                0.5 * (w(i-1,j,k) + w(i-1,j,k+1))
        );
}

Eigen::Vector3d WaterSim::velV(size_t i, size_t j, size_t k) {
    if (j > 0 && j < SIZEY)
        return Eigen::Vector3d(
                0.25 * (u(i,j-1,k) + u(i,j,k) + u(i+1,j-1,k) + u(i+1,j,k)),
                v(i,j,k),
                0.25 * (w(i,j-1,k) + w(i,j-1,k+1) + w(i,j,k) + w(i,j,k+1))
        );
    else if (j == 0)
        return Eigen::Vector3d(
                0.5 * (u(i,j,k) + u(i+1,j,k)),
                v(i,j,k),
                0.5 * (w(i,j,k) + w(i,j,k+1))
        );
    else
        return Eigen::Vector3d(
                0.5 * (u(i,j-1,k) + u(i+1,j-1,k)),
                v(i,j,k),
                0.5 * (w(i,j-1,k) + w(i,j-1,k+1))
        );
}

Eigen::Vector3d WaterSim::velW(size_t i, size_t j, size_t k) {
    if (k > 0 && k < SIZEZ)
        return Eigen::Vector3d(
                0.25 * (u(i,j,k-1) + u(i+1,j,k-1) + u(i,j,k) + u(i+1,j,k)),
                0.25 * (v(i,j,k-1) + v(i,j+1,k-1) + v(i,j,k) + v(i,j+1,k)),
                w(i,j,k)
        );
    else if (k == 0)
        return Eigen::Vector3d(
                0.5 * (u(i,j,k) + u(i+1,j,k)),
                0.5 * (v(i,j,k) + v(i,j+1,k)),
                w(i,j,k)
        );
    else
        return Eigen::Vector3d(
                0.5 * (u(i,j,k-1) + u(i+1,j,k-1)),
                0.5 * (v(i,j,k-1) + v(i,j+1,k-1)),
                w(i,j,k)
        );
}

