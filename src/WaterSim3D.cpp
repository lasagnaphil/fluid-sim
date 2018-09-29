//
// Created by lasagnaphil on 9/13/18.
//
#include "WaterSim3D.h"
#include "FirstPersonCamera.h"
#include "InputManager.h"

#define EIGEN_USE_MKL_ALL
#include "Eigen/SparseCore"
#include "Eigen/IterativeLinearSolvers"

void WaterSim3D::setup() {
    mac.iterate([&](size_t i, size_t j, size_t k) {
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

Vector3d WaterSim3D::clampPos(const Vector3d& x) {
    Vector3d clamped;
    clamped.x = utils::clamp<double>(x.x, 0.0, SIZEX);
    clamped.y = utils::clamp<double>(x.y, 0.0, SIZEY);
    clamped.z = utils::clamp<double>(x.z, 0.0, SIZEZ);
    return clamped;
}

void WaterSim3D::runFrame() {
    applyAdvection();
    applyGravity();
    applyProjection();
}

void WaterSim3D::update() {
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

void WaterSim3D::debugPrint() {
    log_info("Printing water sim states: ");
    for (size_t i = 0; i < SIZEX; i++) {
        for (size_t j = 0; j < SIZEY; j++) {
            for (size_t k = 0; k < SIZEZ; k++) {
                Vector3d vec = mac.vel(i,j,k);
                printf("(%10f, %10f, %10f)\n", vec.x, vec.y, vec.z);
            }
        }
    }
}

void WaterSim3D::applyAdvection() {
    mac.iterateU([&](size_t i, size_t j, size_t k) {
        Vector3d u_pos = Vector3d::create(i, (double)j + 0.5, (double)k + 0.5);
        Vector3d x_mid = u_pos - 0.5 * dt * mac.velU(i, j, k);
        Vector3d x_mid_cl = clampPos(x_mid);
        Vector3d x_p = u_pos - dt * mac.velInterp(x_mid_cl);
        mac.u(i,j,k) = mac.velInterpU(x_p);
    });
    mac.iterateV([&](size_t i, size_t j, size_t k) {
        Vector3d v_pos = Vector3d::create((double)i + 0.5, j, (double)k + 0.5);
        Vector3d x_mid = v_pos - 0.5 * dt * mac.velV(i, j, k);
        Vector3d x_mid_cl = clampPos(x_mid);
        Vector3d x_p = v_pos - dt * mac.velInterp(x_mid_cl);
        mac.v(i,j,k) = mac.velInterpV(x_p);
    });
    mac.iterateW([&](size_t i, size_t j, size_t k) {
        Vector3d w_pos = Vector3d::create((double)i + 0.5, (double)j + 0.5, k);
        Vector3d x_mid = w_pos - 0.5 * dt * mac.velW(i, j, k);
        Vector3d x_mid_cl = clampPos(x_mid);
        Vector3d x_p = w_pos - dt * mac.velInterp(x_mid_cl);
        mac.w(i,j,k) = mac.velInterpW(x_p);
    });
}

void WaterSim3D::applyGravity() {
    mac.iterateU([&](size_t i, size_t j, size_t k) {
        mac.u(i,j,k) += dt * gravity(0);
    });
    mac.iterateV([&](size_t i, size_t j, size_t k) {
        mac.v(i,j,k) += dt * gravity(1);
    });
    mac.iterateW([&](size_t i, size_t j, size_t k) {
        mac.w(i,j,k) += dt * gravity(2);
    });
}

void WaterSim3D::applyProjection() {
    using Triplet = Eigen::Triplet<double>;
    auto triplets = Vec<Triplet>::create(5*SIZEX*SIZEY*SIZEZ);

    // calculate lhs (matrix A)
    double scaleA = dt / (rho * mac.dx * mac.dx);
    mac.iterate([&](size_t i, size_t j, size_t k) {
        // note: if index is out of bounds, regard the cell as EMPTY
        size_t idx = k * SIZEY * SIZEX + j * SIZEX + i;
        double A_diag = 0, A_plusi = 0, A_plusj = 0, A_plusk = 0;
        if (cell(i,j,k) == CellType::FLUID) {
            if (cell(i-1,j,k) == CellType::FLUID) {
                A_diag += scaleA;
            }
            if (cell(i+1,j,k) == CellType::FLUID) {
                A_diag += scaleA;
                A_plusi = -scaleA;
            }
            else if (cell(i+1,j,k) == CellType::EMPTY) {
                A_diag += scaleA;
            }
            if (cell(i,j-1,k) == CellType::FLUID) {
                A_diag += scaleA;
            }
            if (cell(i,j+1,k) == CellType::FLUID) {
                A_diag += scaleA;
                A_plusj = -scaleA;
            }
            else if (cell(i,j+1,k) == CellType::EMPTY) {
                A_diag += scaleA;
            }
            if (cell(i,j,k-1) == CellType::FLUID) {
                A_diag += scaleA;
            }
            if (cell(i,j,k+1) == CellType::FLUID) {
                A_diag += scaleA;
                A_plusk = -scaleA;
            }
            else if (cell(i,j,k+1) == CellType::EMPTY) {
                A_diag += scaleA;
            }
        }
        if (A_diag != 0) {
            triplets.push(Triplet(idx, idx, A_diag));
        }
        if (A_plusi != 0) {
            triplets.push(Triplet(idx, idx + 1, A_plusi));
            triplets.push(Triplet(idx + 1, idx, A_plusi));
        }
        if (A_plusj != 0) {
            triplets.push(Triplet(idx, idx + SIZEX, A_plusj));
            triplets.push(Triplet(idx + SIZEX, idx, A_plusj));
        }
        if (A_plusk != 0) {
            triplets.push(Triplet(idx, idx + SIZEY*SIZEX, A_plusk));
            triplets.push(Triplet(idx + SIZEY*SIZEX, idx, A_plusk));
        }
    });

    Eigen::SparseMatrix<double> A(SIZEX*SIZEY*SIZEZ, SIZEX*SIZEY*SIZEZ);
    A.setFromTriplets(triplets.data, triplets.data + triplets.size);
    triplets.free();

    Eigen::Matrix<double, SIZEX*SIZEY*SIZEZ, 1> rhs = {};
    // calculate rhs
    {
        double scale = 1 / mac.dx;
        mac.iterate([&](size_t i, size_t j, size_t k) {
            size_t idx = k * SIZEY * SIZEX + j * SIZEX + i;
            if (cell(i,j,k) == CellType::FLUID) {
                rhs(idx) = -mac.velDiv(i,j,k);
                // modify rhs to account for solid velocities
                // TODO: use usolid, vsolid, wsolid instead of 0
                if (cell(i-1,j,k) == CellType::SOLID) {
                    rhs(idx) -= scale * (mac.u(i,j,k) - 0);
                }
                if (cell(i+1,j,k) == CellType::SOLID) {
                    rhs(idx) += scale * (mac.u(i+1,j,k) - 0);
                }
                if (cell(i,j-1,k) == CellType::SOLID) {
                    rhs(idx) -= scale * (mac.v(i,j,k) - 0);
                }
                if (cell(i,j+1,k) == CellType::SOLID) {
                    rhs(idx) += scale * (mac.v(i,j+1,k) - 0);
                }
                if (cell(i,j,k-1) == CellType::SOLID) {
                    rhs(idx) -= scale * (mac.w(i,j,k) - 0);
                }
                if (cell(i,j,k+1) == CellType::SOLID) {
                    rhs(idx) += scale * (mac.w(i,j,k+1) - 0);
                }
            }

        });
    }

    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower|Eigen::Upper> cg;
    cg.compute(A);
    Eigen::Map<Eigen::Matrix<double, SIZEX*SIZEY*SIZEZ, 1>> result((double*)p.data);
    result = cg.solve(rhs);

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
        double scale = dt / (rho * mac.dx);
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
}

WaterSim3D::Grid3D<double> WaterSim3D::applyA(const WaterSim3D::Grid3D<double> &r, const WaterSim3D::Grid3D<double> &Adiag,
                                          const WaterSim3D::Grid3D<double> &Aplusi,
                                          const WaterSim3D::Grid3D<double> &Aplusj,
                                          const WaterSim3D::Grid3D<double> &Aplusk) {
    Grid3D<double> z;
    mac.iterate([&](size_t i, size_t j, size_t k) {
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

WaterSim3D::Grid3D<double>
WaterSim3D::applyPreconditioner(const WaterSim3D::Grid3D<double> &r, const WaterSim3D::Grid3D<double> &precon,
                              const WaterSim3D::Grid3D<double> &Aplusi, const WaterSim3D::Grid3D<double> &Aplusj,
                              const WaterSim3D::Grid3D<double> &Aplusk) {
    Grid3D<double> q = {};
    mac.iterate([&](size_t i, size_t j, size_t k) {
        if (cell(i,j,k) == CellType::FLUID) {
            double t = r(i,j,k) - (i > 0? Aplusi(i-1,j,k) * precon(i-1,j,k) * q(i-1,j,k) : 0)
                       - (j > 0? Aplusj(i,j-1,k) * precon(i,j-1,k) * q(i,j-1,k) : 0)
                       - (k > 0? Aplusk(i,j,k-1) * precon(i,j,k-1) * q(i,j,k-1) : 0);
            q(i,j,k) = t * precon(i,j,k);
        }
    });
    Grid3D<double> z = {};
    mac.iterateBackwards([&](size_t i, size_t j, size_t k) {
        if (cell(i,j,k) == CellType::FLUID) {
            double t = q(i,j,k) - (i < SIZEX - 1? Aplusi(i,j,k) * precon(i,j,k) * z(i+1,j,k) : 0)
                       - (j < SIZEY - 1? Aplusi(i,j,k) * precon(i,j,k) * z(i,j+1,k) : 0)
                       - (k < SIZEZ - 1? Aplusi(i,j,k) * precon(i,j,k) * z(i,j,k+1) : 0);
            z(i,j,k) = t * precon(i,j,k);
        }
    });
    return z;
}

