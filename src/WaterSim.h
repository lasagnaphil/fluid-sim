//
// Created by lasagnaphil on 9/13/18.
//

#ifndef FLUID_SIM_WATERSIM_H
#define FLUID_SIM_WATERSIM_H

#include <Eigen/Dense>
#include <cstddef>
#include "Array3D.h"
#include "Utils.h"

class WaterSim {
    static constexpr int SIZEX = 10;
    static constexpr int SIZEY = 10;
    static constexpr int SIZEZ = 10;
    static Eigen::Matrix<double, 64, 64> CUBIC_INTERP_MAT;

    template <typename T>
    using Grid3D = Array3D<T, SIZEX, SIZEY, SIZEZ>;

    enum CellType : uint8_t {
        EMPTY, FLUID, SOLID
    };
    Array3D<double, SIZEX, SIZEY, SIZEZ> p = {};
    Array3D<double, SIZEX + 1, SIZEY, SIZEZ> u = {};
    Array3D<double, SIZEX, SIZEY + 1, SIZEZ> v = {};
    Array3D<double, SIZEX, SIZEY, SIZEZ + 1> w = {};

    Array3D<CellType, SIZEX, SIZEY, SIZEZ> cell = {};

    Eigen::Vector3d gravity = {0, -9.8, 0};
    double rho = 1.0;
    double dx = 1.0;
    double dt = 0.016;

public:
    void setup() {
        iterate([&](size_t i, size_t j, size_t k) {
            if (i == 0 || i == SIZEX - 1 || j == 0 || j == SIZEY - 1 || k == 0 || k == SIZEZ - 1) {
                cell(i, j, k) = SOLID;
            }
            else if (i + j < SIZEY / 2) {
                cell(i, j, k) = FLUID;
            }
            else {
                cell(i, j, k) = EMPTY;
            }
        });
    }
    Eigen::Vector3d clampPos(Eigen::Vector3d x) {
        Eigen::Vector3d clamped;
        clamped(0) = clamp(x(0), 0.0, (double)SIZEX);
        clamped(1) = clamp(x(1), 0.0, (double)SIZEY);
        clamped(2) = clamp(x(2), 0.0, (double)SIZEZ);
        return clamped;
    }

    void applyAdvection() {
        using Eigen::Vector3d;
        iterateU([&](size_t i, size_t j, size_t k) {
            Vector3d u_pos = Vector3d((double)i - 0.5, j, k);
            Vector3d x_mid = u_pos - 0.5 * dt * velU(i, j, k);
            Vector3d x_mid_cl = clampPos(x_mid);
            Vector3d x_p = u_pos - dt * velInterp(x_mid_cl);
            u(i,j,k) = velInterpU(x_p);
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

    void applyGravity() {
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

    void applyProjection() {
        using Eigen::Vector3d;
        Grid3D<double> Adiag = {};
        Grid3D<double> Aplusi = {};
        Grid3D<double> Aplusj = {};
        Grid3D<double> Aplusk = {};
        Grid3D<double> rhs = {};

        // calculate lhs (matrix A)
        double scaleA = dt / (rho * dx * dx);
        iterate([&](size_t i, size_t j, size_t k) {
            if (cell(i,j,k) == FLUID) {
                if (cell(i+1,j,k) == FLUID) {
                    Adiag(i,j,k) += scaleA;
                    Adiag(i+1,j,k) += scaleA;
                    Aplusi(i,j,k) -= scaleA;
                }
                else if (cell(i+1,j,k) == EMPTY) {
                    Adiag(i,j,k) += scaleA;
                }
                if (cell(i,j+1,k) == FLUID) {
                    Adiag(i,j,k) += scaleA;
                    Adiag(i,j+1,k) += scaleA;
                    Aplusj(i,j,k) -= scaleA;
                }
                else if (cell(i,j+1,k) == EMPTY) {
                    Adiag(i,j,k) += scaleA;
                }
                if (cell(i,j,k+1) == FLUID) {
                    Adiag(i,j,k) += scaleA;
                    Adiag(i,j,k+1) += scaleA;
                    Aplusk(i,j,k) -= scaleA;
                }
                else if (cell(i,j,k+1) == EMPTY) {
                    Adiag(i,j,k) += scaleA;
                }
            }
        });
        // calculate rhs
        double scaleRHS = 1 / dx;
        iterate([&](size_t i, size_t j, size_t k) {
            rhs(i, j, k) =
                    (u(i + 1, j, k) - u(i, j, k) + v(i, j + 1, k) - v(i, j, k) + w(i, j, k + 1) - w(i, j, k)) / dx;
            // modify rhs to account for solid velocities
            if (cell(i, j, k) == SOLID) {
                // TODO: use usolid, vsolid, wsolid instead of 0
                if (cell(i - 1, j, k) == SOLID) {
                    rhs(i, j, k) -= scaleRHS * (u(i, j, k) - 0);
                }
                if (cell(i + 1, j, k) == SOLID) {
                    rhs(i, j, k) += scaleRHS * (u(i + 1, j, k) - 0);
                }
                if (cell(i, j - 1, k) == SOLID) {
                    rhs(i, j, k) -= scaleRHS * (v(i, j, k) - 0);
                }
                if (cell(i, j + 1, k) == SOLID) {
                    rhs(i, j, k) += scaleRHS * (v(i, j + 1, k) - 0);
                }
                if (cell(i, j, k - 1) == SOLID) {
                    rhs(i, j, k) -= scaleRHS * (w(i, j, k) - 0);
                }
                if (cell(i, j, k + 1) == SOLID) {
                    rhs(i, j, k) += scaleRHS * (w(i, j, k + 1) - 0);
                }
            }
        });

        // find preconditioner
        Grid3D<double> precon;
        {
            constexpr double tau = 0.97;
            constexpr double sigma = 0.25;
            double e;
            iterate([&](size_t i, size_t j, size_t k) {
                if (cell(i, j, k) == FLUID) {
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
    }

    void runFrame() {
        applyAdvection();
        applyGravity();
        applyProjection();
    }

    void draw() {

    }

private:

    Grid3D<double> applyA(const Grid3D<double>& r,
                          const Grid3D<double>& Adiag, const Grid3D<double>& Aplusi, const Grid3D<double>& Aplusj, const Grid3D<double>& Aplusk) {
        Grid3D<double> z;
        iterate([&](size_t i, size_t j, size_t k) {
            z(i,j,k) = Adiag(i,j,k)*r(i,j,k) + Aplusi(i-1,j,k)*r(i-1,j,k) + Aplusi(i,j,k)*r(i+1,j,k)
                                             + Aplusj(i,j-1,k)*r(i,j-1,k) + Aplusj(i,j,k)*r(i,j+1,k)
                                             + Aplusk(i,j,k-1)*r(i,j,k-1) + Aplusk(i,j,k)*r(i,j,k+1);
        });
        return z;
    }

    Grid3D<double> applyPreconditioner(const Grid3D<double>& r,
            const Grid3D<double>& precon, const Grid3D<double>& Aplusi, const Grid3D<double>& Aplusj, const Grid3D<double>& Aplusk) {
        Grid3D<double> q = {};
        iterate([&](size_t i, size_t j, size_t k) {
            if (cell(i,j,k) == FLUID) {
                double t = r(i,j,k) - (i > 0? Aplusi(i-1,j,k) * precon(i-1,j,k) * q(i-1,j,k) : 0)
                                    - (j > 0? Aplusj(i,j-1,k) * precon(i,j-1,k) * q(i,j-1,k) : 0)
                                    - (k > 0? Aplusk(i,j,k-1) * precon(i,j,k-1) * q(i,j,k-1) : 0);
                q(i,j,k) = t * precon(i,j,k);
            }
        });
        Grid3D<double> z = {};
        iterateBackwards([&](size_t i, size_t j, size_t k) {
            if (cell(i,j,k) == FLUID) {
                double t = r(i,j,k) - (i < SIZEX - 1? Aplusi(i,j,k) * precon(i,j,k) * z(i+1,j,k) : 0)
                                    - (j < SIZEY - 1? Aplusi(i,j,k) * precon(i,j,k) * z(i,j+1,k) : 0)
                                    - (k < SIZEZ - 1? Aplusi(i,j,k) * precon(i,j,k) * z(i,j,k+1) : 0);
                z(i,j,k) = t * precon(i,j,k);
            }
        });
        return z;
    }

    Eigen::Vector3d vel(size_t i, size_t j, size_t k) {
        return 0.5 * Eigen::Vector3d(
                u(i+1,j,k) + u(i,j,k),
                v(i,j+1,k) + v(i,j,k),
                w(i,j,k+1) + w(i,j,k));
    }

    Eigen::Vector3d velU(size_t i, size_t j, size_t k) {
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

    Eigen::Vector3d velV(size_t i, size_t j, size_t k) {
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

    Eigen::Vector3d velW(size_t i, size_t j, size_t k) {
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

    template <typename Fun>
    void iterateU(Fun f) {
        for (size_t i = 0; i < SIZEX + 1; i++)
            for (size_t j = 0; j < SIZEY; j++)
                for (size_t k = 0; k < SIZEZ; k++)
                    f(i, j, k);
    }
    template <typename Fun>
    void iterateV(Fun f) {
        for (size_t i = 0; i < SIZEX; i++)
            for (size_t j = 0; j < SIZEY + 1; j++)
                for (size_t k = 0; k < SIZEZ; k++)
                    f(i, j, k);
    }

    template <typename Fun>
    void iterateW(Fun f) {
        for (size_t i = 0; i < SIZEX; i++)
            for (size_t j = 0; j < SIZEY; j++)
                for (size_t k = 0; k < SIZEZ + 1; k++)
                    f(i, j, k);
    }

    template <typename Fun>
    void iterate(Fun f) {
        for (size_t i = 0; i < SIZEX; i++)
            for (size_t j = 0; j < SIZEY; j++)
                for (size_t k = 0; k < SIZEZ; k++)
                    f(i, j, k);
    }

    template <typename Fun>
    void iterateBackwards(Fun f) {
        for (size_t i = SIZEX; i-- > 0;) {
            for (size_t j = SIZEY; j-- > 0;) {
                for (size_t k = SIZEZ; k-- > 0;) {
                    f(i, j, k);
                }
            }
        }
    }

    template <typename Fun>
    double triCubicInterp(Eigen::Vector3d pos, Fun f) {
        Eigen::Matrix<double, 64, 1> interpValues;
        Eigen::Vector3i flr((int)floor(pos(0)), (int)floor(pos(1)), (int)floor(pos(2)));
        flr(0) = clamp(flr(0), 0, SIZEX - 1);
        flr(1) = clamp(flr(1), 0, SIZEY - 1);
        flr(2) = clamp(flr(2), 0, SIZEZ - 1);
        Eigen::Vector3d offset = pos - flr.cast<double>();
        if (offset(0) < 0 || offset(0) >= 1) offset(0) = 0;
        if (offset(1) < 0 || offset(1) >= 1) offset(1) = 0;
        if (offset(2) < 0 || offset(2) >= 1) offset(2) = 0;

        for (int k = 0; k < 4; k++) {
            for (int j = 0; j < 4; j++) {
                for (int i = 0; i < 4; i++) {
                    double gridVal = f(clamp(flr(0) + i - 1, 0, SIZEX - 1), clamp(flr(1) + j - 1, 0, SIZEY - 1), clamp(flr(2) + k - 1, 0, SIZEZ - 1));
                    interpValues(16 * i + 4 * j + k) = gridVal;
                }
            }
        }
        Eigen::Matrix<double, 64, 1> coeffs = CUBIC_INTERP_MAT * interpValues;
        double result;
        for (int k = 0; k < 4; k++) {
            for (int j = 0; j < 4; j++) {
                for (int i = 0; i < 4; i++) {
                    result += (powi(offset(0), i) * powi(offset(1), j) * powi(offset(2), k)) * coeffs(i * 16 + j * 4 + k);
                }
            }
        }
        return result;
    }

    double velInterpU(const Eigen::Vector3d& pos) {
        return triCubicInterp(pos, [this](size_t i, size_t j, size_t k) { return 0.5 * (u(i,j,k) + u(i+1,j,k)); });
    }

    double velInterpV(const Eigen::Vector3d& pos) {
        return triCubicInterp(pos, [this](size_t i, size_t j, size_t k) { return 0.5 * (v(i,j,k) + v(i,j+1,k)); });
    }

    double velInterpW(const Eigen::Vector3d& pos) {
        return triCubicInterp(pos, [this](size_t i, size_t j, size_t k) { return 0.5 * (w(i,j,k) + w(i,j,k+1)); });
    }

    Eigen::Vector3d velInterp(const Eigen::Vector3d& pos) {
        return Eigen::Vector3d(velInterpU(pos), velInterpV(pos), velInterpW(pos));
    }
};


#endif //FLUID_SIM_WATERSIM_H
