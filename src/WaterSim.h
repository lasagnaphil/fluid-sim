//
// Created by lasagnaphil on 9/13/18.
//

#ifndef FLUID_SIM_WATERSIM_H
#define FLUID_SIM_WATERSIM_H

#include <Eigen/Dense>
#include <cstddef>
#include <Vec.h>

double powi(double d, int i) {
    double res = 1.0;
    while (i > 0) {
        res *= d;
        i--;
    }
    return res;
}

Eigen::Vector4d catmullRom(double a) {
    Eigen::Vector4d coeffs;
    coeffs(0) = -0.5*a + a*a - 0.5*a*a*a;
    coeffs(1) = 1 - 2.5*a*a + 1.5*a*a*a;
    coeffs(2) = 0.5*a + 2*a*a - 1.5*a*a*a;
    coeffs(3) = -0.5*a*a + 0.5*a*a*a;
    return coeffs;
}


struct WaterSim {
    static constexpr int SIZEX = 100;
    static constexpr int SIZEY = 100;
    static constexpr int SIZEZ = 100;
    static Eigen::Matrix<double, 64, 64> CUBIC_INTERP_MAT;

    void run(float dt) {
        using Eigen::Vector3d;
        Vector3d velA[SIZEX][SIZEY][SIZEZ];
        // advection
        iterateU([&](size_t i, size_t j, size_t k) {
            Vector3d u_pos = Vector3d(i, (double)j + 0.5, (double)k + 0.5);
            Vector3d x_mid = u_pos - 0.5 * dt * velU(i, j, k);
            Vector3d x_p = u_pos - dt * velInterp(x_mid);
            u[i][j][k] = velInterpU(x_p);
        });
        iterateV([&](size_t i, size_t j, size_t k) {
            Vector3d v_pos = Vector3d((double)i + 0.5, j, (double)k + 0.5);
            Vector3d x_mid = v_pos - 0.5 * dt * velV(i, j, k);
            Vector3d x_p = v_pos - dt * velInterp(x_mid);
            v[i][j][k] = velInterpV(x_p);
        });
        iterateW([&](size_t i, size_t j, size_t k) {
            Vector3d w_pos = Vector3d((double)i + 0.5, (double)j + 0.5, k);
            Vector3d x_mid = w_pos - 0.5 * dt * velW(i, j, k);
            Vector3d x_p = w_pos - dt * velInterp(x_mid);
            w[i][j][k] = velInterpW(x_p);
        });
        // apply gravity
        iterateU([&](size_t i, size_t j, size_t k) {
            u[i][j][k] += dt * gravity(0);
        });
        iterateV([&](size_t i, size_t j, size_t k) {
            v[i][j][k] += dt * gravity(1);
        });
        iterateW([&](size_t i, size_t j, size_t k) {
            w[i][j][k] += dt * gravity(2);
        });
    }

private:
    double p[SIZEX][SIZEY][SIZEZ] = {};
    double u[SIZEX + 1][SIZEY][SIZEZ] = {};
    double v[SIZEX][SIZEY + 1][SIZEZ] = {};
    double w[SIZEX][SIZEY][SIZEZ + 1] = {};

    Eigen::Vector3d gravity = {0, -9.8, 0};

    Eigen::Vector3d vel(size_t i, size_t j, size_t k) {
        return 0.5 * Eigen::Vector3d(
                u[i + 1][j][k] + u[i][j][k],
                v[i + 1][j][k] + v[i][j][k],
                w[i + 1][j][k] + w[i][j][k]);
    }

    Eigen::Vector3d vel(const Eigen::Vector3i& pos) {
        return vel(pos(0), pos(1), pos(2));
    }

    Eigen::Vector3d velU(size_t i, size_t j, size_t k) {
        return Eigen::Vector3d(
                u[i][j][k],
                0.25 * (v[i][j][k] + v[i][j+1][k] + v[i+1][j][k] + v[i+1][j+1][k]),
                0.25 * (w[i][j][k] + w[i][j][k + 1] + w[i+1][j][k] + w[i+1][j][k+1])
        );
    }

    Eigen::Vector3d velV(size_t i, size_t j, size_t k) {
        return Eigen::Vector3d(
                0.25 * (u[i][j][k] + u[i][j+1][k] + u[i+1][j][k] + u[i+1][j+1][k]),
                v[i][j][k],
                0.25 * (w[i][j][k] + w[i][j][k+1] + w[i][j+1][k] + w[i][j+1][k+1])
        );
    }

    Eigen::Vector3d velW(size_t i, size_t j, size_t k) {
        return Eigen::Vector3d(
                0.25 * (u[i][j][k] + u[i+1][j][k] + u[i][j][k+1] + u[i+1][j][k+1]),
                0.25 * (v[i][j][k] + v[i][j+1][k] + v[i][j][k+1] + v[i][j+1][k+1]),
                w[i][j][k]
        );
    }

    template <typename Fun>
    void iterateU(Fun f) {
        for (size_t i = 0; i < SIZEX + 1; i++) {
            for (size_t j = 0; j < SIZEY; j++) {
                for (size_t k = 0; k < SIZEZ; k++) {
                    f(i, j, k);
                }
            }
        }
    }

    template <typename Fun>
    void iterateV(Fun f) {
        for (size_t i = 0; i < SIZEX; i++) {
            for (size_t j = 0; j < SIZEY + 1; j++) {
                for (size_t k = 0; k < SIZEZ; k++) {
                    f(i, j, k);
                }
            }
        }
    }

    template <typename Fun>
    void iterateW(Fun f) {
        for (size_t i = 0; i < SIZEX; i++) {
            for (size_t j = 0; j < SIZEY; j++) {
                for (size_t k = 0; k < SIZEZ + 1; k++) {
                    f(i, j, k);
                }
            }
        }
    }

    template <typename Fun>
    double triCubicInterp(Eigen::Vector3d pos, Fun f) {
        Eigen::Matrix<double, 64, 1> interpValues;
        Eigen::Vector3i flr((int)floor(pos(0)), (int)floor(pos(1)), (int)floor(pos(2)));
        Eigen::Vector3d offset = pos - flr.cast<double>();
        for (int k = 0; k < 4; k++) {
            for (int j = 0; j < 4; j++) {
                for (int i = 0; i < 4; i++) {
                    double gridVal = f(flr(0) + i - 1, flr(1) + j - 1, flr(2) + k - 1);
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
        return triCubicInterp(pos, [this](size_t i, size_t j, size_t k) {
            return 0.5 * (u[i + 1][j][k] + u[i][j][k]);
        });
    }

    double velInterpV(const Eigen::Vector3d& pos) {
        return triCubicInterp(pos, [this](size_t i, size_t j, size_t k) {
            return 0.5 * (v[i + 1][j][k] + v[i][j][k]);
        });
    }

    double velInterpW(const Eigen::Vector3d& pos) {
        return triCubicInterp(pos, [this](size_t i, size_t j, size_t k) {
            return 0.5 * (w[i + 1][j][k] + w[i][j][k]);
        });
    }

    Eigen::Vector3d velInterp(const Eigen::Vector3d& pos) {
        return Eigen::Vector3d(velInterpU(pos), velInterpV(pos), velInterpW(pos));
    }
};


#endif //FLUID_SIM_WATERSIM_H
