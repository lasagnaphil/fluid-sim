//
// Created by lasagnaphil on 2018-09-18.
//

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

void WaterSim2D::runFrame() {
    applyAdvection();
    applyGravity();
    applyProjection();
}

void WaterSim2D::update() {
    auto inputMgr = InputManager::get();
    if (inputMgr->isKeyEntered(SDL_SCANCODE_RETURN)) {
        runFrame();
    }
}

void WaterSim2D::applyAdvection() {
    mac.iterateU([&](size_t i, size_t j) {
        Vector2d u_pos = Vector2d::create((double)i - 0.5, j);
        Vector2d x_mid = u_pos - 0.5 * dt * mac.velU(i, j);
        Vector2d x_mid_cl = clampPos(x_mid);
        Vector2d x_p = u_pos - dt * mac.velInterp(x_mid_cl);
        double vInterp = mac.velInterp(x_p).x;
        mac.u(i,j) = vInterp;
    });
    mac.iterateV([&](size_t i, size_t j) {
        Vector2d v_pos = Vector2d::create(i, (double)j - 0.5);
        Vector2d x_mid = v_pos - 0.5 * dt * mac.velV(i, j);
        Vector2d x_mid_cl = clampPos(x_mid);
        Vector2d x_p = v_pos - dt * mac.velInterp(x_mid_cl);
        mac.v(i,j) = mac.velInterp(x_p).y;
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
    // TODO
    /*
    Eigen::SparseMatrix<double> A(SIZEX*SIZEY, SIZEX*SIZEY);
    Eigen::Matrix<double, SIZEX*SIZEY, 1> rhs = {};

#define A_DIAG(__i,__j) A.insert(__j*SIZEX + __i,__j*SIZEX + __i)
#define A_PLUSI(__i,__j,__v) \
A.insert(__j*SIZEX + __i,__j*SIZEX + __i+1) = __v; \
A.insert(__j*SIZEX + __i+1,__j*SIZEX + __i+1) = __v;
#define A_PLUSJ(__i,__j,__v) \
A.insert(__j*SIZEX + __i,(__j+1)*SIZEX + __i) = __v; \
A.insert((__j+1)*SIZEX + __i,__j*SIZEX + __i) = __v;

    double scaleA = dt / (rho * mac.dx * mac.dx);
    mac.iterate([&](size_t i, size_t j) {
        if (cell(i-1,j) == CellType::FLUID) {
            A_DIAG(i,j) += scaleA;
        }
        if (cell(i+1,j) == CellType::FLUID) {
            A_DIAG(i,j) += scaleA;
            A_PLUSI(i,j,-scaleA);
        }
        else if (cell(i+1,j) == CellType::EMPTY) {
            A_DIAG(i,j) += scaleA;
        }
        if (cell(i,j-1) == CellType::FLUID) {
            A_DIAG(i,j) += scaleA;
        }
        if (cell(i,j+1) == CellType::FLUID) {
            A_DIAG(i,j) += scaleA;
            A_PLUSJ(i,j,-scaleA);
        }
        else if (cell(i,j+1) == CellType::EMPTY) {
            A_DIAG(i,j) += scaleA;
        }
    });

#undef A_DIAG
#undef A_PLUSI
#undef A_PLUSJ

    double scale = 1 / mac.dx;
    mac.iterate([&](size_t i, size_t j) {
        if (cell(i,j) == CellType::FLUID) {
            rhs(j*SIZEX+i) = -mac.velDiv(i,j);
            if (cell(i-1,j) == CellType::SOLID) {
                rhs(j*SIZEX+i) -= scale * (mac.u(i,j) - 0);
            }
            if (cell(i+1,j) == CellType::SOLID) {
                rhs(j*SIZEX+i) += scale * (mac.u(i+1,j) - 0);
            }
            if (cell(i,j-1) == CellType::SOLID) {
                rhs(j*SIZEX+i) -= scale * (mac.v(i,j) - 0);
            }
            if (cell(i,j+1) == CellType::SOLID) {
                rhs(j*SIZEX+i) += scale * (mac.v(i,j+1) - 0);
            }
        }
    });

    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower|Eigen::Upper> cg;
    cg.compute(A);
    Eigen::Matrix<double, SIZEX*SIZEY, 1> sol = cg.solve(rhs);
    memcpy(p.data, sol.data(), sizeof(double)*SIZEX*SIZEY);
     */
}
