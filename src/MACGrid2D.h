//
// Created by lasagnaphil on 10/2/18.
//
#ifndef FLUID_SIM_MACGRID2D_H
#define FLUID_SIM_MACGRID2D_H

#include <cstddef>
#include <cmath>
#include <mathfu/glsl_mappings.h>
#include <math/Utils.h>
#include "Array2D.h"

template <size_t NX, size_t NY>
struct MACGrid2D {
    Array2D<double, NX + 1, NY> u;
    Array2D<double, NX, NY + 1> v;
    double dx = 1;

    mathfu::vec2d velU(size_t i, size_t j) {
        if (i > 0 && i < NX)
            return mathfu::vec2d(
                    u(i,j),
                    0.25 * (v(i-1,j) + v(i-1,j+1) + v(i,j) + v(i,j+1))
            );
        else if (i == 0)
            return mathfu::vec2d(
                    u(i,j),
                    0.25 * (v(i,j) + v(i,j+1))
            );
        else
            return mathfu::vec2d(
                    u(i,j),
                    0.25 * (v(i-1,j) + v(i-1,j+1))
            );
    }

    mathfu::vec2d velV(size_t i, size_t j) {
        if (j > 0 && j < NY)
            return mathfu::vec2d(
                0.25 * (u(i,j-1) + u(i+1,j-1) + u(i,j) + u(i+1,j)),
                v(i,j)
            );
        else if (j == 0)
            return mathfu::vec2d(
                0.25 * (u(i,j) + u(i+1,j)),
                v(i,j)
            );
        else
            return mathfu::vec2d(
                0.25 * (u(i,j-1) + u(i+1,j-1)),
                v(i,j)
            );
    }

    mathfu::vec2d vel(size_t i, size_t j) {
        return {
            0.5 * (u(i+1,j) + u(i,j)),
            0.5 * (v(i,j+1) + v(i,j))
        };
    }

    double velInterpU(mathfu::vec2d pos) {
        pos /= dx;
        pos.y -= 0.5;
        pos.x = utils::clamp<double>(pos.x, 0, NX);
        pos.y = utils::clamp<double>(pos.y, 0, NY);
        return u.triCubic(pos);
    }

    double velInterpV(mathfu::vec2d pos) {
        pos /= dx;
        pos.x -= 0.5;
        pos.x = utils::clamp<double>(pos.x, 0, NX);
        pos.y = utils::clamp<double>(pos.y, 0, NY);
        return v.triCubic(pos);
    }

    mathfu::vec2d velInterp(mathfu::vec2d pos) {
        return mathfu::vec2d(velInterpU(pos), velInterpV(pos));
    }

    double velDiv(size_t i, size_t j) {
        return (u(i+1,j)-u(i,j) + v(i,j+1)-v(i,j))/dx;
    }
};

#endif //FLUID_SIM_MACGRID2D_H
