//
// Created by lasagnaphil on 10/2/18.
//
#ifndef FLUID_SIM_MACGRID2D_H
#define FLUID_SIM_MACGRID2D_H

#include <cstddef>
#include <cmath>
#include <math_utils.h>
#include "Array2D.h"

struct MACGrid2D {
    Array2D<double> u;
    Array2D<double> v;
    size_t NX;
    size_t NY;
    double dx = 1;

    static MACGrid2D create(size_t nx, size_t ny, double dx) {
        return {
            Array2D<double>::create(nx + 1, ny),
            Array2D<double>::create(nx, ny + 1),
            nx, ny, dx
        };
    }

    void free() {
        u.free();
        v.free();
    }

    void copyFrom(const MACGrid2D& rhs) {
        u.copyFrom(rhs.u);
        v.copyFrom(rhs.v);
    }

    vec2d velU(size_t i, size_t j) {
        if (i > 0 && i < NX)
            return vec2d{
                    u(i,j),
                    0.25 * (v(i-1,j) + v(i-1,j+1) + v(i,j) + v(i,j+1))
            };
        else if (i == 0)
            return vec2d{
                    u(i,j),
                    0.25 * (v(i,j) + v(i,j+1))
            };
        else
            return vec2d{
                    u(i,j),
                    0.25 * (v(i-1,j) + v(i-1,j+1))
            };
    }

    vec2d velV(size_t i, size_t j) {
        if (j > 0 && j < NY)
            return vec2d{
                0.25 * (u(i,j-1) + u(i+1,j-1) + u(i,j) + u(i+1,j)),
                v(i,j)
            };
        else if (j == 0)
            return vec2d{
                0.25 * (u(i,j) + u(i+1,j)),
                v(i,j)
            };
        else
            return vec2d{
                0.25 * (u(i,j-1) + u(i+1,j-1)),
                v(i,j)
            };
    }

    vec2d vel(size_t i, size_t j) {
        return {
            0.5 * (u(i+1,j) + u(i,j)),
            0.5 * (v(i,j+1) + v(i,j))
        };
    }

    double velInterpU(vec2d pos) {
        pos /= dx;
        pos.y -= 0.5;
        pos.x = aml::clamp<double>(pos.x, 1e-6, NX - 1 - 1e-6);
        pos.y = aml::clamp<double>(pos.y, 1e-6, NY - 1 - 1e-6);
        return u.triCubic(pos);
    }

    double velInterpV(vec2d pos) {
        pos /= dx;
        pos.x -= 0.5;
        pos.x = aml::clamp<double>(pos.x, 1e-6, NX - 1 - 1e-6);
        pos.y = aml::clamp<double>(pos.y, 1e-6, NY - 1 - 1e-6);
        return v.triCubic(pos);
    }

    vec2d velInterp(vec2d pos) {
        return vec2d {velInterpU(pos), velInterpV(pos)};
    }

    double velDiv(size_t i, size_t j) {
        return (u(i+1,j)-u(i,j) + v(i,j+1)-v(i,j))/dx;
    }
};

#endif //FLUID_SIM_MACGRID2D_H
