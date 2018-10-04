//
// Created by lasagnaphil on 10/2/18.
//
#ifndef FLUID_SIM_MACGRID2D_H
#define FLUID_SIM_MACGRID2D_H

#include <cstddef>
#include <cmath>
#include <math/Vector2.h>
#include <math/Utils.h>
#include "Array2D.h"

template <size_t NX, size_t NY>
struct MACGrid2D {
    Array2D<double, NX + 1, NY> u;
    Array2D<double, NX, NY + 1> v;
    double dx = 1;

    template <typename Fun>
    void iterateU(Fun f) const {
        for (size_t j = 0; j < NY; j++) {
            for (size_t i = 0; i < NX + 1; i++) {
                f(i, j);
            }
        }
    }

    template <typename Fun>
    void iterateV(Fun f) {
        for (size_t j = 0; j < NY + 1; j++) {
            for (size_t i = 0; i < NX; i++) {
                f(i, j);
            }
        }
    }

    template <typename Fun>
    void iterate(Fun f) {
        for (size_t j = 0; j < NY; j++) {
            for (size_t i = 0; i < NX; i++) {
                f(i, j);
            }
        }
    }

    template <typename Fun>
    void iterateBackwards(Fun f) {
        for (size_t j = NY; j-- > 0;) {
            for (size_t i = NX; i-- > 0;) {
                f(i, j);
            }
        }
    }

    Vector2d velU(size_t i, size_t j) {
        if (i > 0 && i < NX)
            return Vector2d::create(
                    u(i,j),
                    0.25 * (v(i-1,j) + v(i-1,j+1) + v(i,j) + v(i,j+1))
            );
        else if (i == 0)
            return Vector2d::create(
                    u(i,j),
                    0.25 * (v(i,j) + v(i,j+1))
            );
        else
            return Vector2d::create(
                    u(i,j),
                    0.25 * (v(i-1,j) + v(i-1,j+1))
            );
    }

    Vector2d velV(size_t i, size_t j) {
        if (j > 0 && j < NY)
            return Vector2d::create(
                0.25 * (u(i,j-1) + u(i+1,j-1) + u(i,j) + u(i+1,j)),
                v(i,j)
            );
        else if (j == 0)
            return Vector2d::create(
                0.25 * (u(i,j) + u(i+1,j)),
                v(i,j)
            );
        else
            return Vector2d::create(
                0.25 * (u(i,j-1) + u(i+1,j-1)),
                v(i,j)
            );
    }

    Vector2d vel(size_t i, size_t j) {
        return {
            0.5 * (u(i+1,j) + u(i,j)),
            0.5 * (v(i,j+1) + v(i,j))
        };
    }

    double velInterpU(Vector2d pos) {
        pos /= dx;
        pos.y -= 0.5;
        return u.triCubic(pos);
    }

    double velInterpV(Vector2d pos) {
        pos /= dx;
        pos.x -= 0.5;
        return v.triCubic(pos);
    }

    Vector2d velInterp(Vector2d pos) {
        return Vector2d::create(velInterpU(pos), velInterpV(pos));
    }

    double velDiv(size_t i, size_t j) {
        return (u(i+1,j)-u(i,j) + v(i,j+1)-v(i,j))/dx;
    }
};

#endif //FLUID_SIM_MACGRID2D_H
