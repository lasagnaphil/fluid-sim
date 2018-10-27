//
// Created by lasagnaphil on 9/30/18.
//

#ifndef FLUID_SIM_MACGRID3D_H
#define FLUID_SIM_MACGRID3D_H

#include <cstddef>
#include "Array3D.h"
#include <mathfu/glsl_mappings.h>
#include <math/Utils.h>

template <size_t SIZEX, size_t SIZEY, size_t SIZEZ>
struct MACGrid3D {
    Array3D<double, SIZEX + 1, SIZEY, SIZEZ> u = {};
    Array3D<double, SIZEX, SIZEY + 1, SIZEZ> v = {};
    Array3D<double, SIZEX, SIZEY, SIZEZ + 1> w = {};

    template <typename Fun>
    void iterateU(Fun f) {
        for (size_t k = 0; k < SIZEZ; k++)
            for (size_t j = 0; j < SIZEY; j++)
                for (size_t i = 0; i < SIZEX + 1; i++)
                    f(i, j, k);
    }
    template <typename Fun>
    void iterateV(Fun f) {
        for (size_t k = 0; k < SIZEZ; k++)
            for (size_t j = 0; j < SIZEY + 1; j++)
                for (size_t i = 0; i < SIZEX; i++)
                    f(i, j, k);
    }

    template <typename Fun>
    void iterateW(Fun f) {
        for (size_t k = 0; k < SIZEZ + 1; k++)
            for (size_t j = 0; j < SIZEY; j++)
                for (size_t i = 0; i < SIZEX; i++)
                    f(i, j, k);
    }

    template <typename Fun>
    void iterate(Fun f) {
        for (size_t k = 0; k < SIZEZ; k++)
            for (size_t j = 0; j < SIZEY; j++)
                for (size_t i = 0; i < SIZEX; i++)
                    f(i, j, k);
    }

    template <typename Fun>
    void iterateBackwards(Fun f) {
        for (size_t k = SIZEZ; k-- > 0;) {
            for (size_t j = SIZEY; j-- > 0;) {
                for (size_t i = SIZEX; i-- > 0;) {
                    f(i, j, k);
                }
            }
        }
    }

    mathfu::vec3d vel(size_t i, size_t j, size_t k) {
        return 0.5 * mathfu::vec3d(
                u(i+1,j,k) + u(i,j,k),
                v(i,j+1,k) + v(i,j,k),
                w(i,j,k+1) + w(i,j,k));
    }

    mathfu::vec3d velU(size_t i, size_t j, size_t k) {
        if (i > 0 && i < SIZEX)
            return mathfu::vec3d(
                    u(i,j,k),
                    0.25 * (v(i-1,j,k) + v(i-1,j+1,k) + v(i,j,k) + v(i,j+1,k)),
                    0.25 * (w(i-1,j,k) + w(i-1,j,k+1) + w(i,j,k) + w(i,j,k+1))
            );
        else if (i == 0)
            return mathfu::vec3d(
                    u(i,j,k),
                    0.25 * (v(i,j,k) + v(i,j+1,k)),
                    0.25 * (w(i,j,k) + w(i,j,k+1))
            );
        else
            return mathfu::vec3d(
                    u(i,j,k),
                    0.25 * (v(i-1,j,k) + v(i-1,j+1,k)),
                    0.25 * (w(i-1,j,k) + w(i-1,j,k+1))
            );
    }

    mathfu::vec3d velV(size_t i, size_t j, size_t k) {
        if (j > 0 && j < SIZEY)
            return mathfu::vec3d(
                    0.25 * (u(i,j-1,k) + u(i,j,k) + u(i+1,j-1,k) + u(i+1,j,k)),
                    v(i,j,k),
                    0.25 * (w(i,j-1,k) + w(i,j-1,k+1) + w(i,j,k) + w(i,j,k+1))
            );
        else if (j == 0)
            return mathfu::vec3d(
                    0.25 * (u(i,j,k) + u(i+1,j,k)),
                    v(i,j,k),
                    0.25 * (w(i,j,k) + w(i,j,k+1))
            );
        else
            return mathfu::vec3d(
                    0.25 * (u(i,j-1,k) + u(i+1,j-1,k)),
                    v(i,j,k),
                    0.25 * (w(i,j-1,k) + w(i,j-1,k+1))
            );
    }

    mathfu::vec3d velW(size_t i, size_t j, size_t k) {
        if (k > 0 && k < SIZEZ)
            return mathfu::vec3d(
                    0.25 * (u(i,j,k-1) + u(i+1,j,k-1) + u(i,j,k) + u(i+1,j,k)),
                    0.25 * (v(i,j,k-1) + v(i,j+1,k-1) + v(i,j,k) + v(i,j+1,k)),
                    w(i,j,k)
            );
        else if (k == 0)
            return mathfu::vec3d(
                    0.25 * (u(i,j,k) + u(i+1,j,k)),
                    0.25 * (v(i,j,k) + v(i,j+1,k)),
                    w(i,j,k)
            );
        else
            return mathfu::vec3d(
                    0.25 * (u(i,j,k-1) + u(i+1,j,k-1)),
                    0.25 * (v(i,j,k-1) + v(i,j+1,k-1)),
                    w(i,j,k)
            );
    }

    double velInterpU(mathfu::vec3d pos) {
        pos.y -= 0.5;
        pos.z -= 0.5;
        return u.triCubic(pos);
    }

    double velInterpV(mathfu::vec3d pos) {
        pos.z -= 0.5;
        pos.x -= 0.5;
        return v.triCubic(pos);
    }

    double velInterpW(mathfu::vec3d pos) {
        pos.x -= 0.5;
        pos.y -= 0.5;
        return w.triCubic(pos);
    }

    mathfu::vec3d velInterp(mathfu::vec3d pos) {
        return mathfu::vec3d(velInterpU(pos), velInterpV(pos), velInterpW(pos));
    }

    double velDiv(size_t i, size_t j, size_t k) {
        return (u(i+1,j,k) - u(i,j,k) + v(i,j+1,k) - v(i,j,k) + w(i,j,k+1) - w(i,j,k));
    }
};

#endif //FLUID_SIM_MACGRID3D_H
