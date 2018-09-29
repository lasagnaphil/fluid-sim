//
// Created by lasagnaphil on 9/30/18.
//

#ifndef FLUID_SIM_MACGRID3D_H
#define FLUID_SIM_MACGRID3D_H

#include <cstddef>
#include "Array3D.h"
#include <math/Vector3.h>
#include <math/Utils.h>

template <size_t SIZEX, size_t SIZEY, size_t SIZEZ>
struct MACGrid3D {
    Array3D<double, SIZEX + 1, SIZEY, SIZEZ> u = {};
    Array3D<double, SIZEX, SIZEY + 1, SIZEZ> v = {};
    Array3D<double, SIZEX, SIZEY, SIZEZ + 1> w = {};
    double dx = 0.001;

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

    Vector3d vel(size_t i, size_t j, size_t k) {
        return 0.5 * Vector3d::create(
                u(i+1,j,k) + u(i,j,k),
                v(i,j+1,k) + v(i,j,k),
                w(i,j,k+1) + w(i,j,k));
    }

    Vector3d velU(size_t i, size_t j, size_t k) {
        if (i > 0 && i < SIZEX)
            return Vector3d::create(
                    u(i,j,k),
                    0.25 * (v(i-1,j,k) + v(i-1,j+1,k) + v(i,j,k) + v(i,j+1,k)),
                    0.25 * (w(i-1,j,k) + w(i-1,j,k+1) + w(i,j,k) + w(i,j,k+1))
            );
        else if (i == 0)
            return Vector3d::create(
                    u(i,j,k),
                    0.5 * (v(i,j,k) + v(i,j+1,k)),
                    0.5 * (w(i,j,k) + w(i,j,k+1))
            );
        else
            return Vector3d::create(
                    u(i,j,k),
                    0.5 * (v(i-1,j,k) + v(i-1,j+1,k)),
                    0.5 * (w(i-1,j,k) + w(i-1,j,k+1))
            );
    }

    Vector3d velV(size_t i, size_t j, size_t k) {
        if (j > 0 && j < SIZEY)
            return Vector3d::create(
                    0.25 * (u(i,j-1,k) + u(i,j,k) + u(i+1,j-1,k) + u(i+1,j,k)),
                    v(i,j,k),
                    0.25 * (w(i,j-1,k) + w(i,j-1,k+1) + w(i,j,k) + w(i,j,k+1))
            );
        else if (j == 0)
            return Vector3d::create(
                    0.5 * (u(i,j,k) + u(i+1,j,k)),
                    v(i,j,k),
                    0.5 * (w(i,j,k) + w(i,j,k+1))
            );
        else
            return Vector3d::create(
                    0.5 * (u(i,j-1,k) + u(i+1,j-1,k)),
                    v(i,j,k),
                    0.5 * (w(i,j-1,k) + w(i,j-1,k+1))
            );
    }

    Vector3d velW(size_t i, size_t j, size_t k) {
        if (k > 0 && k < SIZEZ)
            return Vector3d::create(
                    0.25 * (u(i,j,k-1) + u(i+1,j,k-1) + u(i,j,k) + u(i+1,j,k)),
                    0.25 * (v(i,j,k-1) + v(i,j+1,k-1) + v(i,j,k) + v(i,j+1,k)),
                    w(i,j,k)
            );
        else if (k == 0)
            return Vector3d::create(
                    0.5 * (u(i,j,k) + u(i+1,j,k)),
                    0.5 * (v(i,j,k) + v(i,j+1,k)),
                    w(i,j,k)
            );
        else
            return Vector3d::create(
                    0.5 * (u(i,j,k-1) + u(i+1,j,k-1)),
                    0.5 * (v(i,j,k-1) + v(i,j+1,k-1)),
                    w(i,j,k)
            );
    }

    double velInterpU(Vector3d pos) {
        pos.y -= 0.5;
        pos.z -= 0.5;
        return u.triCubic(pos);
    }

    double velInterpV(Vector3d pos) {
        pos.z -= 0.5;
        pos.x -= 0.5;
        return v.triCubic(pos);
    }

    double velInterpW(Vector3d pos) {
        pos.x -= 0.5;
        pos.y -= 0.5;
        return w.triCubic(pos);
    }

    Vector3d velInterp(Vector3d pos) {
        return Vector3d::create(velInterpU(pos), velInterpV(pos), velInterpW(pos));
    }

    double velDiv(size_t i, size_t j, size_t k) {
        return (u(i+1,j,k) - u(i,j,k) + v(i,j+1,k) - v(i,j,k) + w(i,j,k+1) - w(i,j,k)) / dx;
    }
};

#endif //FLUID_SIM_MACGRID3D_H
