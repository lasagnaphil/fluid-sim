//
// Created by lasagnaphil on 9/30/18.
//

#ifndef FLUID_SIM_MACGRID3D_H
#define FLUID_SIM_MACGRID3D_H

#include <cstddef>
#include "Array3D.h"
#include <math_utils.h>

struct MACGrid3D {
    Array3D<double> u, v, w;
    int sizeX, sizeY, sizeZ;

    static MACGrid3D create(int nx, int ny, int nz) {
        return {
            Array3D<double>::create(nx+1, ny, nz),
            Array3D<double>::create(nx, ny+1, nz),
            Array3D<double>::create(nx, ny, nz+1),
            nx, ny, nz
        };
    }

    void free() {
        u.free();
        v.free();
        w.free();
    }

    template <typename Fun>
    void iterateU(Fun f) {
        for (int k = 0; k < sizeZ; k++)
            for (int j = 0; j < sizeY; j++)
                for (int i = 0; i < sizeX + 1; i++)
                    f(i, j, k);
    }
    template <typename Fun>
    void iterateV(Fun f) {
        for (int k = 0; k < sizeZ; k++)
            for (int j = 0; j < sizeY + 1; j++)
                for (int i = 0; i < sizeX; i++)
                    f(i, j, k);
    }

    template <typename Fun>
    void iterateW(Fun f) {
        for (int k = 0; k < sizeZ + 1; k++)
            for (int j = 0; j < sizeY; j++)
                for (int i = 0; i < sizeX; i++)
                    f(i, j, k);
    }

    template <typename Fun>
    void iterate(Fun f) {
        for (int k = 0; k < sizeZ; k++)
            for (int j = 0; j < sizeY; j++)
                for (int i = 0; i < sizeX; i++)
                    f(i, j, k);
    }

    template <typename Fun>
    void iterateBackwards(Fun f) {
        for (int k = sizeZ; k-- > 0;) {
            for (int j = sizeY; j-- > 0;) {
                for (int i = sizeX; i-- > 0;) {
                    f(i, j, k);
                }
            }
        }
    }

    vec3d vel(int i, int j, int k) {
        return 0.5 * vec3d{
                u(i+1,j,k) + u(i,j,k),
                v(i,j+1,k) + v(i,j,k),
                w(i,j,k+1) + w(i,j,k)};
    }

    vec3d velU(int i, int j, int k) {
        if (i > 0 && i < sizeX)
            return vec3d{
                    u(i,j,k),
                    0.25 * (v(i-1,j,k) + v(i-1,j+1,k) + v(i,j,k) + v(i,j+1,k)),
                    0.25 * (w(i-1,j,k) + w(i-1,j,k+1) + w(i,j,k) + w(i,j,k+1))
            };
        else if (i == 0)
            return vec3d{
                    u(i,j,k),
                    0.25 * (v(i,j,k) + v(i,j+1,k)),
                    0.25 * (w(i,j,k) + w(i,j,k+1))
            };
        else
            return vec3d{
                    u(i,j,k),
                    0.25 * (v(i-1,j,k) + v(i-1,j+1,k)),
                    0.25 * (w(i-1,j,k) + w(i-1,j,k+1))
            };
    }

    vec3d velV(int i, int j, int k) {
        if (j > 0 && j < sizeY)
            return vec3d{
                    0.25 * (u(i,j-1,k) + u(i,j,k) + u(i+1,j-1,k) + u(i+1,j,k)),
                    v(i,j,k),
                    0.25 * (w(i,j-1,k) + w(i,j-1,k+1) + w(i,j,k) + w(i,j,k+1))
            };
        else if (j == 0)
            return vec3d{
                    0.25 * (u(i,j,k) + u(i+1,j,k)),
                    v(i,j,k),
                    0.25 * (w(i,j,k) + w(i,j,k+1))
            };
        else
            return vec3d{
                    0.25 * (u(i,j-1,k) + u(i+1,j-1,k)),
                    v(i,j,k),
                    0.25 * (w(i,j-1,k) + w(i,j-1,k+1))
            };
    }

    vec3d velW(int i, int j, int k) {
        if (k > 0 && k < sizeZ)
            return vec3d{
                    0.25 * (u(i,j,k-1) + u(i+1,j,k-1) + u(i,j,k) + u(i+1,j,k)),
                    0.25 * (v(i,j,k-1) + v(i,j+1,k-1) + v(i,j,k) + v(i,j+1,k)),
                    w(i,j,k)
            };
        else if (k == 0)
            return vec3d{
                    0.25 * (u(i,j,k) + u(i+1,j,k)),
                    0.25 * (v(i,j,k) + v(i,j+1,k)),
                    w(i,j,k)
            };
        else
            return vec3d{
                    0.25 * (u(i,j,k-1) + u(i+1,j,k-1)),
                    0.25 * (v(i,j,k-1) + v(i,j+1,k-1)),
                    w(i,j,k)
            };
    }

    double velInterpU(vec3d pos) {
        pos.y -= 0.5;
        pos.z -= 0.5;
        return u.triCubic(pos);
    }

    double velInterpV(vec3d pos) {
        pos.z -= 0.5;
        pos.x -= 0.5;
        return v.triCubic(pos);
    }

    double velInterpW(vec3d pos) {
        pos.x -= 0.5;
        pos.y -= 0.5;
        return w.triCubic(pos);
    }

    vec3d velInterp(vec3d pos) {
        return vec3d {velInterpU(pos), velInterpV(pos), velInterpW(pos)};
    }

    double velDiv(int i, int j, int k) {
        return (u(i+1,j,k) - u(i,j,k) + v(i,j+1,k) - v(i,j,k) + w(i,j,k+1) - w(i,j,k));
    }
};

#endif //FLUID_SIM_MACGRID3D_H
