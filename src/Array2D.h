//
// Created by lasagnaphil on 10/2/18.
//

#ifndef FLUID_SIM_ARRAY2D_H
#define FLUID_SIM_ARRAY2D_H

#include <cstddef>
#include <cstdlib>
#include <mathfu/glsl_mappings.h>
#include <Map.h>
#include <Queue.h>
#include <Defer.h>
#include <math/Utils.h>
#include <queue>

#include "immintrin.h"

template <typename T, size_t NX, size_t NY>
struct Array2D {
    T data[NY][NX] = {};

    void copyFrom(Array2D& arr) {
        memcpy(data, arr.data, sizeof(T) * NX * NY);
    }

    T& operator()(size_t i, size_t j) {
        return data[j][i];
    }

    const T& operator()(size_t i, size_t j) const {
        return data[j][i];
    }

    void reset() {
        memset(data, 0, sizeof(T) * NY * NX);
    }
};

template <size_t NX, size_t NY>
struct alignas(32) Array2D<double, NX, NY> {
    double data[NY][NX] = {};

    void copyFrom(Array2D& arr) {
        memcpy(data, arr.data, sizeof(double) * NX * NY);
    }

    double& operator()(size_t i, size_t j) {
        return data[j][i];
    }

    const double& operator()(size_t i, size_t j) const {
        return data[j][i];
    }

    void reset() {
        memset(data, 0, sizeof(double) * NY * NX);
    }

    static void* operator new(std::size_t nbytes) noexcept {
        if (void *p = std::aligned_alloc(alignof(Array2D<double, NX, NY>), nbytes)) {
            return p;
        }
        return nullptr;
    }

    static void operator delete(void* p) {
        free(p);
    }

    Array2D& operator+=(const Array2D& rhs) {
#ifdef USE_AVX_SIMD
        for (size_t i = 0; i < NY*NX; i += 4) {
            __m256d asimd = _mm256_load_pd((double*)data + i);
            __m256d bsimd = _mm256_load_pd((double*)rhs.data + i);
            asimd = _mm256_add_pd(asimd, bsimd);
            _mm256_store_pd((double*)data + i, asimd);
        }
#else
        for (size_t j = 0; j < NY; j++)
            for (size_t i = 0; i < NX; i++)
                (*this)(i,j) += rhs(i,j);
#endif
        return *this;
    }

    Array2D& operator/=(const Array2D& rhs) {
#ifdef USE_AVX_SIMD
        for (size_t i = 0; i < NY*NX; i += 4) {
            __m256d asimd = _mm256_load_pd((double*)data + i);
            __m256d bsimd = _mm256_load_pd((double*)rhs.data + i);
            asimd = _mm256_div_pd(asimd, bsimd);
            _mm256_store_pd((double*)data + i, asimd);
        }
#else
        for (size_t j = 0; j < NY; j++)
            for (size_t i = 0; i < NX; i++)
                (*this)(i,j) /= rhs(i,j);
#endif
        return *this;
    }

    void safeDivBy(const Array2D& rhs) {
        for (size_t j = 0; j < NY; j++) {
            for (size_t i = 0; i < NX; i++) {
                if (rhs(i,j) != 0.0)
                    (*this)(i,j) /= rhs(i,j);
                else
                    (*this)(i,j) = 0.0;
            }
        }
    }

    void setMultiplyAdd(Array2D& a, double b, const Array2D& c) {
#ifdef USE_AVX_SIMD
        for (size_t i = 0; i < NY*NX; i += 4) {
            __m256d asimd = _mm256_load_pd((double*)a.data + i);
            __m256d bsimd = _mm256_set1_pd(b);
            __m256d csimd = _mm256_load_pd((double*)c.data + i);
            __m256d res = _mm256_fmadd_pd(bsimd, csimd, asimd);
            _mm256_store_pd((double*)this->data + i, res);
        }
#else
        for (size_t j = 0; j < NY; j++)
            for (size_t i = 0; i < NX; i++)
                (*this)(i,j) = a(i,j) + b * c(i,j);
#endif
    }

    double innerProduct(const Array2D& rhs) const {
#ifdef USE_AVX_SIMD
        union {
            double results[4];
            __m256d results_simd;
        };
        results_simd = _mm256_setzero_pd();
        for (size_t i = 0; i < NY*NX; i += 4) {
            __m256d a = _mm256_load_pd((double*)data + i);
            __m256d b = _mm256_load_pd((double*)rhs.data + i);
            results_simd = _mm256_add_pd(results_simd, _mm256_mul_pd(a, b));
        }
        return (results[0] + results[1]) + (results[2] + results[3]);
#else
        double result = {};
        for (size_t j = 0; j < NY; j++)
            for (size_t i = 0; i < NX; i++)
                result += (*this)(i,j)*rhs(i,j);
        return result;
#endif
    }

    double infiniteNorm() const {
#ifdef USE_AVX_SIMD
        using utils::max;
        union {
            double results[4];
            __m256d results_simd;
        };
        results_simd = _mm256_setzero_pd();
        for (size_t i = 0; i < NY*NX; i += 4) {
            __m256d lhs = _mm256_load_pd((double*)data + i);
            __m256d lhsabs = _mm256_max_pd(_mm256_sub_pd(_mm256_setzero_pd(), lhs), lhs);
            results_simd = _mm256_max_pd(results_simd, lhsabs);
        }
        return max(max(results[0], results[1]), max(results[2], results[3]));
#else
        double result = {};
        for (size_t j = 0; j < NY; j++)
            for (size_t i = 0; i < NX; i++) {
                if (abs((*this)(i,j)) > result) result = abs((*this)(i,j));
            }
        return result;
#endif
    }

    double triCubic(mathfu::vec2d p) {
        int x = (int) p.x, y = (int) p.y;
        if (x < 0 || x >= NX || y < 0 || y >= NY) {
            return 0;
        }
        double dx = p.x - (double)x, dy = p.y - (double)y;

#define CUBE(x)   ((x) * (x) * (x))
#define SQR(x)    ((x) * (x))

#ifdef USE_AVX_SIMD
        union {
            double u[4];
            __m256d usimd;
        };
        union {
            double v[4];
            __m256d vsimd;
        };

        /* factors for Catmull-Rom interpolation */
        u[0] = -0.5 * CUBE (dx) + SQR (dx) - 0.5 * dx;
        u[1] =  1.5 * CUBE (dx) - 2.5 * SQR (dx) + 1;
        u[2] = -1.5 * CUBE (dx) + 2 * SQR (dx) + 0.5 * dx;
        u[3] =  0.5 * CUBE (dx) - 0.5 * SQR (dx);

        v[0] = -0.5 * CUBE (dy) + SQR (dy) - 0.5 * dy;
        v[1] =  1.5 * CUBE (dy) - 2.5 * SQR (dy) + 1;
        v[2] = -1.5 * CUBE (dy) + 2 * SQR (dy) + 0.5 * dy;
        v[3] =  0.5 * CUBE (dy) - 0.5 * SQR (dy);

        using utils::clamp;
        double values[4][4] = {
            {
                data[clamp<int>(y-1, 0, NY-1)][clamp<int>(x-1, 0, NX-1)],
                data[clamp<int>(y-1, 0, NY-1)][clamp<int>(x, 0, NX-1)],
                data[clamp<int>(y-1, 0, NY-1)][clamp<int>(x+1, 0, NX-1)],
                data[clamp<int>(y-1, 0, NY-1)][clamp<int>(x+2, 0, NX-1)],
            },
            {
                data[clamp<int>(y, 0, NY-1)][clamp<int>(x-1, 0, NX-1)],
                data[clamp<int>(y, 0, NY-1)][clamp<int>(x, 0, NX-1)],
                data[clamp<int>(y, 0, NY-1)][clamp<int>(x+1, 0, NX-1)],
                data[clamp<int>(y, 0, NY-1)][clamp<int>(x+2, 0, NX-1)],
            },
            {
                data[clamp<int>(y+1, 0, NY-1)][clamp<int>(x-1, 0, NX-1)],
                data[clamp<int>(y+1, 0, NY-1)][clamp<int>(x, 0, NX-1)],
                data[clamp<int>(y+1, 0, NY-1)][clamp<int>(x+1, 0, NX-1)],
                data[clamp<int>(y+1, 0, NY-1)][clamp<int>(x+2, 0, NX-1)],
            },
            {
                data[clamp<int>(y+2, 0, NY-1)][clamp<int>(x-1, 0, NX-1)],
                data[clamp<int>(y+2, 0, NY-1)][clamp<int>(x, 0, NX-1)],
                data[clamp<int>(y+2, 0, NY-1)][clamp<int>(x+1, 0, NX-1)],
                data[clamp<int>(y+2, 0, NY-1)][clamp<int>(x+2, 0, NX-1)],
            },
        };
        __m256d dot0 = _mm256_mul_pd(usimd, _mm256_load_pd(values[0]));
        __m256d dot1 = _mm256_mul_pd(usimd, _mm256_load_pd(values[1]));
        __m256d dot2 = _mm256_mul_pd(usimd, _mm256_load_pd(values[2]));
        __m256d dot3 = _mm256_mul_pd(usimd, _mm256_load_pd(values[3]));
        __m256d temp01 = _mm256_hadd_pd(dot0, dot1);
        __m256d temp23 = _mm256_hadd_pd(dot2, dot3);
        __m256d swapped = _mm256_permute2f128_pd(temp01, temp23, 0x21);
        __m256d blended = _mm256_blend_pd(temp01, temp23, 0b1100);
        __m256d dotpds = _mm256_add_pd(swapped, blended);
        __m256d final = _mm256_mul_pd(dotpds, vsimd);

        __m128d final_low = _mm256_extractf128_pd(final, 0);
        __m128d final_high = _mm256_extractf128_pd(final, 1);
        __m128d final_sum1 = _mm_add_pd(final_low, final_high);
        __m128d final_swapped = _mm_shuffle_pd(final_sum1, final_sum1, 0b01);
        __m128d dotproduct = _mm_add_pd(final_sum1, final_swapped);
        double res = _mm_cvtsd_f64(dotproduct);
        return res;
#else
        double u[4], v[4];
        double r[4];
        double vox = 0;

        u[0] = -0.5 * CUBE (dx) + SQR (dx) - 0.5 * dx;
        u[1] =  1.5 * CUBE (dx) - 2.5 * SQR (dx) + 1;
        u[2] = -1.5 * CUBE (dx) + 2 * SQR (dx) + 0.5 * dx;
        u[3] =  0.5 * CUBE (dx) - 0.5 * SQR (dx);

        v[0] = -0.5 * CUBE (dy) + SQR (dy) - 0.5 * dy;
        v[1] =  1.5 * CUBE (dy) - 2.5 * SQR (dy) + 1;
        v[2] = -1.5 * CUBE (dy) + 2 * SQR (dy) + 0.5 * dy;
        v[3] =  0.5 * CUBE (dy) - 0.5 * SQR (dy);

        for (int j = 0; j < 4; j++)
        {
            int yp = utils::clamp<int>(y+j-1, 0, NY-1);
            r[j] = 0;
            for (int i = 0; i < 4; i++)
            {
                int xp = utils::clamp<int>(x+i-1, 0, NX-1);
                r[j] += u[i] * data[yp][xp];
            }
            vox += v[j] * r[j];
        }
        return vox;
#endif

#undef CUBE
#undef SQR
    }

    void distribute(mathfu::vec2d pos, double value) {
        int ui = (int) (pos.x);
        int uj = (int) (pos.y);
        auto disp = mathfu::vec2d(pos.x - ui, pos.y - uj);
        double kx1 = 0.5 * (0.5 - disp.x) * (0.5 - disp.x);
        double kx2 = (0.75 - disp.x * disp.x);
        double kx3 = 0.5 * (0.5 + disp.x) * (0.5 + disp.x);
        double ky1 = 0.5 * (0.5 - disp.y) * (0.5 - disp.y);
        double ky2 = (0.75 - disp.y * disp.y);
        double ky3 = 0.5 * (0.5 + disp.y) * (0.5 + disp.y);
        if (disp.x < 0.5 && disp.y < 0.5) {
            (*this)(ui - 1, uj - 1) += kx1 * ky1 * value;
            (*this)(ui - 1, uj    ) += kx1 * ky2 * value;
            (*this)(ui - 1, uj + 1) += kx1 * ky3 * value;
            (*this)(ui    , uj - 1) += kx2 * ky1 * value;
            (*this)(ui    , uj    ) += kx2 * ky2 * value;
            (*this)(ui    , uj + 1) += kx2 * ky3 * value;
            (*this)(ui + 1, uj - 1) += kx3 * ky1 * value;
            (*this)(ui + 1, uj    ) += kx3 * ky2 * value;
            (*this)(ui + 1, uj + 1) += kx3 * ky3 * value;
        }
        else if (disp.x >= 0.5 && disp.y < 0.5) {
            (*this)(ui    , uj - 1) += kx2 * ky1 * value;
            (*this)(ui    , uj    ) += kx2 * ky2 * value;
            (*this)(ui    , uj + 1) += kx2 * ky3 * value;
            (*this)(ui + 1, uj - 1) += kx3 * ky1 * value;
            (*this)(ui + 1, uj    ) += kx3 * ky2 * value;
            (*this)(ui + 1, uj + 1) += kx3 * ky3 * value;
            (*this)(ui + 2, uj - 1) += kx1 * ky1 * value;
            (*this)(ui + 2, uj    ) += kx1 * ky2 * value;
            (*this)(ui + 2, uj + 1) += kx1 * ky3 * value;
        }
        else if (disp.x < 0.5 && disp.y >= 0.5) {
            (*this)(ui - 1, uj    ) += kx1 * ky2 * value;
            (*this)(ui - 1, uj + 1) += kx1 * ky3 * value;
            (*this)(ui - 1, uj + 2) += kx1 * ky1 * value;
            (*this)(ui    , uj    ) += kx2 * ky2 * value;
            (*this)(ui    , uj + 1) += kx2 * ky3 * value;
            (*this)(ui    , uj + 2) += kx2 * ky1 * value;
            (*this)(ui + 1, uj    ) += kx3 * ky2 * value;
            (*this)(ui + 1, uj + 1) += kx3 * ky3 * value;
            (*this)(ui + 1, uj + 2) += kx3 * ky1 * value;
        }
        else if (disp.x >= 0.5 && disp.y >= 0.5) {
            (*this)(ui    , uj    ) += kx2 * ky2 * value;
            (*this)(ui    , uj + 1) += kx2 * ky3 * value;
            (*this)(ui    , uj + 2) += kx2 * ky1 * value;
            (*this)(ui + 1, uj    ) += kx3 * ky2 * value;
            (*this)(ui + 1, uj + 1) += kx3 * ky3 * value;
            (*this)(ui + 1, uj + 2) += kx3 * ky1 * value;
            (*this)(ui + 2, uj    ) += kx1 * ky2 * value;
            (*this)(ui + 2, uj + 1) += kx1 * ky3 * value;
            (*this)(ui + 2, uj + 2) += kx1 * ky1 * value;
        }
    }

    double extract(mathfu::vec2d pos) {
        double value = 0.0;
        int ui = (int) (pos.x);
        int uj = (int) (pos.y);
        auto disp = mathfu::vec2d(pos.x - ui, pos.y - uj);
        double kx1 = 0.5 * (0.5 - disp.x) * (0.5 - disp.x);
        double kx2 = (0.75 - disp.x * disp.x);
        double kx3 = 0.5 * (0.5 + disp.x) * (0.5 + disp.x);
        double ky1 = 0.5 * (0.5 - disp.y) * (0.5 - disp.y);
        double ky2 = (0.75 - disp.y * disp.y);
        double ky3 = 0.5 * (0.5 + disp.y) * (0.5 + disp.y);
        if (disp.x < 0.5 && disp.y < 0.5) {
            value += kx1 * ky1 * (*this)(ui - 1, uj - 1);
            value += kx1 * ky2 * (*this)(ui - 1, uj    );
            value += kx1 * ky3 * (*this)(ui - 1, uj + 1);
            value += kx2 * ky1 * (*this)(ui    , uj - 1);
            value += kx2 * ky2 * (*this)(ui    , uj    );
            value += kx2 * ky3 * (*this)(ui    , uj + 1);
            value += kx3 * ky1 * (*this)(ui + 1, uj - 1);
            value += kx3 * ky2 * (*this)(ui + 1, uj    );
            value += kx3 * ky3 * (*this)(ui + 1, uj + 1);
        }
        else if (disp.x >= 0.5 && disp.y < 0.5) {
            value += kx2 * ky1 * (*this)(ui    , uj - 1);
            value += kx2 * ky2 * (*this)(ui    , uj    );
            value += kx2 * ky3 * (*this)(ui    , uj + 1);
            value += kx3 * ky1 * (*this)(ui + 1, uj - 1);
            value += kx3 * ky2 * (*this)(ui + 1, uj    );
            value += kx3 * ky3 * (*this)(ui + 1, uj + 1);
            value += kx1 * ky1 * (*this)(ui + 2, uj - 1);
            value += kx1 * ky2 * (*this)(ui + 2, uj    );
            value += kx1 * ky3 * (*this)(ui + 2, uj + 1);
        }
        else if (disp.x < 0.5 && disp.y >= 0.5) {
            value += kx1 * ky2 * (*this)(ui - 1, uj    );
            value += kx1 * ky3 * (*this)(ui - 1, uj + 1);
            value += kx1 * ky1 * (*this)(ui - 1, uj + 2);
            value += kx2 * ky2 * (*this)(ui    , uj    );
            value += kx2 * ky3 * (*this)(ui    , uj + 1);
            value += kx2 * ky1 * (*this)(ui    , uj + 2);
            value += kx3 * ky2 * (*this)(ui + 1, uj    );
            value += kx3 * ky3 * (*this)(ui + 1, uj + 1);
            value += kx3 * ky1 * (*this)(ui + 1, uj + 2);
        }
        else if (disp.x >= 0.5 && disp.y >= 0.5) {
            value += kx2 * ky2 * (*this)(ui    , uj    );
            value += kx2 * ky3 * (*this)(ui    , uj + 1);
            value += kx2 * ky1 * (*this)(ui    , uj + 2);
            value += kx3 * ky2 * (*this)(ui + 1, uj    );
            value += kx3 * ky3 * (*this)(ui + 1, uj + 1);
            value += kx3 * ky1 * (*this)(ui + 1, uj + 2);
            value += kx1 * ky2 * (*this)(ui + 2, uj    );
            value += kx1 * ky3 * (*this)(ui + 2, uj + 1);
            value += kx1 * ky1 * (*this)(ui + 2, uj + 2);
        }
        return value;
    }

    void extrapolate(Array2D<uint32_t, NX, NY>& intMask) {
        using mathfu::vec2i;
        auto Wu = std::queue<vec2i>();
        for (size_t j = 1; j < NY-1; j++) {
            for (size_t i = 1; i < NX-1; i++) {
                if (intMask(i, j) != 0 &&
                    (intMask(i-1, j) == 0 || intMask(i+1, j) == 0 || intMask(i, j-1) == 0 || intMask(i, j+1) == 0)) {
                    intMask(i, j) = 1;
                    Wu.push(vec2i(i, j));
                }
            }
        }
        while (Wu.size() > 0) {
            vec2i pos = Wu.front();
            Wu.pop();
            int x = pos.x, y = pos.y;
            double sum = 0.0;
            int count = 0;
            for (auto neighbor : {vec2i(x-1,y), vec2i(x+1,y), vec2i(x,y-1), vec2i(x,y+1)}) {
                if (neighbor.x < 0 || neighbor.x >= NX || neighbor.y < 0 || neighbor.y >= NY) continue;
                if (intMask(neighbor.x, neighbor.y) < intMask(x,y)) {
                    sum += (*this)(neighbor.x, neighbor.y);
                    count++;
                }
            }
            (*this)(x, y) = count == 0? 0 : sum / count;
            for (auto neighbor : {vec2i(x-1,y), vec2i(x+1,y), vec2i(x,y-1), vec2i(x,y+1)}) {
                if (neighbor.x < 0 || neighbor.x >= NX || neighbor.y < 0 || neighbor.y >= NY) continue;
                if (intMask(neighbor.x, neighbor.y) == UINT32_MAX) {
                    intMask(neighbor.x, neighbor.y) = intMask(x,y) + 1;
                    Wu.push(neighbor);
                }
            }
        }
    }
};

#endif //FLUID_SIM_ARRAY2D_H
