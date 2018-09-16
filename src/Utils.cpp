//
// Created by lasagnaphil on 9/16/18.
//

#include "Utils.h"

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
