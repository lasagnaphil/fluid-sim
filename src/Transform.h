//
// Created by lasagnaphil on 9/9/18.
//

#ifndef ALTLIB_TRANSFORM_H
#define ALTLIB_TRANSFORM_H

#include <vec4f.h>
#include <quatf.h>
#include <mat4f.h>
#include <mat_utils.h>

struct Transform {
    vec4f pos;
    quatf rot;
    vec4f scale;

    static Transform createDefault() {
        Transform trans;
        trans.pos = {0.0f, 5.0f, 5.0f, 0.0f};
        trans.rot = {0.0f, 0.0f, 0.0f, 1.0f};
        trans.scale = {1.0f, 1.0f, 1.0f, 0.0f};
        return trans;
    }

    mat4f toMat4() {
        mat4f mat = aml::scaleMat(scale);
        mat = aml::toMatrix4(rot) * mat;
        mat = aml::transMat(pos) * mat;

        return mat;
    }

    vec4f frontVec() const {
        auto front = aml::toMatrix4(rot) * vec4f {0.0f, 0.0f, 1.0f, 0.0f};
        return aml::normalize(front);
    }

    vec4f upVec() const {
        auto up = aml::toMatrix4(rot) * vec4f {0.0f, 1.0f, 0.0f, 0.0f};
        return aml::normalize(up);
    }

    vec4f rightVec() const {
        auto right = aml::toMatrix4(rot) * vec4f {1.0f, 0.0f, 0.0f, 0.0f};
        return aml::normalize(right);
    }
};

#endif //ALTLIB_TRANSFORM_H
