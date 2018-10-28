//
// Created by lasagnaphil on 9/9/18.
//

#ifndef ALTLIB_TRANSFORM_H
#define ALTLIB_TRANSFORM_H

#include <vec3.h>
#include <mat4f.h>
#include <quat.h>
#include <vec_utils.h>
#include <mat_utils.h>

struct Transform {
    vec3f pos;
    quatf rot;
    vec3f scale;

    static Transform createDefault() {
        Transform trans;
        trans.pos = vec3f {0.0f, 5.0f, 5.0f};
        trans.rot = quatf {0.0f, 0.0f, 0.0f, 1.0f};
        trans.scale = vec3f {1.0f, 1.0f, 1.0f};
        return trans;
    }

    mat4f toMat4() {
        mat4f mat = aml::scaleMat(scale);
        mat = rot.toMatrix4() * mat;
        mat = aml::transMat(pos) * mat;

        return mat;
    }

    vec3f frontVec() const {
        auto front = rot.toMatrix4() * vec4f{0.0f, 0.0f, 1.0f, 0.0f};
        return aml::normalize(aml::xyz(front));
    }

    vec3f upVec() const {
        auto up = rot.toMatrix4() * vec4f{0.0f, 1.0f, 0.0f, 0.0f};
        return aml::normalize(aml::xyz(up));
    }

    vec3f rightVec() const {
        auto right = rot.toMatrix4() * vec4f{1.0f, 0.0f, 0.0f, 0.0f};
        return aml::normalize(aml::xyz(right));
    }
};

#endif //ALTLIB_TRANSFORM_H
