//
// Created by lasagnaphil on 9/9/18.
//

#ifndef ALTLIB_TRANSFORM_H
#define ALTLIB_TRANSFORM_H

#include <mathfu/glsl_mappings.h>

struct Transform {
    mathfu::vec3 pos;
    mathfu::quat rot;
    mathfu::vec3 scale;

    static Transform createDefault() {
        Transform trans;
        trans.pos = {0.0f, 5.0f, 5.0f};
        trans.rot = {0.0f, 0.0f, 0.0f, 1.0f};
        trans.scale = {1.0f, 1.0f, 1.0f};
        return trans;
    }

    mathfu::mat4 toMat4() {
        mathfu::mat4 mat = mathfu::mat4::FromScaleVector(scale);
        mat = rot.ToMatrix4() * mat;
        mat = mathfu::mat4::FromTranslationVector(pos) * mat;

        return mat;
    }

    mathfu::vec3 frontVec() const {
        auto front = rot.ToMatrix4() * mathfu::vec4(0.0f, 0.0f, 1.0f, 0.0f);
        return front.xyz().Normalized();
    }

    mathfu::vec3 upVec() const {
        auto up = rot.ToMatrix4() * mathfu::vec4(0.0f, 1.0f, 0.0f, 0.0f);
        return up.xyz().Normalized();
    }

    mathfu::vec3 rightVec() const {
        auto right = rot.ToMatrix4() * mathfu::vec4(1.0f, 0.0f, 0.0f, 0.0f);
        return right.xyz().Normalized();
    }
};

#endif //ALTLIB_TRANSFORM_H
