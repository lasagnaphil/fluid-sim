//
// Created by lasagnaphil on 9/9/18.
//

#ifndef ALTLIB_TRANSFORM_H
#define ALTLIB_TRANSFORM_H

#include "HandmadeMath.h"

struct Transform {
    hmm_vec3 pos;
    hmm_quaternion rot;
    hmm_vec3 scale;

    static Transform createDefault() {
        Transform trans;
        trans.pos = HMM_Vec3(0.0f, 5.0f, 5.0f);
        trans.rot = HMM_Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
        trans.scale = HMM_Vec3(1.0f, 1.0f, 1.0f);
        return trans;
    }

    hmm_mat4 toMat4() {
        hmm_mat4 mat = HMM_Mat4();
        mat[0][0] = scale.X;
        mat[1][1] = scale.Y;
        mat[2][2] = scale.Z;
        mat = HMM_QuaternionToMat4(rot) * mat;
        mat = HMM_Translate(pos) * mat;

        return mat;
    }

    hmm_vec3 frontVec() const {
        hmm_vec4 front = HMM_QuaternionToMat4(rot) * HMM_Vec4(0.0f, 0.0f, 1.0f, 0.0f);
        return HMM_Normalize(front.XYZ);
    }

    hmm_vec3 upVec() const {
        hmm_vec4 up = HMM_QuaternionToMat4(rot) * HMM_Vec4(0.0f, 1.0f, 0.0f, 0.0f);
        return HMM_Normalize(up.XYZ);
    }

    hmm_vec3 rightVec() const {
        hmm_vec4 right = HMM_QuaternionToMat4(rot) * HMM_Vec4(1.0f, 0.0f, 0.0f, 0.0f);
        return HMM_Normalize(right.XYZ);
    }
};

#endif //ALTLIB_TRANSFORM_H
