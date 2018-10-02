//
// Created by lasagnaphil on 10/2/18.
//

#ifndef FLUID_SIM_CAMERA2D_H
#define FLUID_SIM_CAMERA2D_H

#include <StackVec.h>
#include "InputManager.h"

struct AppSettings;
struct Shader;

struct Camera2D {
    static Camera2D create(AppSettings* settings, hmm_vec2 pos);

    void addShader(Shader* shader);

    void update(float dt);

    hmm_mat4 getViewMatrix() const;

    hmm_mat4 getProjMatrix() const;

    void drawUI();

    AppSettings* settings;
    StackVec<Shader*, 4> shaders = {};

    float speed = 1.0f;
    float zoom = 1.0f;

    hmm_vec2 pos;
};

#endif //FLUID_SIM_CAMERA2D_H
