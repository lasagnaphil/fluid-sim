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
    static Camera2D create(AppSettings* settings, mathfu::vec2f pos);

    void addShader(Shader* shader);

    void update(float dt);

    mathfu::mat4f getViewMatrix() const;

    mathfu::mat4f getProjMatrix() const;

    void drawUI();

    AppSettings* settings;
    StackVec<Shader*, 16> shaders = {};

    float speed = 5.0f;
    float pixelsPerMeter = 100.0f;
    float zoom = 1.0f;

    mathfu::vec2f pos;
};

#endif //FLUID_SIM_CAMERA2D_H
