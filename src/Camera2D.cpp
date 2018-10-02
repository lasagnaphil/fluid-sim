//
// Created by lasagnaphil on 10/2/18.
//

#include "Camera2D.h"
#include "Shader.h"
#include "App.h"

Camera2D Camera2D::create(AppSettings* settings, hmm_vec2 pos) {
    Camera2D cam;
    cam.settings = settings;
    cam.pos = pos;
    return cam;
}

void Camera2D::addShader(Shader* shader) {
    shaders.push(shader);
}

void Camera2D::update(float dt) {
    auto inputMgr = InputManager::get();
    if (inputMgr->isKeyPressed(SDL_SCANCODE_W)) {
        pos.Y += speed * dt;
    }
    else if (inputMgr->isKeyPressed(SDL_SCANCODE_S)) {
        pos.Y -= speed * dt;
    }
    if (inputMgr->isKeyPressed(SDL_SCANCODE_A)) {
        pos.X -= speed * dt;
    }
    else if (inputMgr->isKeyPressed(SDL_SCANCODE_D)) {
        pos.X += speed * dt;
    }

    FOREACH(shaders, shader, {
        shader->use();
        shader->setMatrix4("proj", getProjMatrix());
        shader->setMatrix4("view", getViewMatrix());
    })
}

hmm_mat4 Camera2D::getViewMatrix() const {
    return HMM_LookAt(HMM_Vec3(pos.X, pos.Y, 1.0), HMM_Vec3(pos.X, pos.Y, 0.0), HMM_Vec3(0.0, 0.0, 1.0));
}

hmm_mat4 Camera2D::getProjMatrix() const {
    float screenRatio = (float)settings->screenSize.x / settings->screenSize.y;
    return HMM_Orthographic(0.5f * zoom * screenRatio, 0.5f * zoom * screenRatio,
                            -0.5f * zoom * screenRatio, 0.5f * zoom * screenRatio,
                            0.01f, 1000.f);
}

void Camera2D::drawUI() {

}
