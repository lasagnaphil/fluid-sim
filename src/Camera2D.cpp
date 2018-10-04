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
    /*
    Matrix4f viewMat = Matrix4f::scale(Vector3f::create(zoom, zoom, 1));
    viewMat = Matrix4f::trans(Vector3f::create(info->screenSize.x/zoom/2, info->screenSize.y/zoom/2, 0)) * viewMat;
    viewMat = Matrix4f::rotateZ(rotation) * viewMat;
    viewMat = Matrix4f::trans(Vector3f::create(-pos.x, -pos.y, 0)) * viewMat;
     */

    float screenWidth = (float)settings->screenSize.x / pixelsPerMeter;
    float screenHeight = (float)settings->screenSize.y / pixelsPerMeter;
    hmm_mat4 mat = HMM_Scale(HMM_Vec3(zoom, zoom, 1));
    mat = HMM_Translate(HMM_Vec3(screenWidth/zoom/2, screenHeight/zoom/2, 0)) * mat;
    mat = HMM_Translate(HMM_Vec3(-pos.X, -pos.Y, 0)) * mat;
    return mat;
}

hmm_mat4 Camera2D::getProjMatrix() const {
    float screenWidth = (float)settings->screenSize.x / pixelsPerMeter;
    float screenHeight = (float)settings->screenSize.y / pixelsPerMeter;
    return HMM_Orthographic(0.0f, screenWidth,
                            0.0f, screenHeight,
                            -1.0f, 1.0f);
}

void Camera2D::drawUI() {

}
