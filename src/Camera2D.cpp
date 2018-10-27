//
// Created by lasagnaphil on 10/2/18.
//

#include "Camera2D.h"
#include "Shader.h"
#include "App.h"

using namespace mathfu;

Camera2D Camera2D::create(AppSettings* settings, vec2f pos) {
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
        pos.y += speed / zoom * dt;
    }
    else if (inputMgr->isKeyPressed(SDL_SCANCODE_S)) {
        pos.y -= speed / zoom * dt;
    }
    if (inputMgr->isKeyPressed(SDL_SCANCODE_A)) {
        pos.x -= speed / zoom * dt;
    }
    else if (inputMgr->isKeyPressed(SDL_SCANCODE_D)) {
        pos.x += speed / zoom * dt;
    }
    if (inputMgr->isKeyEntered(SDL_SCANCODE_EQUALS)) {
        zoom *= 2;
    }
    else if (inputMgr->isKeyEntered(SDL_SCANCODE_MINUS)) {
        zoom /= 2;
    }

    FOREACH(shaders, shader, {
        shader->use();
        shader->setMatrix4("proj", getProjMatrix());
        shader->setMatrix4("view", getViewMatrix());
    })
}

mat4f Camera2D::getViewMatrix() const {
    return mat4f::Transform(vec3f(-pos.x, -pos.y, 0), mat3f::Identity(), vec3f(zoom, zoom, 1));
    /*
    mat4 mat = HMM_Translate(HMM_Vec3(-pos.x, -pos.y, 0));
    mat = HMM_Scale(HMM_Vec3(zoom, zoom, 1)) * mat;
    return mat;
     */
}

mat4f Camera2D::getProjMatrix() const {
    float screenWidth = (float)settings->screenSize.x / pixelsPerMeter;
    float screenHeight = (float)settings->screenSize.y / pixelsPerMeter;
    return mat4f::Ortho(-screenWidth/2, screenWidth/2,
                            -screenHeight/2, screenHeight/2,
                            -1.0f, 1.0f);
}

void Camera2D::drawUI() {
    ImGui::Begin("Camera");
    ImGui::Text("Position: %f %f", pos.x, pos.y);
    ImGui::Text("Zoom: %f", zoom);
    ImGui::End();
}
