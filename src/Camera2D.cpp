//
// Created by lasagnaphil on 10/2/18.
//

#include "mat_utils.h"
#include "Camera2D.h"
#include "imgui.h"
#include "Shader.h"
#include "App.h"

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
    mat4f mat = aml::transMat(vec3f {-pos.x, -pos.y, 0});
    mat = mat * aml::scaleMat(vec3f {zoom, zoom, 1});
    return mat;
}

mat4f Camera2D::getProjMatrix() const {
    float screenWidth = (float)settings->screenSize.x / pixelsPerMeter;
    float screenHeight = (float)settings->screenSize.y / pixelsPerMeter;
    return aml::ortho(-screenWidth/2, screenWidth/2,
                      -screenHeight/2, screenHeight/2,
                      -1.0f, 1.0f);
}

void Camera2D::drawUI() {
    ImGui::Begin("Camera");
    ImGui::Text("Position: %f %f", pos.x, pos.y);
    ImGui::Text("Zoom: %f", zoom);
    ImGui::End();
}
