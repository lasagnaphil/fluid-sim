//
// Created by lasagnaphil on 2/8/18.
//

#include <imgui.h>
#include "App.h"

#include "FirstPersonCamera.h"
#include "InputManager.h"
#include "Shader.h"

FirstPersonCamera FirstPersonCamera::create(AppSettings* settings, float yaw, float pitch)
{
    FirstPersonCamera cam;
    cam.yaw = yaw;
    cam.pitch = pitch;
    cam.movementSpeed = Settings::Speed;
    cam.mouseSensitivity = Settings::Sensitivity;
    cam.zoom = Settings::Zoom;
    cam.constrainPitch = true;
    cam.viewport.x = 0;
    cam.viewport.y = 0;
    cam.viewport.width = settings->screenSize.x;
    cam.viewport.height = settings->screenSize.y;
    cam.settings = settings;
    cam.transform = Transform::createDefault();
    cam.shaders = {};
    cam.mouseMovementEnabled = true;
    cam.updateCameraVectors();

    return cam;
}

void FirstPersonCamera::addShader(Shader *shader) {
    shaders.push(shader);
}

void FirstPersonCamera::update(float dt) {

    //
    // Update based on mouse / keyboard input
    //
    auto inputMgr = InputManager::get();

    if (inputMgr->isKeyEntered(SDL_SCANCODE_M)) {
        mouseMovementEnabled = !mouseMovementEnabled;
        SDL_SetRelativeMouseMode(mouseMovementEnabled? SDL_TRUE : SDL_FALSE);
    }

    // Mouse movement
    if (mouseMovementEnabled) {
        auto mouseOffsetI = inputMgr->getRelMousePos();
        hmm_vec2 mouseOffset = HMM_Vec2(mouseOffsetI.x, mouseOffsetI.y);

        mouseOffset *= mouseSensitivity;

        yaw += mouseOffset.X;
        pitch += mouseOffset.Y;

        if (constrainPitch) {
            if (pitch > 89.0f) pitch = 89.0f;
            if (pitch < -89.0f) pitch = -89.0f;
        }
    }

    updateCameraVectors();

    // Keyboard movement
    float velocity = movementSpeed * dt;
    if (inputMgr->isKeyPressed(SDL_SCANCODE_W)) {
        transform.pos -= transform.frontVec() * velocity;
    }
    else if (inputMgr->isKeyPressed(SDL_SCANCODE_S)) {
        transform.pos += transform.frontVec() * velocity;
    }
    if (inputMgr->isKeyPressed(SDL_SCANCODE_A)) {
        transform.pos -= transform.rightVec() * velocity;
    }
    else if (inputMgr->isKeyPressed(SDL_SCANCODE_D)) {
        transform.pos += transform.rightVec() * velocity;
    }
    if (inputMgr->isKeyPressed(SDL_SCANCODE_Q)) {
        transform.pos += transform.upVec() * velocity;
    }
    else if (inputMgr->isKeyPressed(SDL_SCANCODE_E)) {
        transform.pos -= transform.upVec() * velocity;
    }

    FOREACH(shaders, shader, {
        shader->use();
        shader->setVector3("viewPos", transform.pos);
        shader->setMatrix4("proj", getProjMatrix());
        shader->setMatrix4("view", getViewMatrix());
    })

    // Mouse scroll movement
    /*
    if (mouseMovementEnabled) {
        int yoffset = inputMgr->relWheelPos().y;
        if (zoom >= 1.0f && zoom <= 45.0f) {
            zoom -= yoffset;
        }
        if (zoom <= 1.0f) {
            zoom = 1.0f;
        }
        if (zoom >= 45.0f) {
            zoom = 45.0f;
        }
    }
     */
}

void FirstPersonCamera::updateCameraVectors() {
    hmm_quaternion quatX = HMM_QuaternionFromAxisAngle(HMM_Vec3(0.0f, 1.0f, 0.0f), -HMM_ToRadians(yaw));
    hmm_quaternion quatY = HMM_QuaternionFromAxisAngle(HMM_Vec3(1.0f, 0.0f, 0.0f), -HMM_ToRadians(pitch));
    transform.rot = HMM_NormalizeQuaternion(quatX * quatY);
}

FirstPersonCamera::Config FirstPersonCamera::exportConfig() {
    return Config {
            pitch, yaw, movementSpeed, mouseSensitivity, zoom,
            viewport, constrainPitch, mouseMovementEnabled, transform
    };
}

void FirstPersonCamera::importConfig(FirstPersonCamera::Config &data) {
    pitch = data.pitch;
    yaw = data.yaw;
    movementSpeed = data.movementSpeed;
    mouseSensitivity = data.mouseSensitivity;
    zoom = data.zoom;
    viewport = data.viewport;
    constrainPitch = data.constrainPitch;
    mouseMovementEnabled = data.mouseMovementEnabled;
    transform = data.transform;
}

hmm_mat4 FirstPersonCamera::getViewMatrix() const {
    hmm_mat4 view = HMM_LookAt(transform.pos, transform.pos - transform.frontVec(), transform.upVec());
    return view;
}

hmm_mat4 FirstPersonCamera::getProjMatrix() const {
    hmm_mat4 proj = HMM_Perspective(zoom, (float)settings->screenSize.x / (float)settings->screenSize.y, 0.01f, 1000.f);
    return proj;
}

void FirstPersonCamera::drawUI() {
    ImGui::Begin("Camera Info");
    ImGui::Text("pos: %f %f %f", transform.pos.X, transform.pos.Y, transform.pos.Z);
    ImGui::Text("rot: %f %f %f %f", transform.rot.X, transform.rot.Y, transform.rot.Z, transform.rot.W);
    ImGui::Text("scale: %f %f %f", transform.scale.X, transform.scale.Y, transform.scale.Z);
    ImGui::Text("pitch: %f, yaw: %f", pitch, yaw);
    ImGui::End();
}
