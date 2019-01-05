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
        vec2f mouseOffset = aml::toFloat(inputMgr->getRelMousePos());

        mouseOffset *= mouseSensitivity;

        yaw += mouseOffset.x;
        pitch += mouseOffset.y;

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
        shader->setVector3("viewPos", aml::xyz(transform.pos));
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
    quatf quatX = aml::quatAngleAxis(-(float)M_PI / 180.f * yaw, vec3f {0.0f, 1.0f, 0.0f});
    quatf quatY = aml::quatAngleAxis(-(float)M_PI / 180.f * pitch, vec3f {1.0f, 0.0f, 0.0f});
    transform.rot = aml::normalize(quatX * quatY);
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

mat4f FirstPersonCamera::getViewMatrix() const {
    mat4f view = aml::lookAt(aml::xyz(transform.pos), aml::xyz(transform.pos - transform.frontVec()), aml::xyz(transform.upVec()));
    return view;
}

mat4f FirstPersonCamera::getProjMatrix() const {
    mat4f proj = aml::perspective(zoom, (float)settings->screenSize.x / (float)settings->screenSize.y, 0.01f, 1000.f);
    return proj;
}

void FirstPersonCamera::drawUI() {
    ImGui::Begin("Camera Info");
    ImGui::Text("pos: %f %f %f", transform.pos.x, transform.pos.y, transform.pos.z);
    ImGui::Text("rot: %f %f %f %f", transform.rot.w, transform.rot.x, transform.rot.y, transform.rot.z);
    ImGui::Text("scale: %f %f %f", transform.scale.x, transform.scale.y, transform.scale.z);
    ImGui::Text("pitch: %f, yaw: %f", pitch, yaw);
    ImGui::End();
}
