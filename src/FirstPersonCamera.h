//
// Created by lasagnaphil on 2/8/18.
//

#ifndef GENGINE_FIRSTPERSONCAMERA_H
#define GENGINE_FIRSTPERSONCAMERA_H

#include <StackVec.h>
#include "Transform.h"

struct AppSettings;
struct Shader;

enum CameraMovement {
    Forward,
    Backward,
    Left,
    Right
};

struct IntRect {
    int x;
    int y;
    int width;
    int height;
};

class FirstPersonCamera {
public:
    struct Settings {
        static constexpr float Yaw = 0.0f;
        static constexpr float Pitch = 0.0f;
        static constexpr float Speed = 1.0f;
        static constexpr float Sensitivity = 0.02f;
        static constexpr float Zoom = 45.0f;
    };

    struct Config {
        float pitch;
        float yaw;
        float movementSpeed;
        float mouseSensitivity;
        float zoom;
        IntRect viewport;
        bool constrainPitch;
        bool mouseMovementEnabled;
        Transform transform;
    };

    static FirstPersonCamera create(AppSettings* settings,
            float yaw = Settings::Yaw, float pitch = Settings::Pitch);

    void addShader(Shader* shader);

    void update(float dt);

    Config exportConfig();

    void importConfig(Config &data);

    mathfu::mat4f getViewMatrix() const;

    mathfu::mat4f getProjMatrix() const;

    void drawUI();

    AppSettings* settings;
    StackVec<Shader*, 4> shaders = {};

    float pitch;
    float yaw;
    float movementSpeed;
    float mouseSensitivity;
    float zoom;
    IntRect viewport;
    bool constrainPitch;

    bool mouseMovementEnabled = true;

    Transform transform;

private:

    void updateCameraVectors();
};


#endif //GENGINE_CAMERA_H
