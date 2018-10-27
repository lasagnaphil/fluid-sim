//
// Created by lasagnaphil on 9/9/18.
//

#ifndef FLUID_SIM_APP_H
#define FLUID_SIM_APP_H

#include <SDL.h>

#include "FirstPersonCamera.h"
#include "Camera2D.h"
#include "WaterRenderer2D.h"
#include "WaterRenderer3D.h"

struct AppSettings {
    mathfu::vec2i screenSize = {800, 600};
};

struct App {
    enum class Mode {
        Dim2, Dim3
    };
    static constexpr int MAX_SHADERS = 16;

    void init(mathfu::vec2i screenSize, Mode mode);
    void free();
    void start();

    SDL_Window* window;
    SDL_GLContext glContext;

    bool quit = false;
    AppSettings settings;
    Mode mode;

    FirstPersonCamera fpsCamera;
    Camera2D camera2d;

    WaterSim2D* waterSim2D;
    WaterRenderer2D* waterRenderer2D;
    WaterSim3D* waterSim3D;
    WaterRenderer3D* waterRenderer3D;
};

#endif //FLUID_SIM_APP_H
