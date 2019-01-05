//
// Created by lasagnaphil on 9/9/18.
//

#ifndef FLUID_SIM_APP_H
#define FLUID_SIM_APP_H

#include <SDL.h>

#include "FirstPersonCamera.h"
#include "Camera2D.h"
#include "vec2d.h"

struct WaterSim2D;
struct WaterRenderer2D;
struct WaterSim3D;
struct WaterRenderer3D;

struct AppSettings {
    vec2i screenSize = {800, 600};
};

struct App {
    enum class Mode {
        Dim2, Dim3
    };
    static constexpr int MAX_SHADERS = 16;

    void init(vec2i screenSize, Mode mode);
    void free();
    void start();
    void screenshot(const char* filename);

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
