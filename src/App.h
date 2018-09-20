//
// Created by lasagnaphil on 9/9/18.
//

#ifndef FLUID_SIM_APP_H
#define FLUID_SIM_APP_H

#include <math/Vector2.h>
#include <SDL.h>
#include "FirstPersonCamera.h"
#include "WaterSim3D.h"
#include "WaterRenderer.h"

struct AppSettings {
    Vector2i screenSize = {800, 600};
    bool isMouseRelative = true;
};

struct App {
    static constexpr int MAX_SHADERS = 16;

    void init(Vector2i screenSize);
    void free();
    void start();

    SDL_Window* window;
    SDL_GLContext glContext;

    bool quit = false;
    AppSettings settings;

    FirstPersonCamera camera;
    WaterSim3D waterSim;
    WaterRenderer waterRenderer;
};

#endif //FLUID_SIM_APP_H
