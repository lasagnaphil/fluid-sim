//
// Created by lasagnaphil on 9/9/18.
//

#ifndef FLUID_SIM_APP_H
#define FLUID_SIM_APP_H

#include <SDL.h>

#include "vec2d.h"
#include "Camera2D.h"
#include "FluidSim2D.h"
#include "FluidRenderer2D.h"

struct AppSettings {
    vec2i screenSize = {800, 600};
};

struct App {
    enum class Mode {
        Dim2, Dim3
    };
    static constexpr int MAX_SHADERS = 16;

    void init(vec2i screenSize);
    void free();
    void start();
    void screenshot(const char* filename);

    SDL_Window* window;
    SDL_GLContext glContext;

    bool quit = false;
    AppSettings settings;

    Camera2D camera2d;

    FluidSim2D fluidSim2D;
    FluidRenderer2D fluidRenderer2D;
};

#endif //FLUID_SIM_APP_H
