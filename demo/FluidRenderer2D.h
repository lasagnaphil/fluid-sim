//
// Created by lasagnaphil on 10/2/18.
//
#ifndef FLUID_SIM_FLUIDRENDERER2D_H
#define FLUID_SIM_FLUIDRENDERER2D_H

#include <glad/glad.h>
#include <StackVec.h>
#include <imgui.h>

#include "Shader.h"
#include "FluidSim2D.h"
#include "FirstPersonCamera.h"
#include "InputManager.h"
#include "Camera2D.h"

static float origQuadVertices[12] = {
        0.0f, 0.0f,
        0.0f, 1.0f,
        1.0f, 1.0f,
        0.0f, 0.0f,
        1.0f, 1.0f,
        1.0f, 0.0f,
};

struct FluidRenderer2D {
    int sizeX;
    int sizeY;
    int particlesPerCellSqrt;
    int particlesPerCell;

    GLuint waterCellVAO;
    GLuint solidCellVAO;
    GLuint cellVelVAO;
    GLuint pressureCellVAO;
    GLuint phiCellVAO;
    GLuint particleVAO;
    GLuint particleVelVAO;

    GLuint quadVBO;
    GLuint waterCellOffsetVBO;
    GLuint solidCellOffsetVBO;
    GLuint cellVelVBO;
    GLuint pressureCellOffsetVBO;
    GLuint pressureCellValueVBO;
    GLuint allCellOffsetVBO;
    GLuint phiCellValueVBO;
    GLuint particleVBO;
    GLuint particleVelVBO;

    vec2f quadVertices[6];
    Vec<vec2f> waterCellLocations = {};
    Vec<vec2f> solidCellLocations = {};
    Vec<vec2f> pressureCellLocations = {};
    Vec<vec2f> cellVels = {};
    Vec<float> pressureCellValues = {};
    Vec<vec2f> allCellLocations = {};
    Vec<float> phiCellValues = {};

    Vec<vec2f> particleVelLines = {};

    Shader cellShader;
    Shader particleShader;
    Shader cellFieldShader;

    FluidSim2D* sim;
    Camera2D* camera;

    bool renderParticles = true;
    bool renderParticleVels = true;
    bool renderCells = true;
    bool renderCellVels = false;
    bool renderPressures = false;
    bool renderLevelSet = false;

    static const char* particleVS;
    static const char* cellVS;
    static const char* cellFieldVS;
    static const char* cellFS;

    static FluidRenderer2D create(FluidSim2D* sim, Camera2D* camera);

    void free();

    void setup();

    void update();

    void draw();

    void drawUI();

    void updateBuffers();
};

#endif //FLUID_SIM_FLUIDRENDERER2D_H
