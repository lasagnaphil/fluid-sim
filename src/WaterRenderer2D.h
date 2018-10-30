//
// Created by lasagnaphil on 10/2/18.
//
#ifndef FLUID_SIM_WATERRENDERER2D_H
#define FLUID_SIM_WATERRENDERER2D_H

#include <glad/glad.h>
#include <StackVec.h>
#include <imgui.h>

#include "WaterSimSettings.h"
#include "Shader.h"
#include "WaterSim2D.h"
#include "FirstPersonCamera.h"
#include "InputManager.h"
#include "Camera2D.h"

using namespace mathfu;

static float origQuadVertices[12] = {
        0.0f, 0.0f,
        0.0f, 1.0f,
        1.0f, 1.0f,
        0.0f, 0.0f,
        1.0f, 1.0f,
        1.0f, 0.0f,
};

class WaterRenderer2D {
private:
    static constexpr int SIZEX = WaterSimSettings::Dim2D::SIZEX;
    static constexpr int SIZEY = WaterSimSettings::Dim2D::SIZEY;

    GLuint waterCellVAO;
    GLuint solidCellVAO;
    GLuint pressureCellVAO;
    GLuint phiCellVAO;
    GLuint particleVAO;
    GLuint particleVelVAO;

    GLuint quadVBO;
    GLuint waterCellOffsetVBO;
    GLuint solidCellOffsetVBO;
    GLuint pressureCellOffsetVBO;
    GLuint pressureCellValueVBO;
    GLuint allCellOffsetVBO;
    GLuint phiCellValueVBO;
    GLuint particleVBO;
    GLuint particleVelVBO;

    vec2fp quadVertices[6];
    StackVec<vec2fp, SIZEX*SIZEY> waterCellLocations = {};
    StackVec<vec2fp, SIZEX*SIZEY> solidCellLocations = {};
    StackVec<vec2fp, SIZEX*SIZEY> pressureCellLocations = {};
    StackVec<float, SIZEX*SIZEY> pressureCellValues = {};
    StackVec<vec2fp, SIZEX*SIZEY> allCellLocations = {};
    StackVec<float, SIZEX*SIZEY> phiCellValues = {};

    StackVec<vec2fp, 8*SIZEX*SIZEY> particleVelLines = {};

    Shader cellShader;
    Shader particleShader;
    Shader cellFieldShader;

    WaterSim2D* sim;
    Camera2D* camera;

    bool renderParticles = true;
    bool renderParticleVels = true;
    bool renderCells = true;
    bool renderPressures = true;
    bool renderLevelSet = false;

    static const char* particleVS;
    static const char* cellVS;
    static const char* cellFieldVS;
    static const char* cellFS;

public:
    static constexpr float VEL_SIZE = 0.01f;

    void setup(WaterSim2D* sim, Camera2D* camera);

    void update();

    void draw();

    void drawUI();

    void updateBuffers();

    // void createWaterMesh();
};

#endif //FLUID_SIM_WATERRENDERER2D_H
