//
// Created by lasagnaphil on 2018-09-17.
//

#ifndef FLUID_SIM_FLUIDRENDERER3D_H
#define FLUID_SIM_FLUIDRENDERER3D_H

#include <cstddef>
#include <glad/glad.h>
#include <StackVec.h>

#include "Shader.h"
#include "FluidSimSettings.h"

struct FirstPersonCamera;
struct FluidSim3D;

class WaterRenderer3D {
    static constexpr int SIZEX = FluidSimSettings::Dim3D::SIZEX;
    static constexpr int SIZEY = FluidSimSettings::Dim3D::SIZEY;
    static constexpr int SIZEZ = FluidSimSettings::Dim3D::SIZEZ;
    static constexpr int ENABLE_DEBUG_UI = FluidSimSettings::Dim3D::ENABLE_DEBUG_UI;

    enum class DrawMode {
        POINT, LINE, VOXEL
    };
    DrawMode drawMode = DrawMode::POINT;
    static constexpr float CELL_SIZE = 1.0f / SIZEY;
    static constexpr float VEL_LINE_SCALE = 0.001f / SIZEY;
    static constexpr size_t POINT_VERTEX_COUNT = SIZEX * SIZEY * SIZEZ;
    static constexpr size_t LINE_VERTEX_COUNT = SIZEX * SIZEY * SIZEZ * 2;

    GLuint lineVAO;
    GLuint pointVAO;
    GLuint lineVBO;
    GLuint lineTypeVBO;
    GLuint voxelVAO;
    GLuint voxelVertexVBO;
    GLuint cellOffsetVBO;

    vec3f vertices[LINE_VERTEX_COUNT];
    vec4f vertexColors[LINE_VERTEX_COUNT];
    float cubeVertices[3*36];
    StackVec<vec3f, LINE_VERTEX_COUNT> waterVoxelLocations = {};

    Shader lineShader;
    Shader voxelShader;

    FluidSim3D* sim;

public:
    void setup(FluidSim3D* sim, FirstPersonCamera* camera);
    void update();
    void draw();
    void drawUI();

private:
    void updateWaterVoxelLocations();
};


#endif //FLUID_SIM_FLUIDRENDERER3D_H
