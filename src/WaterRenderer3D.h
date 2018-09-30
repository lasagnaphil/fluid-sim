//
// Created by lasagnaphil on 2018-09-17.
//

#ifndef FLUID_SIM_WATERRENDERER3D_H
#define FLUID_SIM_WATERRENDERER3D_H

#include <cstddef>
#include <glad/glad.h>
#include <StackVec.h>
#include "HandmadeMath.h"
#include "Shader.h"
#include "WaterSimSettings.h"

struct FirstPersonCamera;
struct WaterSim3D;

class WaterRenderer3D {
    static constexpr int SIZEX = WaterSimSettings::Dim3D::SIZEX;
    static constexpr int SIZEY = WaterSimSettings::Dim3D::SIZEY;
    static constexpr int SIZEZ = WaterSimSettings::Dim3D::SIZEZ;
    static constexpr int ENABLE_DEBUG_UI = WaterSimSettings::Dim3D::ENABLE_DEBUG_UI;

    enum class DrawMode {
        POINT, LINE, VOXEL
    };
    DrawMode drawMode = DrawMode::POINT;
    static constexpr float CELL_SIZE = 1.0f / SIZEY;
    static constexpr float VEL_LINE_SCALE = 1.0f / SIZEY;
    static constexpr size_t POINT_VERTEX_COUNT = SIZEX * SIZEY * SIZEZ;
    static constexpr size_t LINE_VERTEX_COUNT = SIZEX * SIZEY * SIZEZ * 2;

    GLuint lineVAO;
    GLuint pointVAO;
    GLuint lineVBO;
    GLuint lineTypeVBO;
    GLuint voxelVAO;
    GLuint voxelVertexVBO;
    GLuint cellOffsetVBO;

    hmm_vec3 vertices[LINE_VERTEX_COUNT];
    hmm_vec4 vertexColors[LINE_VERTEX_COUNT];
    float cubeVertices[3*36];
    StackVec<hmm_vec3, LINE_VERTEX_COUNT> waterVoxelLocations = {};

    Shader lineShader;
    Shader voxelShader;

    WaterSim3D* sim;

public:
    void setup(WaterSim3D* sim, FirstPersonCamera* camera);
    void update();
    void draw();
    void drawUI();

private:
    void updateWaterVoxelLocations();
};


#endif //FLUID_SIM_WATERRENDERER3D_H
