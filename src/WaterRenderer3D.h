//
// Created by lasagnaphil on 2018-09-17.
//

#ifndef FLUID_SIM_WATERRENDERER3D_H
#define FLUID_SIM_WATERRENDERER3D_H

#include <cstddef>
#include <glad/glad.h>
#include "HandmadeMath.h"
#include "Shader.h"
#include "WaterSimSettings.h"

struct FirstPersonCamera;
struct WaterSim3D;

class WaterRenderer3D {
    static constexpr int SIZEX = WaterSimSettings::Dim3D::SIZEX;
    static constexpr int SIZEY = WaterSimSettings::Dim3D::SIZEY;
    static constexpr int SIZEZ = WaterSimSettings::Dim3D::SIZEZ;

    enum class DrawMode {
        POINT, LINE
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
    hmm_vec3 vertices[LINE_VERTEX_COUNT];
    hmm_vec4 vertexColors[LINE_VERTEX_COUNT];
    Shader shader;

    WaterSim3D* sim;

public:
    void setup(WaterSim3D* sim, FirstPersonCamera* camera);
    void update();
    void draw();
    void drawUI();
};


#endif //FLUID_SIM_WATERRENDERER3D_H
