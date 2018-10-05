//
// Created by lasagnaphil on 10/2/18.
//
#ifndef FLUID_SIM_WATERRENDERER2D_H
#define FLUID_SIM_WATERRENDERER2D_H

#include <glad/glad.h>
#include <HandmadeMath.h>
#include <StackVec.h>
#include <imgui.h>

#include "WaterSimSettings.h"
#include "Shader.h"
#include "WaterSim2D.h"
#include "FirstPersonCamera.h"
#include "InputManager.h"
#include "Camera2D.h"

static hmm_vec2 origQuadVertices[6] = {
        HMM_Vec2(0.0f, 0.0f),
        HMM_Vec2(0.0f, 1.0f),
        HMM_Vec2(1.0f, 1.0f),
        HMM_Vec2(0.0f, 0.0f),
        HMM_Vec2(1.0f, 1.0f),
        HMM_Vec2(1.0f, 0.0f),
};

class WaterRenderer2D {
private:
    static constexpr int SIZEX = WaterSimSettings::Dim2D::SIZEX;
    static constexpr int SIZEY = WaterSimSettings::Dim2D::SIZEY;
    static constexpr int ENABLE_DEBUG_UI = WaterSimSettings::Dim2D::ENABLE_DEBUG_UI;

    static constexpr float CELL_SIZE = 1.0f / SIZEY;
    static constexpr float VEL_LINE_SCALE = 10.0f / SIZEY;

    GLuint waterCellVAO;
    GLuint solidCellVAO;
    GLuint particleVAO;

    GLuint quadVBO;
    GLuint waterCellOffsetVBO;
    GLuint solidCellOffsetVBO;
    GLuint particleVBO;

    hmm_vec2 cellVertices[SIZEX*SIZEY];
    hmm_vec2 quadVertices[6];
    StackVec<hmm_vec2, SIZEX*SIZEY> waterCellLocations = {};
    StackVec<hmm_vec2, SIZEX*SIZEY> solidCellLocations = {};
    StackVec<hmm_vec2, 4*SIZEX*SIZEY> particleLocations = {};

    Shader cellShader;
    Shader particleShader;

    WaterSim2D* sim;

    const char* particleVS = R"SHADER(
#version 330 core

layout (location = 0) in vec2 inPos;

out vec4 color;

uniform mat4 view;
uniform mat4 proj;

void main() {
    gl_Position = proj * view * vec4(inPos, 0.1, 1.0);
    color = vec4(1.0, 1.0, 1.0, 1.0);
}

)SHADER";

    const char* cellVS = R"SHADER(
#version 330 core

layout (location = 0) in vec2 inPos;
layout (location = 1) in vec2 inOffset;

out vec4 color;

uniform vec4 uniColor;
uniform mat4 view;
uniform mat4 proj;

void main() {
    gl_Position = proj * view * vec4(inPos + inOffset, 0.0, 1.0);
    color = uniColor;
}
)SHADER";

    const char* cellFS = R"SHADER(
#version 330 core

in vec4 color;

out vec4 fragColor;

void main() {
	fragColor = color;
}
)SHADER";


public:
    void setup(WaterSim2D* sim, Camera2D* camera) {
        this->sim = sim;

        const auto EMPTY_COLOR = HMM_Vec4(0.0f, 0.0f, 0.0f, 1.0f);
        const auto FLUID_COLOR = HMM_Vec4(0.0f, 0.0f, 1.0f, 1.0f);
        const auto SOLID_COLOR = HMM_Vec4(0.1f, 0.1f, 0.1f, 1.0f);

        cellShader = Shader::fromStr(cellVS, cellFS);
        particleShader = Shader::fromStr(particleVS, cellFS);

        memcpy(quadVertices, origQuadVertices, sizeof(origQuadVertices));
        for (int i = 0; i < 6*36; i++) {
            quadVertices[i] *= sim->dx;
        }

        updateLocations();

        glGenVertexArrays(1, &waterCellVAO);
        glGenVertexArrays(1, &solidCellVAO);
        glGenVertexArrays(1, &particleVAO);
        glGenBuffers(1, &quadVBO);
        glGenBuffers(1, &waterCellOffsetVBO);
        glGenBuffers(1, &solidCellOffsetVBO);
        glGenBuffers(1, &particleVBO);

        glBindVertexArray(waterCellVAO);

        glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(hmm_vec2) * 6, quadVertices, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(hmm_vec2), 0);

        glBindBuffer(GL_ARRAY_BUFFER, waterCellOffsetVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(hmm_vec2) * SIZEX*SIZEY, waterCellLocations.data, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(hmm_vec2), 0);
        glVertexAttribDivisor(1, 1);

        glBindVertexArray(solidCellVAO);

        glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(hmm_vec2), 0);

        glBindBuffer(GL_ARRAY_BUFFER, solidCellOffsetVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(hmm_vec2) * SIZEX*SIZEY, solidCellLocations.data, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(hmm_vec2), 0);
        glVertexAttribDivisor(1, 1);

        glBindVertexArray(particleVAO);

        glBindBuffer(GL_ARRAY_BUFFER, particleVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(hmm_vec2) * 4*SIZEX*SIZEY, particleLocations.data, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(hmm_vec2), 0);

        glBindVertexArray(0);

        camera->addShader(&cellShader);
        camera->addShader(&particleShader);
    }

    void update() {
        if (!sim->rendered) {
            updateLocations();
            glBindVertexArray(waterCellVAO);
            glBindBuffer(GL_ARRAY_BUFFER, waterCellOffsetVBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(hmm_vec2) * waterCellLocations.size, waterCellLocations.data);
            glBindVertexArray(solidCellVAO);
            glBindBuffer(GL_ARRAY_BUFFER, solidCellOffsetVBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(hmm_vec2) * solidCellLocations.size, solidCellLocations.data);
            glBindVertexArray(particleVAO);
            glBindBuffer(GL_ARRAY_BUFFER, particleVBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(hmm_vec2) * particleLocations.size, particleLocations.data);
            glBindVertexArray(0);
        }
    }

    void draw() {
        cellShader.use();
        cellShader.setVector4("uniColor", HMM_Vec4(0.0f, 0.0f, 1.0f, 1.0f));
        glBindVertexArray(waterCellVAO);
        glDrawArraysInstanced(GL_TRIANGLES, 0, 6, waterCellLocations.size);
        cellShader.setVector4("uniColor", HMM_Vec4(0.5f, 0.5f, 0.5f, 1.0f));
        glBindVertexArray(solidCellVAO);
        glDrawArraysInstanced(GL_TRIANGLES, 0, 6, solidCellLocations.size);
        particleShader.use();
        glBindVertexArray(particleVAO);
        glDrawArrays(GL_POINTS, 0, particleLocations.size);
        glBindVertexArray(0);
    }

    void drawUI() {
        if (ENABLE_DEBUG_UI) {
            ImGui::Begin("Simulation Info");
            ImGui::Text("Current frame: %f", sim->currentTime);
            ImGui::Text("Average pressure: %f", sim->avgPressure());
            ImGui::Text("Average pressure in fluid: %f", sim->avgPressureInFluid());
            /*
            if (ImGui::CollapsingHeader("Pressure")) {
                ImGui::Columns(SIZEY, "table_p");
                ImGui::Separator();
                for (size_t j = 0; j < SIZEY; j++) {
                    for (size_t i = 0; i < SIZEX; i++) {
                        char fstr[32];
                        sprintf(fstr, "%6f", sim->p(i, SIZEY - 1 - j));
                        ImGui::Text(fstr);
                        ImGui::NextColumn();
                    }
                    ImGui::Separator();
                }
                ImGui::Columns(1);
                ImGui::Separator();
            }
            if (ImGui::CollapsingHeader("Velocity")) {
                ImGui::Columns(SIZEY, "table_vel");
                ImGui::Separator();
                for (size_t j = 0; j < SIZEY; j++) {
                    for (size_t i = 0; i < SIZEX; i++) {
                        char fstr[32];
                        Vector2d v = sim->mac.vel(i, SIZEY - 1 - j);
                        sprintf(fstr, "%2.2f %2.2f %2.2f", v.x, v.y);
                        ImGui::Text(fstr);
                        ImGui::NextColumn();
                    }
                    ImGui::Separator();
                }
                ImGui::Columns(1);
                ImGui::Separator();
            }
             */
            ImGui::End();
        }
    }

    void updateLocations() {
        waterCellLocations.size = 0;
        solidCellLocations.size = 0;
        sim->mac.iterate([&](size_t i, size_t j) {
            WaterSim2D::CellType cellType = sim->cell(i,j);
            if (cellType == WaterSim2D::CellType::FLUID) {
                waterCellLocations.push(HMM_Vec2((float)i,(float)j) * sim->dx);
            }
            else if (cellType == WaterSim2D::CellType::SOLID) {
                solidCellLocations.push(HMM_Vec2((float)i,(float)j) * sim->dx);
            }
        });
        particleLocations.size = sim->particles.size;
        for (size_t i = 0; i < sim->particles.size; i++) {
            particleLocations[i].X = (float)sim->particles[i].x;
            particleLocations[i].Y = (float)sim->particles[i].y;
        }
    }
};

#endif //FLUID_SIM_WATERRENDERER2D_H
