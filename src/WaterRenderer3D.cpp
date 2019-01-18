//
// Created by lasagnaphil on 2018-09-17.
//

#include "FluidSim3D.h"
#include "WaterRenderer3D.h"

#include <imgui.h>
#include "FirstPersonCamera.h"
#include "InputManager.h"

static float origCubeVertices[3*36] = {
    -0.5f, -0.5f, -0.5f,
    0.5f, -0.5f, -0.5f,
    0.5f,  0.5f, -0.5f,
    0.5f,  0.5f, -0.5f,
    -0.5f,  0.5f, -0.5f,
    -0.5f, -0.5f, -0.5f,

    -0.5f, -0.5f,  0.5f,
    0.5f, -0.5f,  0.5f,
    0.5f,  0.5f,  0.5f,
    0.5f,  0.5f,  0.5f,
    -0.5f,  0.5f,  0.5f,
    -0.5f, -0.5f,  0.5f,

    -0.5f,  0.5f,  0.5f,
    -0.5f,  0.5f, -0.5f,
    -0.5f, -0.5f, -0.5f,
    -0.5f, -0.5f, -0.5f,
    -0.5f, -0.5f,  0.5f,
    -0.5f,  0.5f,  0.5f,

    0.5f,  0.5f,  0.5f,
    0.5f,  0.5f, -0.5f,
    0.5f, -0.5f, -0.5f,
    0.5f, -0.5f, -0.5f,
    0.5f, -0.5f,  0.5f,
    0.5f,  0.5f,  0.5f,

    -0.5f, -0.5f, -0.5f,
    0.5f, -0.5f, -0.5f,
    0.5f, -0.5f,  0.5f,
    0.5f, -0.5f,  0.5f,
    -0.5f, -0.5f,  0.5f,
    -0.5f, -0.5f, -0.5f,

    -0.5f,  0.5f, -0.5f,
    0.5f,  0.5f, -0.5f,
    0.5f,  0.5f,  0.5f,
    0.5f,  0.5f,  0.5f,
    -0.5f,  0.5f,  0.5f,
    -0.5f,  0.5f, -0.5f,
};

constexpr float WaterRenderer3D::CELL_SIZE;
constexpr float WaterRenderer3D::VEL_LINE_SCALE;
constexpr size_t WaterRenderer3D::POINT_VERTEX_COUNT;
constexpr size_t WaterRenderer3D::LINE_VERTEX_COUNT;

void WaterRenderer3D::setup(FluidSim3D* sim, FirstPersonCamera* camera) {
    this->sim = sim;

    const auto EMPTY_COLOR = vec4f {0.0f, 0.0f, 0.0f, 1.0f};
    const auto FLUID_COLOR = vec4f {0.0f, 0.0f, 1.0f, 1.0f};
    const auto SOLID_COLOR = vec4f {0.1f, 0.1f, 0.1f, 1.0f};

    lineShader = Shader::create("assets/shaders/lines.vert", "assets/shaders/lines.frag");
    voxelShader = Shader::create("assets/shaders/voxels.vert", "assets/shaders/lines.frag");

    memcpy(cubeVertices, origCubeVertices, sizeof(origCubeVertices));
    for (int i = 0; i < 3*36; i++) {
        cubeVertices[i] *= 0.8f * CELL_SIZE;
    }

    sim->mac.iterate([&](size_t i, size_t j, size_t k) {
        FluidSim3D::CellType cellType = sim->cell(i, j, k);
        if (cellType == FluidSim3D::CellType::EMPTY) {
            vertexColors[2 * (k * SIZEY * SIZEX + j * SIZEX + i)] = EMPTY_COLOR;
            vertexColors[2 * (k * SIZEY * SIZEX + j * SIZEX + i) + 1] = EMPTY_COLOR;
        }
        else if (cellType == FluidSim3D::CellType::FLUID) {
            vertexColors[2 * (k * SIZEY * SIZEX + j * SIZEX + i)] = FLUID_COLOR;
            vertexColors[2 * (k * SIZEY * SIZEX + j * SIZEX + i) + 1] = FLUID_COLOR;
        }
        else if (cellType == FluidSim3D::CellType::SOLID) {
            vertexColors[2 * (k * SIZEY * SIZEX + j * SIZEX + i)] = SOLID_COLOR;
            vertexColors[2 * (k * SIZEY * SIZEX + j * SIZEX + i) + 1] = SOLID_COLOR;
        }
    });

    updateWaterVoxelLocations();

    glGenVertexArrays(1, &lineVAO);
    glGenVertexArrays(1, &pointVAO);
    glGenVertexArrays(1, &voxelVAO);
    glGenBuffers(1, &lineVBO);
    glGenBuffers(1, &lineTypeVBO);
    glGenBuffers(1, &voxelVertexVBO);
    glGenBuffers(1, &cellOffsetVBO);

    glBindVertexArray(lineVAO);

    glBindBuffer(GL_ARRAY_BUFFER, lineVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vec3f) * LINE_VERTEX_COUNT, vertices, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vec3f), 0);

    glBindBuffer(GL_ARRAY_BUFFER, lineTypeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vec4f) * LINE_VERTEX_COUNT, vertexColors, GL_STATIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(vec4f), 0);

    glBindVertexArray(pointVAO);

    glBindBuffer(GL_ARRAY_BUFFER, lineVBO);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 2*sizeof(vec3f), 0);

    glBindBuffer(GL_ARRAY_BUFFER, lineTypeVBO);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 2*sizeof(vec4f), 0);

    glBindVertexArray(voxelVAO);

    glBindBuffer(GL_ARRAY_BUFFER, voxelVertexVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * 36, cubeVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, 0);

    glBindBuffer(GL_ARRAY_BUFFER, cellOffsetVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vec3f) * SIZEX*SIZEY*SIZEZ, waterVoxelLocations.data, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vec3d), 0);
    glVertexAttribDivisor(1, 1);

    glBindVertexArray(0);

    lineShader.use();
    lineShader.setMatrix4("model", aml::matIdentity<float>());
    camera->addShader(&lineShader);
    camera->addShader(&voxelShader);
}

void WaterRenderer3D::update() {
    auto inputMgr = InputManager::get();
    if (inputMgr->isKeyEntered(SDL_SCANCODE_1)) {
        drawMode = DrawMode::POINT;
        sim->rendered = false;
    }
    else if (inputMgr->isKeyEntered(SDL_SCANCODE_2)) {
        drawMode = DrawMode::LINE;
        sim->rendered = false;
    }
    else if (inputMgr->isKeyEntered(SDL_SCANCODE_3)) {
        drawMode = DrawMode::VOXEL;
        sim->rendered = false;
    }

    if (!sim->rendered) {
        if (drawMode == DrawMode::POINT || drawMode == DrawMode::LINE) {
            sim->mac.iterate([&](size_t i, size_t j, size_t k) {
                vec3d dir_d = sim->mac.vel(i, j, k);
                vec3f dir = vec3f{(float)dir_d.x, (float)dir_d.y, (float)dir_d.z};
                vertices[2 * (k * SIZEY * SIZEX + j * SIZEX + i)]
                        = CELL_SIZE * vec3f{(float)i, (float)j, (float)k};
                vertices[2 * (k * SIZEY * SIZEX + j * SIZEX + i) + 1]
                        = CELL_SIZE * vec3f{(float)i, (float)j, (float)k} + VEL_LINE_SCALE * dir;
            });
            glBindVertexArray(lineVAO);
            glBindBuffer(GL_ARRAY_BUFFER, lineVBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vec3d) * LINE_VERTEX_COUNT, vertices);
            glBindVertexArray(0);
        }
        else if (drawMode == DrawMode::VOXEL) {
            updateWaterVoxelLocations();
            glBindVertexArray(voxelVAO);
            glBindBuffer(GL_ARRAY_BUFFER, cellOffsetVBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vec3d) * waterVoxelLocations.size, waterVoxelLocations.data);
            glBindVertexArray(0);
        }
        sim->rendered = true;
    }
}

void WaterRenderer3D::draw() {
    if (drawMode == DrawMode::POINT) {
        glBindVertexArray(pointVAO);
        lineShader.use();
        glDrawArrays(GL_POINTS, 0, POINT_VERTEX_COUNT);
    }
    else if (drawMode == DrawMode::LINE) {
        glBindVertexArray(lineVAO);
        lineShader.use();
        glDrawArrays(GL_LINES, 0, LINE_VERTEX_COUNT);
    }
    else if (drawMode == DrawMode::VOXEL) {
        glBindVertexArray(voxelVAO);
        voxelShader.use();
        glDrawArraysInstanced(GL_TRIANGLES, 0, 6*36, waterVoxelLocations.size);
    }
    glBindVertexArray(0);
}

void WaterRenderer3D::drawUI() {
    static size_t curr_k = 0;

    ImGui::Begin("Simulation Data");
    if (sim->stage == FluidSim3D::Stage::ADVECTION) {
        ImGui::Text("Advection done.");
    }
    else if (sim->stage == FluidSim3D::Stage::GRAVITY) {
        ImGui::Text("Gravity done.");
    }
    else if (sim->stage == FluidSim3D::Stage::PROJECTION) {
        ImGui::Text("Projection done.");
    }

    ImGui::SliderInt("layer(z)", (int *) &curr_k, 0, SIZEZ - 1);

    if (ImGui::CollapsingHeader("Pressure")) {
        ImGui::Columns(SIZEZ, "table_p");
        ImGui::Separator();
        for (size_t j = 0; j < SIZEY; j++) {
            for (size_t i = 0; i < SIZEX; i++) {
                char fstr[32];
                sprintf(fstr, "%6f", sim->p(i, SIZEY - 1 - j, curr_k));
                ImGui::Text("%s", fstr);
                ImGui::NextColumn();
            }
            ImGui::Separator();
        }
        ImGui::Columns(1);
        ImGui::Separator();
    }
    if (ImGui::CollapsingHeader("Velocity")) {
        ImGui::Columns(SIZEZ, "table_vel");
        ImGui::Separator();
        for (size_t j = 0; j < SIZEY; j++) {
            for (size_t i = 0; i < SIZEX; i++) {
                char fstr[32];
                vec3d v = sim->mac.vel(i, SIZEY - 1 - j, curr_k);
                sprintf(fstr, "%2.2f %2.2f %2.2f", v.x, v.y, v.z);
                ImGui::Text("%s", fstr);
                ImGui::NextColumn();
            }
            ImGui::Separator();
        }
        ImGui::Columns(1);
        ImGui::Separator();
    }
    ImGui::End();
}

void WaterRenderer3D::updateWaterVoxelLocations() {
    waterVoxelLocations.size = 0;
    sim->mac.iterate([&](size_t i, size_t j, size_t k) {
        FluidSim3D::CellType cellType = sim->cell(i, j, k);
        if (cellType == FluidSim3D::CellType::FLUID) {
            waterVoxelLocations.push(vec3f {i*CELL_SIZE, j*CELL_SIZE, k*CELL_SIZE});
        }
    });
}

