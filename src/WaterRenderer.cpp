//
// Created by lasagnaphil on 2018-09-17.
//

#include <imgui.h>
#include "WaterRenderer.h"
#include "FirstPersonCamera.h"
#include "InputManager.h"
#include "WaterSim3D.h"

void WaterRenderer::setup(WaterSim3D* sim, FirstPersonCamera* camera) {
    this->sim = sim;

    const auto EMPTY_COLOR = HMM_Vec4(0.0f, 0.0f, 0.0f, 1.0f);
    const auto FLUID_COLOR = HMM_Vec4(0.0f, 0.0f, 1.0f, 1.0f);
    const auto SOLID_COLOR = HMM_Vec4(0.1f, 0.1f, 0.1f, 1.0f);

    sim->iterate([&](size_t i, size_t j, size_t k) {
        WaterSim3D::CellType cellType = sim->cell(i, j, k);
        if (cellType == WaterSim3D::CellType::EMPTY) {
            vertexColors[2 * (i * SIZEY * SIZEZ + j * SIZEZ + k)] = EMPTY_COLOR;
            vertexColors[2 * (i * SIZEY * SIZEZ + j * SIZEZ + k) + 1] = EMPTY_COLOR;
        }
        else if (cellType == WaterSim3D::CellType::FLUID) {
            vertexColors[2 * (i * SIZEY * SIZEZ + j * SIZEZ + k)] = FLUID_COLOR;
            vertexColors[2 * (i * SIZEY * SIZEZ + j * SIZEZ + k) + 1] = FLUID_COLOR;
        }
        else if (cellType == WaterSim3D::CellType::SOLID) {
            vertexColors[2 * (i * SIZEY * SIZEZ + j * SIZEZ + k)] = SOLID_COLOR;
            vertexColors[2 * (i * SIZEY * SIZEZ + j * SIZEZ + k) + 1] = SOLID_COLOR;
        }
    });

    shader = Shader::create("assets/shaders/lines.vert", "assets/shaders/lines.frag");
    glGenVertexArrays(1, &lineVAO);
    glGenVertexArrays(1, &pointVAO);
    glGenBuffers(1, &lineVBO);
    glGenBuffers(1, &lineTypeVBO);

    glBindVertexArray(lineVAO);

    glBindBuffer(GL_ARRAY_BUFFER, lineVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(hmm_vec3) * LINE_VERTEX_COUNT, vertices, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(hmm_vec3), 0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, lineTypeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(hmm_vec4) * LINE_VERTEX_COUNT, vertexColors, GL_STATIC_DRAW);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(hmm_vec4), 0);
    glEnableVertexAttribArray(1);

    glBindVertexArray(pointVAO);

    glBindBuffer(GL_ARRAY_BUFFER, lineVBO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 2*sizeof(hmm_vec3), 0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, lineTypeVBO);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 2*sizeof(hmm_vec4), 0);
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);

    shader.use();
    shader.setMatrix4("model", HMM_Mat4d(1.0));
    camera->addShader(&shader);
}

void WaterRenderer::update() {
    auto inputMgr = InputManager::get();
    if (inputMgr->isKeyEntered(SDL_SCANCODE_1)) {
        drawMode = DrawMode::POINT;
    }
    else if (inputMgr->isKeyEntered(SDL_SCANCODE_2)) {
        drawMode = DrawMode::LINE;
    }

    sim->iterate([&](size_t i, size_t j, size_t k) {
        Eigen::Vector3d dir_d = sim->vel(i, j, k);
        hmm_vec3 dir = HMM_Vec3((float)dir_d(0), (float)dir_d(1), (float)dir_d(2));
        vertices[2 * (i * SIZEY * SIZEZ + j * SIZEZ + k)]
                = CELL_SIZE * HMM_Vec3(i, j, k);
        vertices[2 * (i * SIZEY * SIZEZ + j * SIZEZ + k) + 1]
                = CELL_SIZE * HMM_Vec3(i, j, k) + VEL_LINE_SCALE * dir;
    });

    glBindVertexArray(lineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, lineVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(hmm_vec3) * LINE_VERTEX_COUNT, vertices);
    glBindVertexArray(0);
}

void WaterRenderer::draw() {
    shader.use();

    if (drawMode == DrawMode::POINT) {
        glBindVertexArray(pointVAO);
        glDrawArrays(GL_POINTS, 0, POINT_VERTEX_COUNT);
    }
    else if (drawMode == DrawMode::LINE) {
        glBindVertexArray(lineVAO);
        glDrawArrays(GL_LINES, 0, LINE_VERTEX_COUNT);
    }
    glBindVertexArray(0);
}

void WaterRenderer::drawUI() {
    static size_t curr_i = 0;

    ImGui::Begin("Simulation Data");
    ImGui::SliderInt("layer(x)", (int *) &curr_i, 0, SIZEX - 1);

    if (ImGui::CollapsingHeader("Pressure")) {
        ImGui::Columns(SIZEZ, "table_p");
        ImGui::Separator();
        for (size_t j = 0; j < SIZEY; j++) {
            for (size_t k = 0; k < SIZEZ; k++) {
                char fstr[32];
                Eigen::Vector3d v = sim->vel(curr_i, j, k);
                sprintf(fstr, "%6f", sim->p(curr_i, j, k));
                ImGui::Text(fstr);
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
            for (size_t k = 0; k < SIZEZ; k++) {
                char fstr[32];
                Eigen::Vector3d v = sim->vel(curr_i, j, k);
                sprintf(fstr, "%2.2f %2.2f %2.2f", v(0), v(1), v(2));
                ImGui::Text(fstr);
                ImGui::NextColumn();
            }
            ImGui::Separator();
        }
        ImGui::Columns(1);
        ImGui::Separator();
    }
    ImGui::End();
}

