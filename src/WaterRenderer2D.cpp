//
// Created by lasagnaphil on 10/27/18.
//

#include "WaterRenderer2D.h"
#include "App.h"

const char* WaterRenderer2D::particleVS = R"SHADER(
#version 330 core

layout (location = 0) in vec2 inPos;

out vec4 color;

uniform mat4 view;
uniform mat4 proj;

void main() {
    gl_Position = proj * view * vec4(inPos, 0.2, 1.0);
    color = vec4(1.0, 1.0, 1.0, 1.0);
}

)SHADER";

const char* WaterRenderer2D::cellVS = R"SHADER(
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

const char* WaterRenderer2D::cellFieldVS = R"SHADER(
#version 330 core

layout (location = 0) in vec2 inPos;
layout (location = 1) in vec2 inOffset;
layout (location = 2) in float inValue;

out vec4 color;

uniform mat4 view;
uniform mat4 proj;
uniform vec4 posColor;
uniform vec4 negColor;

void main() {
    gl_Position = proj * view * vec4(inPos + inOffset, 0.1, 1.0);
    color = inValue * posColor + (1 - inValue) * negColor;
}
)SHADER";

const char* WaterRenderer2D::cellFS = R"SHADER(
#version 330 core

in vec4 color;

out vec4 fragColor;

void main() {
	fragColor = color;
}
)SHADER";

void WaterRenderer2D::setup(WaterSim2D *sim, Camera2D *camera) {
    this->sim = sim;
    this->camera = camera;

    const auto EMPTY_COLOR = vec4f(0.0f, 0.0f, 0.0f, 1.0f);
    const auto FLUID_COLOR = vec4f(0.0f, 0.0f, 1.0f, 1.0f);
    const auto SOLID_COLOR = vec4f(0.1f, 0.1f, 0.1f, 1.0f);

    cellShader = Shader::fromStr(cellVS, cellFS);
    particleShader = Shader::fromStr(particleVS, cellFS);
    cellFieldShader = Shader::fromStr(cellFieldVS, cellFS);

    memcpy(quadVertices, origQuadVertices, sizeof(origQuadVertices));
    for (int i = 0; i < 6*36; i++) {
        quadVertices[i].x *= sim->dx;
        quadVertices[i].y *= sim->dx;
    }

    allCellLocations.size = SIZEX*SIZEY;
    sim->iterate([&](size_t i, size_t j) {
        allCellLocations[j*SIZEX + i] = vec2f(i,j) * (float)sim->dx;
    });

    updateBuffers();

    glGenVertexArrays(1, &waterCellVAO);
    glGenVertexArrays(1, &solidCellVAO);
    glGenVertexArrays(1, &cellVelVAO);
    glGenVertexArrays(1, &pressureCellVAO);
    glGenVertexArrays(1, &phiCellVAO);
    glGenVertexArrays(1, &particleVAO);
    glGenVertexArrays(1, &particleVelVAO);
    glGenBuffers(1, &quadVBO);
    glGenBuffers(1, &waterCellOffsetVBO);
    glGenBuffers(1, &solidCellOffsetVBO);
    glGenBuffers(1, &cellVelVBO);
    glGenBuffers(1, &pressureCellOffsetVBO);
    glGenBuffers(1, &pressureCellValueVBO);
    glGenBuffers(1, &allCellOffsetVBO);
    glGenBuffers(1, &phiCellValueVBO);
    glGenBuffers(1, &particleVBO);
    glGenBuffers(1, &particleVelVBO);

    glBindVertexArray(waterCellVAO);

    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vec2fp) * 6, quadVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(vec2fp), 0);

    glBindBuffer(GL_ARRAY_BUFFER, waterCellOffsetVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vec2fp) * SIZEX*SIZEY, waterCellLocations.data, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(vec2fp), 0);
    glVertexAttribDivisor(1, 1);

    glBindVertexArray(solidCellVAO);

    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(vec2fp), 0);

    glBindBuffer(GL_ARRAY_BUFFER, solidCellOffsetVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vec2fp) * SIZEX*SIZEY, solidCellLocations.data, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(vec2fp), 0);
    glVertexAttribDivisor(1, 1);

    glBindVertexArray(cellVelVAO);

    glBindBuffer(GL_ARRAY_BUFFER, cellVelVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vec2fp) * 2*SIZEX*SIZEY, cellVels.data, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(vec2fp), 0);

    glBindVertexArray(pressureCellVAO);

    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(vec2fp), 0);

    glBindBuffer(GL_ARRAY_BUFFER, pressureCellOffsetVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vec2fp) * SIZEX*SIZEY, pressureCellLocations.data, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(vec2fp), 0);
    glVertexAttribDivisor(1, 1);

    glBindBuffer(GL_ARRAY_BUFFER, pressureCellValueVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * SIZEX*SIZEY, pressureCellValues.data, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(float), 0);
    glVertexAttribDivisor(2, 1);

    glBindVertexArray(phiCellVAO);

    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(vec2fp), 0);

    glBindBuffer(GL_ARRAY_BUFFER, allCellOffsetVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vec2fp) * SIZEX*SIZEY, allCellLocations.data, GL_STATIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(vec2fp), 0);
    glVertexAttribDivisor(1, 1);

    glBindBuffer(GL_ARRAY_BUFFER, phiCellValueVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * SIZEX*SIZEY, phiCellValues.data, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(float), 0);
    glVertexAttribDivisor(2, 1);

    glBindVertexArray(particleVAO);

    glBindBuffer(GL_ARRAY_BUFFER, particleVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vec2fp) * 2*8*SIZEX*SIZEY, particleVelLines.data, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(vec2fp), 0);

    glBindVertexArray(particleVelVAO);

    glBindBuffer(GL_ARRAY_BUFFER, particleVelVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vec2fp) * 2*8*SIZEX*SIZEY, particleVelLines.data, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(vec2fp), 0);

    glBindVertexArray(0);

    camera->addShader(&cellShader);
    camera->addShader(&particleShader);
    camera->addShader(&cellFieldShader);
}

void WaterRenderer2D::update() {
    if (!sim->rendered) {
        updateBuffers();
        if (renderCells) {
            glBindVertexArray(waterCellVAO);
            glBindBuffer(GL_ARRAY_BUFFER, waterCellOffsetVBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vec2fp) * waterCellLocations.size, waterCellLocations.data);
            glBindVertexArray(solidCellVAO);
            glBindBuffer(GL_ARRAY_BUFFER, solidCellOffsetVBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vec2fp) * solidCellLocations.size, solidCellLocations.data);
            glBindVertexArray(0);
        }
        if (renderCellVels) {
            glBindVertexArray(cellVelVAO);
            glBindBuffer(GL_ARRAY_BUFFER, cellVelVBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vec2fp) * cellVels.size, cellVels.data);
            glBindVertexArray(0);
        }
        if (renderPressures) {
            glBindVertexArray(pressureCellVAO);
            glBindBuffer(GL_ARRAY_BUFFER, pressureCellOffsetVBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vec2fp) * pressureCellLocations.size, pressureCellLocations.data);
            glBindVertexArray(particleVelVAO);
            glBindBuffer(GL_ARRAY_BUFFER, pressureCellValueVBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * pressureCellValues.size, pressureCellValues.data);
            glBindVertexArray(0);
        }
        if (renderParticles) {
            glBindVertexArray(particleVAO);
            glBindBuffer(GL_ARRAY_BUFFER, particleVBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vec2fp) * particleVelLines.size, particleVelLines.data);
            glBindVertexArray(0);
        }
        if (renderParticleVels) {
            glBindVertexArray(particleVelVAO);
            glBindBuffer(GL_ARRAY_BUFFER, particleVelVBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vec2fp) * particleVelLines.size, particleVelLines.data);
            glBindVertexArray(0);
        }
        if (renderLevelSet) {
            glBindVertexArray(phiCellVAO);
            glBindBuffer(GL_ARRAY_BUFFER, phiCellValueVBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * phiCellValues.size, phiCellValues.data);
            glBindVertexArray(0);
        }
        sim->rendered = true;
    }

    // Change velocity using mouse drag
    static vec2f startMousePos;
    vec2f currMousePos;
    constexpr float VEL_MOUSEDRAG_SCALE = 0.1f;
    auto inputMgr = InputManager::get();
    if (inputMgr->isMouseEntered(SDL_BUTTON_LEFT)) {
        startMousePos = vec2f(inputMgr->getMousePos());
    }
    else if (inputMgr->isMousePressed(SDL_BUTTON_LEFT)) {
        currMousePos = vec2f(inputMgr->getMousePos());
        vec2f velIncrease = VEL_MOUSEDRAG_SCALE * (currMousePos - startMousePos);

        vec2f screenSize = vec2f(camera->settings->screenSize);
        vec2f normMousePos = startMousePos - (screenSize / 2.f);
        normMousePos.y *= -1;
        vec2f gridPos = (normMousePos / camera->pixelsPerMeter / camera->zoom + camera->pos) / (float)sim->dx;

        if (gridPos.x >= 1 && gridPos.x <= SIZEX - 2 && gridPos.y >= 1 && gridPos.y <= SIZEY - 2) {
            printf("pos: (%f, %f), velIncrease: (%f, %f)\n", gridPos.x, gridPos.y, velIncrease.x, velIncrease.y);
            size_t ux = utils::clamp<size_t>(gridPos.x, 1, SIZEX - 2);
            size_t uy = utils::clamp<size_t>(gridPos.y - 0.5, 1, SIZEY - 2);
            sim->mac.u(ux, uy) += velIncrease.x;
            size_t vx = utils::clamp<size_t>(gridPos.x - 0.5, 1, SIZEX - 2);
            size_t vy = utils::clamp<size_t>(gridPos.y, 1, SIZEY - 2);
            sim->mac.v(vx, vy) += velIncrease.y;
        }
    }
}

void WaterRenderer2D::draw() {
    if (renderCells) {
        cellShader.use();
        cellShader.setVector4("uniColor", vec4f(0.0f, 0.0f, 1.0f, 1.0f));
        glBindVertexArray(waterCellVAO);
        glDrawArraysInstanced(GL_TRIANGLES, 0, 6, waterCellLocations.size);
        cellShader.setVector4("uniColor", vec4f(0.5f, 0.5f, 0.5f, 1.0f));
        glBindVertexArray(solidCellVAO);
        glDrawArraysInstanced(GL_TRIANGLES, 0, 6, solidCellLocations.size);
        glBindVertexArray(0);
    }
    if (renderCellVels) {
        particleShader.use();
        glBindVertexArray(cellVelVAO);
        glDrawArrays(GL_LINES, 0, particleVelLines.size);
        glBindVertexArray(0);
    }
    if (renderPressures) {
        cellFieldShader.use();
        cellFieldShader.setVector4("posColor", vec4f(1.0f, 0.0f, 0.0f, 1.0f));
        cellFieldShader.setVector4("negColor", vec4f(0.0f, 0.0f, 1.0f, 1.0f));
        glBindVertexArray(pressureCellVAO);
        glDrawArraysInstanced(GL_TRIANGLES, 0, 6, pressureCellLocations.size);
        glBindVertexArray(0);
    }
    if (renderParticles) {
        particleShader.use();
        glBindVertexArray(particleVAO);
        glDrawArrays(GL_POINTS, 0, particleVelLines.size / 2);
        glBindVertexArray(0);
    }
    if (renderParticleVels) {
        particleShader.use();
        glBindVertexArray(particleVelVAO);
        glDrawArrays(GL_LINES, 0, particleVelLines.size);
        glBindVertexArray(0);
    }

    if (renderLevelSet) {
        cellFieldShader.use();
        cellFieldShader.setVector4("posColor", vec4f(1.0f, 0.0f, 0.0f, 1.0f));
        cellFieldShader.setVector4("negColor", vec4f(0.0f, 0.0f, 1.0f, 1.0f));
        glBindVertexArray(phiCellVAO);
        glDrawArraysInstanced(GL_TRIANGLES, 0, 6, allCellLocations.size);
        glBindVertexArray(0);
    }

}

void WaterRenderer2D::drawUI() {
    ImGui::Begin("Water Simulation");
    ImGui::Text("Current frame: %f", sim->currentTime);
    ImGui::Text("Current stage: %s", sim->printStage());
    ImGui::Text("Average pressure: %f", sim->avgPressure());
    ImGui::Text("Average pressure in fluid: %f", sim->avgPressureInFluid());
    double maxVelocity = sim->maxVelocity();
    ImGui::Text("Max velocity in fluid: %f", maxVelocity);
    double CFLnum = sim->dt * maxVelocity / sim->dx;
    ImGui::Text("CFL Number: %f", CFLnum);

    ImGui::Checkbox("Render cells", &renderCells);
    ImGui::Checkbox("Render cell vels", &renderCellVels);
    ImGui::Checkbox("Render pressures", &renderPressures);
    ImGui::Checkbox("Render particles", &renderParticles);
    ImGui::Checkbox("Render particle vels", &renderParticleVels);
    ImGui::Checkbox("Render level set", &renderLevelSet);

    ImGui::End();
}

void WaterRenderer2D::updateBuffers() {
    if (renderCells) {
        waterCellLocations.size = 0;
        solidCellLocations.size = 0;
        sim->iterate([&](size_t i, size_t j) {
            WaterSim2D::CellType cellType = sim->cell(i,j);
            if (cellType == WaterSim2D::CellType::FLUID) {
                waterCellLocations.push(vec2fp(i * sim->dx, j * sim->dx));
            }
            else if (cellType == WaterSim2D::CellType::SOLID) {
                solidCellLocations.push(vec2fp(i * sim->dx, j * sim->dx));
            }
        });
    }
    if (renderCellVels) {
        cellVels.size = 0;
        for (size_t j = 0; j < sim->SIZEY - 1; j++) {
            for (size_t i = 0; i < sim->SIZEX - 1; i++) {
                vec2d pos = vec2d(i + 0.5,j + 0.5) * sim->dx;
                vec2d vel = sim->mac.velInterp(pos);
                cellVels.push(vec2fp(pos.x, pos.y));
                cellVels.push(vec2fp(pos.x + vel.x * sim->dt, pos.y + vel.y * sim->dt));
            }
        }
    }
    if (renderPressures) {
        pressureCellLocations.size = 0;
        pressureCellValues.size = 0;
        sim->iterate([&](size_t i, size_t j) {
            if (sim->p(i,j) != 0) {
                pressureCellLocations.push(vec2fp(i * sim->dx, j * sim->dx));
                float p = (float)sim->p(i,j);
                pressureCellValues.push(utils::sigmoid(0.01f * p));
            }
        });
    }
    if (renderParticles || renderParticleVels) {
        particleVelLines.size = 2*sim->particles.size;
        for (size_t i = 0; i < sim->particles.size; i++) {
            particleVelLines[2*i].x = (float)sim->particles[i].x;
            particleVelLines[2*i].y = (float)sim->particles[i].y;
            particleVelLines[2*i + 1].x = (float)sim->particles[i].x + (float)sim->dt * (float)sim->mac.velInterp(sim->particles[i]).x;
            particleVelLines[2*i + 1].y = (float)sim->particles[i].y + (float)sim->dt * (float)sim->mac.velInterp(sim->particles[i]).y;
        }
    }
    phiCellValues.size = 0;
    if (renderLevelSet) {
        sim->iterate([&](size_t i, size_t j) {
            phiCellValues.push(utils::sigmoid<float>(100.0f * sim->phi(i,j)));
        });
    }
}

/*
void WaterRenderer2D::createWaterMesh() {
    float dx = (float)sim->dx;
    StackVec<vec2p, 4> contourLookupTable[16] = {};
    contourLookupTable[1].push(HMM_Vec2(0, 0.5f));
    contourLookupTable[1].push(HMM_Vec2(0.5f, 1));
    contourLookupTable[2].push(HMM_Vec2(0.5f, 1));
    contourLookupTable[2].push(HMM_Vec2(1, 0.5f));
    contourLookupTable[3].push(HMM_Vec2(0, 0.5f));
    contourLookupTable[3].push(HMM_Vec2(1, 0.5f));
    contourLookupTable[4].push(HMM_Vec2(0.5f, 0));
    contourLookupTable[4].push(HMM_Vec2(1, 0.5f));
    contourLookupTable[5].push(HMM_Vec2(0, 0.5f));
    contourLookupTable[5].push(HMM_Vec2(0.5f, 0));
    contourLookupTable[5].push(HMM_Vec2(0.5f, 1));
    contourLookupTable[5].push(HMM_Vec2(1, 0.5f));
    contourLookupTable[6].push(HMM_Vec2(0.5f, 0));
    contourLookupTable[6].push(HMM_Vec2(0.5f, 1));
    contourLookupTable[7].push(HMM_Vec2(0, 0.5f));
    contourLookupTable[7].push(HMM_Vec2(0.5f, 0));
    contourLookupTable[8].push(HMM_Vec2(0, 0.5f));
    contourLookupTable[8].push(HMM_Vec2(0.5f, 0));
    contourLookupTable[9].push(HMM_Vec2(0.5f, 0));
    contourLookupTable[9].push(HMM_Vec2(0.5f, 1));
    contourLookupTable[10].push(HMM_Vec2(0.5f, 1));
    contourLookupTable[10].push(HMM_Vec2(0.5f, 1));
    contourLookupTable[10].push(HMM_Vec2(0.5f, 1));
    contourLookupTable[10].push(HMM_Vec2(0.5f, 1));

    auto phiSign = new Array2D<bool, SIZEX, SIZEY>();
    defer {delete phiSign;};
    sim->iterate([&](size_t i, size_t j) {
        (*phiSign)(i,j) = sim->phi(i,j) > 0;
    });

    for (size_t j = 0; j < SIZEY - 1; j++) {
        for (size_t i = 0; i < SIZEX - 1; i++) {
            int key = ((*phiSign)(i+1,j+1) * 8) + ((*phiSign)(i,j+1) * 4)
                      + ((*phiSign)(i+1,j) * 2) + ((*phiSign)(i,j));
            switch (key) {
                case 0:

            }

        }
    }
}
 */
