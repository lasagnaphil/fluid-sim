//
// Created by lasagnaphil on 10/27/18.
//

#include "FluidRenderer2D.h"

#include <vec4f.h>
#include "App.h"

const char* FluidRenderer2D::particleVS = R"SHADER(
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

const char* FluidRenderer2D::cellVS = R"SHADER(
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

const char* FluidRenderer2D::cellFieldVS = R"SHADER(
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

const char* FluidRenderer2D::cellFS = R"SHADER(
#version 330 core

in vec4 color;

out vec4 fragColor;

void main() {
	fragColor = color;
}
)SHADER";

FluidRenderer2D FluidRenderer2D::create(FluidSim2D *sim, Camera2D *camera) {
    FluidRenderer2D ren;
    ren.sim = sim;
    ren.camera = camera;
    int sizeX = ren.sizeX = sim->sizeX;
    int sizeY = ren.sizeY = sim->sizeY;
    ren.particlesPerCellSqrt = sim->particlesPerCellSqrt;
    ren.particlesPerCell = sim->particlesPerCell;

    ren.waterCellLocations.reserve(sizeX*sizeY);
    ren.solidCellLocations.reserve(sizeX*sizeY);
    ren.pressureCellLocations.reserve(sizeX*sizeY);
    ren.cellVels.reserve(2*sizeX*sizeY);
    ren.pressureCellValues.reserve(sizeX*sizeY);
    ren.allCellLocations.reserve(sizeX*sizeY);
    ren.phiCellValues.reserve(sizeX*sizeY);
    ren.particleVelLines.reserve(2*ren.particlesPerCell*sizeX*sizeY);

    return ren;
}

void FluidRenderer2D::setup() {
    cellShader = Shader::fromStr(cellVS, cellFS);
    particleShader = Shader::fromStr(particleVS, cellFS);
    cellFieldShader = Shader::fromStr(cellFieldVS, cellFS);

    memcpy(quadVertices, origQuadVertices, sizeof(origQuadVertices));
    for (int i = 0; i < 6; i++) {
        quadVertices[i].x *= sim->dx;
        quadVertices[i].y *= sim->dx;
    }

    allCellLocations.size = sizeX*sizeY;
    sim->iterate([&](size_t i, size_t j) {
        allCellLocations[j*sizeX + i] = vec2f{(float)i,(float)j} * (float)sim->dx;
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
    glBufferData(GL_ARRAY_BUFFER, sizeof(vec2f) * 6, quadVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(vec2f), 0);

    glBindBuffer(GL_ARRAY_BUFFER, waterCellOffsetVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vec2f) * sizeX*sizeY, waterCellLocations.data, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(vec2f), 0);
    glVertexAttribDivisor(1, 1);

    glBindVertexArray(solidCellVAO);

    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(vec2f), 0);

    glBindBuffer(GL_ARRAY_BUFFER, solidCellOffsetVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vec2f) * sizeX*sizeY, solidCellLocations.data, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(vec2f), 0);
    glVertexAttribDivisor(1, 1);

    glBindVertexArray(cellVelVAO);

    glBindBuffer(GL_ARRAY_BUFFER, cellVelVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vec2f) * 2*sizeX*sizeY, cellVels.data, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(vec2f), 0);

    glBindVertexArray(pressureCellVAO);

    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(vec2f), 0);

    glBindBuffer(GL_ARRAY_BUFFER, pressureCellOffsetVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vec2f) * sizeX*sizeY, pressureCellLocations.data, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(vec2f), 0);
    glVertexAttribDivisor(1, 1);

    glBindBuffer(GL_ARRAY_BUFFER, pressureCellValueVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * sizeX*sizeY, pressureCellValues.data, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(float), 0);
    glVertexAttribDivisor(2, 1);

    glBindVertexArray(phiCellVAO);

    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(vec2f), 0);

    glBindBuffer(GL_ARRAY_BUFFER, allCellOffsetVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vec2f) * sizeX*sizeY, allCellLocations.data, GL_STATIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(vec2f), 0);
    glVertexAttribDivisor(1, 1);

    glBindBuffer(GL_ARRAY_BUFFER, phiCellValueVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * sizeX*sizeY, phiCellValues.data, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(float), 0);
    glVertexAttribDivisor(2, 1);

    glBindVertexArray(particleVAO);

    glBindBuffer(GL_ARRAY_BUFFER, particleVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vec2f) * 2*particlesPerCell*sizeX*sizeY, particleVelLines.data, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(vec2f), 0);

    glBindVertexArray(particleVelVAO);

    glBindBuffer(GL_ARRAY_BUFFER, particleVelVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vec2f) * 2*particlesPerCell*sizeX*sizeY, particleVelLines.data, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(vec2f), 0);

    glBindVertexArray(0);

    camera->addShader(&cellShader);
    camera->addShader(&particleShader);
    camera->addShader(&cellFieldShader);
}

void FluidRenderer2D::free() {
    waterCellLocations.free();
    solidCellLocations.free();
    pressureCellLocations.free();
    cellVels.free();
    pressureCellValues.free();
    allCellLocations.free();
    phiCellValues.free();
    particleVelLines.free();
}

void FluidRenderer2D::update() {
    if (!sim->rendered) {
        updateBuffers();
        if (renderCells) {
            glBindVertexArray(waterCellVAO);
            glBindBuffer(GL_ARRAY_BUFFER, waterCellOffsetVBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vec2f) * waterCellLocations.size, waterCellLocations.data);
            glBindVertexArray(solidCellVAO);
            glBindBuffer(GL_ARRAY_BUFFER, solidCellOffsetVBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vec2f) * solidCellLocations.size, solidCellLocations.data);
            glBindVertexArray(0);
        }
        if (renderCellVels) {
            glBindVertexArray(cellVelVAO);
            glBindBuffer(GL_ARRAY_BUFFER, cellVelVBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vec2f) * cellVels.size, cellVels.data);
            glBindVertexArray(0);
        }
        if (renderPressures) {
            glBindVertexArray(pressureCellVAO);
            glBindBuffer(GL_ARRAY_BUFFER, pressureCellOffsetVBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vec2f) * pressureCellLocations.size, pressureCellLocations.data);
            glBindVertexArray(particleVelVAO);
            glBindBuffer(GL_ARRAY_BUFFER, pressureCellValueVBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * pressureCellValues.size, pressureCellValues.data);
            glBindVertexArray(0);
        }
        if (renderParticles) {
            glBindVertexArray(particleVAO);
            glBindBuffer(GL_ARRAY_BUFFER, particleVBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vec2f) * particleVelLines.size, particleVelLines.data);
            glBindVertexArray(0);
        }
        if (renderParticleVels) {
            glBindVertexArray(particleVelVAO);
            glBindBuffer(GL_ARRAY_BUFFER, particleVelVBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vec2f) * particleVelLines.size, particleVelLines.data);
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

    auto inputMgr = InputManager::get();

    // Change velocity using mouse drag (only works for Semi-Lagrangian for now)
    static vec2f startLeftMousePos;
    vec2f currLeftMousePos;
    constexpr float VEL_MOUSEDRAG_SCALE = 0.1f;
    if (inputMgr->isMouseEntered(SDL_BUTTON_LEFT)) {
        startLeftMousePos = aml::toFloat(inputMgr->getMousePos());
    }
    else if (inputMgr->isMousePressed(SDL_BUTTON_LEFT)) {
        currLeftMousePos = aml::toFloat(inputMgr->getMousePos());
        vec2f velIncrease = VEL_MOUSEDRAG_SCALE * (currLeftMousePos - startLeftMousePos);

        vec2f screenSize = aml::toFloat(camera->settings->screenSize);
        vec2f normMousePos = startLeftMousePos - (screenSize / 2.f);
        normMousePos.y *= -1;
        vec2f gridPos = (normMousePos / camera->pixelsPerMeter / camera->zoom + camera->pos) / (float)sim->dx;

        if (gridPos.x >= 1 && gridPos.x <= sizeX - 2 && gridPos.y >= 1 && gridPos.y <= sizeY - 2) {
            printf("pos: (%f, %f), velIncrease: (%f, %f)\n", gridPos.x, gridPos.y, velIncrease.x, velIncrease.y);
            size_t ux = aml::clamp<size_t>(gridPos.x, 1, sizeX - 2);
            size_t uy = aml::clamp<size_t>(gridPos.y - 0.5, 1, sizeY - 2);
            sim->mac.u(ux, uy) += velIncrease.x;
            size_t vx = aml::clamp<size_t>(gridPos.x - 0.5, 1, sizeX - 2);
            size_t vy = aml::clamp<size_t>(gridPos.y, 1, sizeY - 2);
            sim->mac.v(vx, vy) += velIncrease.y;
        }
    }

    // Change gravity using mouse drag
    static vec2f startRightMousePos;
    vec2f currRightMousePos;
    constexpr float GRAVITY_MOUSEDRAG_SCALE = 0.002f;
    if (inputMgr->isMouseEntered(SDL_BUTTON_RIGHT)) {
        startRightMousePos = aml::toFloat(inputMgr->getMousePos());
    }
    else if (inputMgr->isMousePressed(SDL_BUTTON_RIGHT)) {
        currRightMousePos = aml::toFloat(inputMgr->getMousePos());
        auto gravity = GRAVITY_MOUSEDRAG_SCALE * (currRightMousePos - startRightMousePos);
        sim->gravity = vec2d {sim->origGravity.x + gravity.x, sim->origGravity.y + gravity.y};
    }

}

void FluidRenderer2D::draw() {
    if (renderCells) {
        cellShader.use();
        cellShader.setVector4("uniColor", vec4f {0.0f, 0.0f, 1.0f, 1.0f});
        glBindVertexArray(waterCellVAO);
        glDrawArraysInstanced(GL_TRIANGLES, 0, 6, waterCellLocations.size);
        cellShader.setVector4("uniColor", vec4f {0.5f, 0.5f, 0.5f, 1.0f});
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
        cellFieldShader.setVector4("posColor", vec4f {1.0f, 0.0f, 0.0f, 1.0f});
        cellFieldShader.setVector4("negColor", vec4f {0.0f, 0.0f, 1.0f, 1.0f});
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
        cellFieldShader.setVector4("posColor", vec4f {1.0f, 0.0f, 0.0f, 1.0f});
        cellFieldShader.setVector4("negColor", vec4f {0.0f, 0.0f, 1.0f, 1.0f});
        glBindVertexArray(phiCellVAO);
        glDrawArraysInstanced(GL_TRIANGLES, 0, 6, allCellLocations.size);
        glBindVertexArray(0);
    }

}

void FluidRenderer2D::drawUI() {
    ImGui::Begin("Water Simulation");
    if (ImGui::CollapsingHeader("Parameters")) {
        float gravity[2];
        gravity[0] = (float)sim->gravity.x;
        gravity[1] = (float)sim->gravity.y;
        ImGui::SliderFloat2("Gravity", gravity, -30.0f, 30.0f);
        sim->gravity.x = gravity[0];
        sim->gravity.y = gravity[1];

        float picFlipAlpha = sim->picFlipAlpha;
        ImGui::SliderFloat("PIC/Flip Interpolation Factor", &picFlipAlpha, 0.0f, 1.0f);
        sim->picFlipAlpha = picFlipAlpha;
    }
    if (ImGui::CollapsingHeader("Data")) {
        ImGui::Text("Current frame: %f", sim->currentTime);
        ImGui::Text("Current stage: %s", sim->printStage());
        ImGui::Text("Average pressure: %f", sim->avgPressure());
        ImGui::Text("Average pressure in fluid: %f", sim->avgPressureInFluid());
        double maxVelocity = sim->maxVelocity();
        ImGui::Text("Max velocity in fluid: %f", maxVelocity);
        double CFLnum = sim->dt * maxVelocity / sim->dx;
        ImGui::Text("CFL Number: %f", CFLnum);

        ImGui::Text("Original fluid volume: %f", sim->origWaterVolume);
        ImGui::Text("Current fluid volume: %f", sim->waterVolume);
        if (sim->origWaterVolume < sim->waterVolume) {
            ImGui::Text("(%f %% volume increase) ", (sim->waterVolume - sim->origWaterVolume) / sim->origWaterVolume * 100.f);
        }
        else if (sim->origWaterVolume > sim->waterVolume) {
            ImGui::Text("(%f %% volume decrease) ", (sim->origWaterVolume - sim->waterVolume) / sim->origWaterVolume * 100.f);
        }
        ImGui::Text("Total energy: %f", sim->totalEnergy);
        ImGui::Text("Total energy (Particle): %f", sim->particleTotalEnergy);

        ImGui::Text("Current timestep: %f", sim->dt);
    }
    if (ImGui::CollapsingHeader("Options")) {
        ImGui::Checkbox("Render cells", &renderCells);
        ImGui::Checkbox("Render cell vels", &renderCellVels);
        ImGui::Checkbox("Render pressures", &renderPressures);
        ImGui::Checkbox("Render particles", &renderParticles);
        ImGui::Checkbox("Render particle vels", &renderParticleVels);
        ImGui::Checkbox("Render level set", &renderLevelSet);
    }
    if (ImGui::CollapsingHeader("Performance")) {
        sim->perfCounter.renderUI();
    }

    ImGui::End();
}

void FluidRenderer2D::updateBuffers() {
    if (renderCells) {
        waterCellLocations.size = 0;
        solidCellLocations.size = 0;
        sim->iterate([&](size_t i, size_t j) {
            FluidCellType cellType = sim->cell(i,j);
            if (cellType == FS_FLUID) {
                waterCellLocations.push(vec2f {(float)(i * sim->dx), (float)(j * sim->dx)});
            }
            else if (cellType == FS_SOLID) {
                solidCellLocations.push(vec2f {(float)(i * sim->dx), (float)(j * sim->dx)});
            }
        });
    }
    if (renderCellVels) {
        cellVels.size = 0;
        for (size_t j = 0; j < sim->sizeY - 1; j++) {
            for (size_t i = 0; i < sim->sizeX - 1; i++) {
                vec2d pos = vec2d {i + 0.5, j + 0.5} * sim->dx;
                vec2d vel = sim->mac.velInterp(pos);
                cellVels.push(vec2f {(float)pos.x, (float)pos.y});
                cellVels.push(vec2f {(float)(pos.x + vel.x * sim->dt), (float)(pos.y + vel.y * sim->dt)});
            }
        }
    }
    if (renderPressures) {
        pressureCellLocations.size = 0;
        pressureCellValues.size = 0;
        sim->iterate([&](size_t i, size_t j) {
            if (sim->p(i,j) != 0) {
                pressureCellLocations.push(vec2f {(float)(i * sim->dx), (float)(j * sim->dx)});
                float p = (float)sim->p(i,j);
                pressureCellValues.push(aml::sigmoid(0.01f * p));
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
            phiCellValues.push(aml::sigmoid<float>(100.0f * sim->waterLevelSet.phi(i,j)));
        });
    }
}
