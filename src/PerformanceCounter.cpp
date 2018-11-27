//
// Created by lasagnaphil on 2018-11-26.
//

#include "PerformanceCounter.h"

PerformanceCounter PerformanceCounter::create(int numStages) {
    PerformanceCounter counter = {};
    counter.samples.resize(numStages);
    counter.average.resize(numStages);
    return counter;
}

void PerformanceCounter::beginStage() {
    currentTime = Clock::now();
}

void PerformanceCounter::endStage() {
    using std::chrono::duration_cast;
    using std::chrono::nanoseconds;

    auto endTime = Clock::now();
    float duration = duration_cast<nanoseconds>(endTime - currentTime).count() * 1e-6f;
    samples[currentStage][currentFrame] = duration;
    currentStage++;
}

void PerformanceCounter::endFrame() {
    if (sampleFinished) {
        for (int i = 0; i < samples.size; i++) {
            average[i] = 0.0f;
            for (int k = 0; k < SampleCount; k++) {
                average[i] += samples[i][k];
            }
            average[i] /= SampleCount;
        }
    }

    currentStage = 0;
    if (currentFrame == SampleCount - 1) sampleFinished = true;
    currentFrame = (currentFrame + 1) % SampleCount;

    avgTimePerFrame = 0.0f;
    for (int i = 0; i < average.size; i++) {
        avgTimePerFrame += average[i];
    }
}

void PerformanceCounter::renderUI() {
    if (sampleFinished) {
        for (int i = 0; i < average.size; i++) {
            ImGui::Text("Stage %d: %f ms", i, average[i]);
        }
        ImGui::Text("Avg time per frame: %f ms", avgTimePerFrame);
    } else {
        ImGui::Text("Sampling frames...");
    }
}

void PerformanceCounter::free() {
    samples.free();
    average.free();
}
