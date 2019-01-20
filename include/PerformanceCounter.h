//
// Created by lasagnaphil on 2018-11-26.
//

#ifndef FLUID_SIM_PERFORMANCECOUNTER_H
#define FLUID_SIM_PERFORMANCECOUNTER_H

#include <Vec.h>
#include <chrono>

struct PerformanceCounter {
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = decltype(Clock::now());

    static constexpr int SampleCount = 30;
    TimePoint currentTime;
    int currentStage = 0;
    int currentFrame = 0;
    Vec<float[SampleCount]> samples = {};
    Vec<float> average = {};
    float avgTimePerFrame = 0.0f;
    bool sampleFinished = false;

    Vec<float> averageStore = {};

    static PerformanceCounter create(int numStages);

    void beginStage();

    void endStage();

    void endFrame();

    void saveToFile(const char* filename);

    void free();
};

#endif //FLUID_SIM_PERFORMANCECOUNTER_H
