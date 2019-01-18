//
// Created by lasagnaphil on 2018-11-26.
//

#include <File.h>
#include "PerformanceCounter.h"
#include <string>

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
        averageStore.push(average[i]);
        avgTimePerFrame += average[i];
    }
}

void PerformanceCounter::free() {
    samples.free();
    average.free();
    averageStore.free();
}

void PerformanceCounter::saveToFile(const char* filename) {
    File file = File::open(filename, "w+").unwrap();

    std::string buf;

    int numStages = average.size;
    for (int i = SampleCount; i < averageStore.size / numStages; i++) {
        for (int j = 0; j < numStages; j++) {
            auto str = std::to_string(averageStore[numStages*i + j]);
            buf += str;
            buf += ',';
        }
        buf += '\n';
    }
    file.writeAll(buf.c_str());
    file.close();
}
