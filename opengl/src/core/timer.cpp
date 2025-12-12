#include "timer.h"

void FrameTimers::beginFrame() {
    frameStart = Clock::now();
}

void FrameTimers::endFrame() {
    auto end = Clock::now();
    float ms = std::chrono::duration<float, std::milli>(end - frameStart).count();
    submit("Frame", ms);
}

void FrameTimers::submit(const std::string& name, float ms) {
    timers[name].update(ms);
}

float FrameTimers::get(const std::string& name) const {
    auto it = timers.find(name);
    return (it != timers.end()) ? it->second.lastMs : 0.0f;
}

float FrameTimers::getSmooth(const std::string& name) const {
    auto it = timers.find(name);
    return (it != timers.end()) ? it->second.smoothedMs : 0.0f;
}

ScopedTimer::ScopedTimer(FrameTimers& ft, const char* n)
    : timers(ft), name(n), start(std::chrono::high_resolution_clock::now()) {
}

ScopedTimer::~ScopedTimer() {
    auto end = std::chrono::high_resolution_clock::now();
    float ms = std::chrono::duration<float, std::milli>(end - start).count();
    timers.submit(name, ms);
}