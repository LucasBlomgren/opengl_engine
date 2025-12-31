#include "timer.h"

// FrameTimers implementation
void FrameTimers::beginFrame() {
    frameStart = Clock::now();
}

// Measure and submit the frame time
void FrameTimers::endFrame() {
    auto end = Clock::now();
    float ms = std::chrono::duration<float, std::milli>(end - frameStart).count();
    submit("Frame", ms);
}

// Submit a timing measurement for a named timer
void FrameTimers::submit(const std::string& name, float ms) {
    timers[name].update(ms);
}

// Retrieve the last measured time for a named timer
float FrameTimers::get(const std::string& name) const {
    auto it = timers.find(name);
    return (it != timers.end()) ? it->second.lastMs : 0.0f;
}

// Retrieve the smoothed time for a named timer
float FrameTimers::getSmooth(const std::string& name) const {
    auto it = timers.find(name);
    return (it != timers.end()) ? it->second.smoothedMs : 0.0f;
}

// ScopedTimer implementation
ScopedTimer::ScopedTimer(FrameTimers& ft, const char* n)
    : timers(ft), name(n), start(std::chrono::high_resolution_clock::now()) {
}

// On destruction, measure elapsed time and submit to FrameTimers
ScopedTimer::~ScopedTimer() {
    auto end = std::chrono::high_resolution_clock::now();
    float ms = std::chrono::duration<float, std::milli>(end - start).count();
    timers.submit(name, ms);
}