#pragma once
#include <chrono>
#include <unordered_map>
#include <string>

struct GpuTimers {
    double shadowMs = 0.0;
    double mainMs = 0.0;
    double debugMs = 0.0;

    double totalMs() const { return shadowMs + mainMs + debugMs; }
};

class TimerStats {
public:
    float lastMs = 0.0f;
    float smoothedMs = 0.0f;

    void update(float ms, float alpha = 0.2f) {
        lastMs = ms;
        if (smoothedMs == 0.0f) {
            smoothedMs = ms;
        } else {
            smoothedMs = smoothedMs + alpha * (ms - smoothedMs);
        }
    }
};

class FrameTimers {
public:
    void beginFrame();
    void endFrame();

    void submit(const std::string& name, float ms);

    float get(const std::string& name) const;
    float getSmooth(const std::string& name) const;

private:
    using Clock = std::chrono::high_resolution_clock;
    Clock::time_point frameStart;

    std::unordered_map<std::string, TimerStats> timers;
};

class ScopedTimer {
public:
    ScopedTimer(FrameTimers& ft, const char* name);
    ~ScopedTimer();

private:
    FrameTimers& timers;
    const char* name;
    std::chrono::high_resolution_clock::time_point start;
};