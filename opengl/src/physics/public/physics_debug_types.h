#pragma once

#include <cstddef>

struct DebugData {
    size_t awake = 0;
    size_t asleep = 0;
    size_t staticBodies = 0;
    size_t colliders = 0;
    size_t terrainTris = 0;
    size_t contacts = 0;
    size_t currentSubstepAmount = 0;
};

enum class PhysicsStepDebugPhase {
    Ready,
    PausedBeforePositionIntegration
};