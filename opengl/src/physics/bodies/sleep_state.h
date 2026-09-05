#pragma once
#include <glm/ext/vector_float3.hpp>
#include "core/ring_buffer.h"

namespace physics::internal {

class SleepState {
public:
    bool asleep = false;
    bool allowSleep = true;

    // to avoid waking up immediately and 
    // to not add duplicate wake-up requests
    bool inSleepTransition = false;

    float counter = 0;
    float counterThreshold = 1.5f;

    float anchorTimer = 0.0f;
    glm::vec3 anchorPoint{ 0.0f };

    int collisionCount = 0;
    float lastAvg = 0.0f;
    RingBuffer collisionHistory;
};
}