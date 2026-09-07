#pragma once
#include <glm/ext/vector_float3.hpp>
#include "core/ring_buffer.h"

namespace physics::internal {

class SleepState {
public:
    SleepState() = default;

    SleepState(
        bool allowSleep,
        bool startAsleep,
        const glm::vec3& anchorPoint)
        : asleep(startAsleep)
        , allowSleep(allowSleep)
        , anchorPoint(anchorPoint)
    {}

    bool isAsleep() const {
        return asleep;
    }

    bool canSleep() const {
        return allowSleep;
    }

    float getCollisionHistoryAverage() const {
        return collisionHistory.average();
    }

private:
    friend class SleepManager;

    bool asleep = false;
    bool allowSleep = true;

    float counter = 0.0f;
    static constexpr float counterThreshold = 1.5f;

    float anchorTimer = 0.0f;
    glm::vec3 anchorPoint{ 0.0f };

    int collisionCount = 0;
    float lastAvg = 0.0f;
    RingBuffer collisionHistory;
};
}