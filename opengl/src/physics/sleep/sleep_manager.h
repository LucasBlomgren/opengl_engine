#pragma once

#include <utility>
#include <optional>
#include <vector>
#include "physics/public/handles.h"

namespace physics::internal {

class PhysicsWorld;
class BroadphaseManager;
class RigidBody;
class SleepState;
struct ContactBatch;

class SleepManager {
public:
    SleepManager(PhysicsWorld& world, BroadphaseManager& broadphase);

    void wakeBody(const BodyHandle handle, RigidBody& body, SleepState& sleepState);
    void sleepBody(const BodyHandle handle, SleepState& sleepState);

    void processContactWakeUps(const ContactBatch& batch);
    void processSleepCandidates(
        const std::vector<BodyHandle>& awakeHandles,
        float dt);

    void applySleepDamping(const std::vector<BodyHandle>& awakeHandles, float dt);

private:
    PhysicsWorld& world;
    BroadphaseManager& broadphase;

    std::pair<bool, bool> computeWakeUp(
        const RigidBody& A,
        const RigidBody& B,
        const SleepState* sleepA,
        const SleepState* sleepB);

    std::optional<std::pair<float, float>> calculateSleepThresholds(
        RigidBody& body,
        SleepState& sleepState);


    const float velocityThreshold = 0.4f;
    const float angularThreshold = 0.2f;

    const float jitterThreshold = 1.0f;
    const float anchorTimerThreshold = 6.0f;

    const float v2_threshold = velocityThreshold * velocityThreshold;
    const float w2_threshold = angularThreshold * angularThreshold;
};

}
