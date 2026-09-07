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

    SleepState& ensureSleepState(BodyHandle handle);

    void wakeBody(const BodyHandle handle, RigidBody& body, SleepState& sleepState);
    void sleepBody(const BodyHandle handle, SleepState& sleepState);
    void setBodyAllowSleep(const BodyHandle handle, bool allowSleep);

    void processContactWakeUps(const ContactBatch& batch);
    void processSleepCandidates(
        const std::vector<BodyHandle>& awakeHandles,
        float dt);

    void applySleepDamping(const std::vector<BodyHandle>& awakeHandles, float dt);

private:
    PhysicsWorld& world;
    BroadphaseManager& broadphase;


    struct WakeMotion {
        float v2 = 0.0f;
        float w2 = 0.0f;
        bool valid = false;
    };

    bool shouldWake(
        const SleepState& sleepState,
        BodyHandle otherHandle,
        const RigidBody& otherBody);

    const WakeMotion& getWakeMotion(
        BodyHandle handle,
        const RigidBody& body);

    std::optional<std::pair<float, float>> calculateSleepThresholds(
        RigidBody& body,
        SleepState& sleepState);

    std::vector<WakeMotion> wakeMotionBySlot;

    const float velocityThreshold = 0.4f;
    const float angularThreshold = 0.2f;

    const float jitterThreshold = 1.0f;
    const float anchorTimerThreshold = 6.0f;

    const float v2_threshold = velocityThreshold * velocityThreshold;
    const float w2_threshold = angularThreshold * angularThreshold;
};

}
