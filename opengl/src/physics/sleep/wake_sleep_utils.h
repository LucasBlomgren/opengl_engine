#pragma once

#include <vector>
#include "physics/world/physics_world.h"
#include "physics/bodies/rigidbody.h"

namespace physics::internal {

namespace WakeSleep
{
    struct WakeUpInfo {
        bool A = false;
        bool B = false;
    };

    inline constexpr float velocityThreshold = 0.4f;
    inline constexpr float angularThreshold = 0.2f;

    inline constexpr float jitterThreshold = 1.0f;
    inline constexpr float anchorTimerThreshold = 6.0f;

    WakeUpInfo computeWakeUpInfo(
        PhysicsWorld& world,
        const RigidBody& A,
        const RigidBody& B
    );

    void enqueueWakeRequests(
        PhysicsWorld& world,
        const WakeUpInfo& info,
        RigidBody& A, 
        RigidBody& B,
        const BodyHandle& handleA,
        const BodyHandle& handleB,
        std::vector<BodyHandle>& toWake
    );

    bool updateSleepState(
        PhysicsWorld& world,
        RigidBody& body,
        float dt
    );
}

}
