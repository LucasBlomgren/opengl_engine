#pragma once

#include <vector>
#include "physics/bodies/rigidbody.h"

namespace physics::internal {

namespace WakeSleep
{
    struct WakeUpInfo {
        bool A = false;
        bool B = false;
    };

    WakeUpInfo computeWakeUpInfo(
        const RigidBody& A,
        const RigidBody& B,
        float velocityThreshold = 0.4f, // 6
        float angularThreshold = 0.2f  // 4
    );

    void enqueueWakeRequests(
        const WakeUpInfo& info,
        RigidBody& A, RigidBody& B,
        const BodyHandle& handleA,
        const BodyHandle& handleB,
        std::vector<BodyHandle>& toWake
    );

    bool updateSleepStateAndCheckIfShouldSleep(
        RigidBody& body,
        float dt,
        float jitterThreshold = 1.0f,
        float anchorTimerThreshold = 6.0f
    );
}

}
