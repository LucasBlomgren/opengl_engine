#pragma once

#include <vector>
#include "rigid_body.h"

namespace WakeSleep
{
    struct WakeUpInfo {
        bool A = false;
        bool B = false;
    };

    WakeUpInfo computeWakeUpInfo(
        const RigidBody& A,
        const RigidBody& B,
        float velocityThreshold = 1.2f,
        float angularThreshold = 0.8f
    );

    void enqueueWakeRequests(
        const WakeUpInfo& info,
        RigidBody& A, RigidBody& B,
        const RigidBodyHandle& handleA,
        const RigidBodyHandle& handleB,
        std::vector<RigidBodyHandle>& toWake
    );

    bool updateSleepStateAndCheckIfShouldSleep(
        RigidBody& body,
        const Transform& transform,
        float dt,
        float jitterThreshold = 1.0f,
        float anchorTimerThreshold = 5.0f
    );
}