#include "pch.h"
#include "wake_sleep_utils.h"
#include "physics/public/engine.h"

namespace physics {

using namespace internal;

//===================================================================
// Process wake list
//===================================================================
void Engine::processWakeList() {
    for (BodyHandle rb : toWake) {
        broadphaseManager.moveToAwake(rb);

        RigidBody& body = physicsWorld.getBody(rb);
        SleepState& sleepState = physicsWorld.getSleepState(body.sleepStateHandle);
        sleepState.inSleepTransition = false;
    }

    toWake.clear();
}

//===================================================================
// Process sleep list: Check which bodies should go to
// sleep and move them to the asleep list.
//===================================================================
void Engine::processSleepList(float outerDt) {
    toSleep.clear();

    const std::vector<BodyHandle>& awakeHandles = broadphaseManager.getAwakeList();

    for (const BodyHandle& handle : awakeHandles) {
        RigidBody& body = physicsWorld.getBody(handle);
        if (body.type != BodyType::Dynamic) continue;
        if (body.motionControl == MotionControl::External) continue;

        SleepState& sleepState = physicsWorld.getSleepState(body.sleepStateHandle);
        if (sleepState.asleep) continue;
        if (!sleepState.allowSleep) continue;
        if (sleepState.inSleepTransition) continue;

        bool goingToSleep =
            WakeSleep::updateSleepState(physicsWorld, body, outerDt);

        if (goingToSleep) {
            toSleep.push_back(handle);
        }
    }

    for (BodyHandle rb : toSleep) {
        broadphaseManager.moveToAsleep(rb);
    }

    toSleep.clear();
}

//============================================================
// Sleep damping: Apply additional damping to bodies
// that are close to going to sleep,
//============================================================
void Engine::addSleepDamping() {
    const std::vector<BodyHandle>& awakeHandles = broadphaseManager.getAwakeList();
    for (const BodyHandle& handle : awakeHandles) {
        RigidBody& body = physicsWorld.getBody(handle);
        if (body.type != BodyType::Dynamic) continue;
        if (body.motionControl == MotionControl::External) continue;

        SleepState& sleepState = physicsWorld.getSleepState(body.sleepStateHandle);
        if (sleepState.asleep) continue;
        if (!sleepState.allowSleep) continue;
        if (sleepState.inSleepTransition) continue;

        float sleepT = glm::clamp(sleepState.counter / sleepState.counterThreshold, 0.0f, 1.0f);

        // Smoothstep
        sleepT = sleepT * sleepT * (3.0f - 2.0f * sleepT);

        constexpr float linearDampingStrength = 5.0f;
        constexpr float angularDampingStrength = 4.5f;

        float linearFactor = std::exp(-linearDampingStrength * sleepT * dt);
        float angularFactor = std::exp(-angularDampingStrength * sleepT * dt);

        body.linearVelocity *= linearFactor;
        body.angularVelocity *= angularFactor;
    }
}

}
