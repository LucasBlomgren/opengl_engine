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
        body.inSleepTransition = false;
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
        if (body.asleep) continue;
        if (!body.allowSleep) continue;
        if (body.inSleepTransition) continue;

        bool goingToSleep =
            WakeSleep::updateSleepStateAndCheckIfShouldSleep(body, outerDt);

        if (goingToSleep) {
            toSleep.push_back(handle);
        }
    }

    for (BodyHandle rb : toSleep) {
        broadphaseManager.moveToAsleep(rb);
    }

    toSleep.clear();
}

//====================================================================
// Sleep Thresholds: Update the sleep thresholds for awake dynamic
// bodies based on their recent collision history.
//====================================================================
void Engine::updateSleepThresholds() {
    const std::vector<BodyHandle>& awakeHandles = broadphaseManager.getAwakeList();

    for (const BodyHandle& handle : awakeHandles) {
        RigidBody& body = physicsWorld.getBody(handle);

        if (body.asleep || !body.allowSleep ||
            body.motionControl == MotionControl::External || body.type != BodyType::Dynamic)
            continue;

        body.collisionHistory.push(body.totalCollisionCount);
        body.totalCollisionCount = 0;
        float avg = body.collisionHistory.average();

        if (avg <= 0.0f) {
            if (std::abs(avg - body.lastAvg) >= 1) {
                body.sleepCounter = 0.0f;
            }
            body.lastAvg = avg;

            continue;
        }

        avg = std::max(avg, 1.0f);
        body.lastAvg = avg;

        constexpr float linearFactor = 0.2f;
        constexpr float angularFactor = 0.1f;

        // set thresholds
        body.velocityThreshold = avg * linearFactor;
        body.angularVelocityThreshold = avg * angularFactor * body.invRadius;
    }
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
        if (!body.allowSleep) continue;
        if (body.totalCollisionCount == 0) continue;
        if (body.sleepCounter <= 0.0f) continue;

        float sleepT = glm::clamp(body.sleepCounter / body.sleepCounterThreshold, 0.0f, 1.0f);

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
