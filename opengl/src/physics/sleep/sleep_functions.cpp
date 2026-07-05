#include "pch.h"
#include "physics_engine.h"
#include "wake_sleep_utils.h"

//====================================
//   Process wake list
//====================================
void PhysicsEngine::processWakeList() {
    for (RigidBodyHandle rb : toWake) {
        broadphaseManager.moveToAwake(rb);

        RigidBody* body = caches.bodies.get(rb, FUNC_NAME);
        if (body) {
            body->inSleepTransition = false;
        }
    }

    toWake.clear();
}

//====================================
//    Process sleep list
//====================================
void PhysicsEngine::processSleepList(float outerDt) {
    toSleep.clear();

    const std::vector<RigidBodyHandle>& awakeHandles = broadphaseManager.getAwakeList();

    for (const RigidBodyHandle& handle : awakeHandles) {
        RigidBody* body = caches.bodies.get(handle, FUNC_NAME);
        if (!body) continue;

        if (body->type != BodyType::Dynamic) continue;
        if (body->motionControl == MotionControl::External) continue;
        if (body->asleep) continue;
        if (!body->allowSleep) continue;
        if (body->inSleepTransition) continue;

        Transform* transform = caches.transforms.get(body->rootTransformHandle, FUNC_NAME);
        if (!transform) continue;

        bool goingToSleep =
            WakeSleep::updateSleepStateAndCheckIfShouldSleep(*body, *transform, outerDt);

        if (goingToSleep) {
            toSleep.push_back(handle);
        }
    }

    for (RigidBodyHandle rb : toSleep) {
        broadphaseManager.moveToAsleep(rb);
    }

    toSleep.clear();
}

//====================================
//       Sleep Thresholds
//====================================
void PhysicsEngine::updateSleepThresholds() {
    const std::vector<RigidBodyHandle>& awakeHandles = broadphaseManager.getAwakeList();
    for (const RigidBodyHandle& handle : awakeHandles) {
        RigidBody* body = caches.bodies.get(handle, FUNC_NAME);

        if (body->asleep || !body->allowSleep ||
            body->motionControl == MotionControl::External || body->type != BodyType::Dynamic)
            continue;

        body->collisionHistory.push(body->totalCollisionCount);
        body->totalCollisionCount = 0;
        float avg = body->collisionHistory.average();

        if (avg <= 0.0f) {
            if (std::abs(avg - body->lastAvg) >= 1) {
                body->sleepCounter = 0.0f;
            }
            body->lastAvg = avg;

            continue;
        }

        avg = std::max(avg, 1.0f);
        body->lastAvg = avg;

        constexpr float linearFactor = 0.2f;
        constexpr float angularFactor = 0.1f;

        // set thresholds
        body->velocityThreshold = avg * linearFactor;
        body->angularVelocityThreshold = avg * angularFactor * body->invRadius;
    }
}


//====================================
//      Sleep damping
//====================================
void PhysicsEngine::addSleepDamping() {
    const std::vector<RigidBodyHandle>& awakeHandles = broadphaseManager.getAwakeList();
    for (const RigidBodyHandle& handle : awakeHandles) {
        RigidBody* body = caches.bodies.get(handle, FUNC_NAME);
        if (body->type != BodyType::Dynamic) continue;
        if (body->motionControl == MotionControl::External) continue;
        if (!body->allowSleep) continue;
        if (body->totalCollisionCount == 0) continue;
        if (body->sleepCounter <= 0.0f) continue;

        float sleepT = glm::clamp(body->sleepCounter / body->sleepCounterThreshold, 0.0f, 1.0f);

        // Smoothstep
        sleepT = sleepT * sleepT * (3.0f - 2.0f * sleepT);

        constexpr float linearDampingStrength = 5.0f;
        constexpr float angularDampingStrength = 4.5f;

        float linearFactor = std::exp(-linearDampingStrength * sleepT * dt);
        float angularFactor = std::exp(-angularDampingStrength * sleepT * dt);

        body->linearVelocity *= linearFactor;
        body->angularVelocity *= angularFactor;
    }
}