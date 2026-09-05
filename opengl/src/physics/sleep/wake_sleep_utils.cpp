#include "pch.h"
#include "wake_sleep_utils.h"

namespace physics::internal {

namespace WakeSleep
{
    //========================================================================
    // Compute wake-up info for two rigid bodies A and B based on their
    // velocities and sleep states.
    //========================================================================
    WakeUpInfo computeWakeUpInfo(
        PhysicsWorld& world,
        const RigidBody& A,
        const RigidBody& B)
    {
        const float v2 = velocityThreshold * velocityThreshold;
        const float w2 = angularThreshold * angularThreshold;

        const float Av2 = glm::dot(A.linearVelocity, A.linearVelocity);
        const float Aw2 = glm::dot(A.angularVelocity, A.angularVelocity);
        const float Bv2 = glm::dot(B.linearVelocity, B.linearVelocity);
        const float Bw2 = glm::dot(B.angularVelocity, B.angularVelocity);

        WakeUpInfo info{};

        // If body A is asleep and allowed to sleep,
        // check if body B's velocities exceed the thresholds.
        // If so, mark body A to be woken up.
        if (A.type == BodyType::Dynamic) {
            SleepState& sleepA = world.getSleepState(A.sleepStateHandle);

            if (sleepA.asleep && sleepA.allowSleep) {
                if (Bv2 > v2 || Bw2 > w2) {
                    info.A = true;
                }
            }
        }

        // Same logic for body B: if it's asleep and allowed to sleep,
        if (B.type == BodyType::Dynamic) {
            SleepState& sleepB = world.getSleepState(B.sleepStateHandle);

            if (sleepB.asleep && sleepB.allowSleep) {
                if (Av2 > v2 || Aw2 > w2) {
                    info.B = true;
                }
            }
        }

        return info;
    }

    //========================================================================
    // Enqueue wake-up requests for bodies A and B based on
    // the computed wake-up info.
    // This function adds the handles of bodies that need to be woken up
    // to the toWake vector and marks them as in sleep transition.
    //========================================================================
    void enqueueWakeRequests(
        PhysicsWorld& world,
        const WakeUpInfo& info,
        RigidBody& A, 
        RigidBody& B,
        const BodyHandle& handleA,
        const BodyHandle& handleB,
        std::vector<BodyHandle>& toWake)
    {
        if (A.type == BodyType::Dynamic) {
            SleepState& sleepA = world.getSleepState(A.sleepStateHandle);
            if (info.A && !sleepA.inSleepTransition) {
                toWake.push_back(handleA);
                sleepA.inSleepTransition = true;
            }

        }

        if (B.type == BodyType::Dynamic) {
            SleepState& sleepB = world.getSleepState(B.sleepStateHandle);
            if (info.B && !sleepB.inSleepTransition) {
                toWake.push_back(handleB);
                sleepB.inSleepTransition = true;
            }
        }
    }

    //========================================================================
    // Calculate sleep thresholds for a rigid body based on its collision history.
    // This function returns a pair of linear and angular velocity thresholds
    // or std::nullopt if the body is asleep, not allowed to sleep, or not dynamic.
    //========================================================================
    std::optional<std::pair<float, float>> calculateSleepThresholds(
        RigidBody& body, 
        SleepState& sleepState) 
    {
        sleepState.collisionHistory.push(sleepState.collisionCount);
        sleepState.collisionCount = 0;
        float avg = sleepState.collisionHistory.average();

        if (avg <= 0.0f) {
            if (std::abs(avg - sleepState.lastAvg) >= 1) {
                sleepState.counter = 0.0f;
            }
            sleepState.lastAvg = avg;

            return std::nullopt;
        }

        avg = std::max(avg, 1.0f);
        sleepState.lastAvg = avg;

        constexpr float linearFactor = 0.2f;
        constexpr float angularFactor = 0.1f;

        // set thresholds
        float linearVelocityThreshold = avg * linearFactor;
        float angularVelocityThreshold = avg * angularFactor * body.invRadius;

        return std::pair<float, float>(linearVelocityThreshold, angularVelocityThreshold);
    }

    //========================================================================
    // Update the sleep state of a rigid body and check if it should
    // go to sleep based on its velocity and anchor point.
    //========================================================================
    bool updateSleepState(
        PhysicsWorld& world,
        RigidBody& body,
        float dt)
    {
        bool goingToSleep = false;

        SleepState& sleepState = world.getSleepState(body.sleepStateHandle);

        // Sleep check based on anchor point:
        // if the body is staying close to the anchor point, increment the
        // anchor timer.
        // Example: even if the body is moving fast, but is stuck somewhere,
        // it will eventually go to sleep.
        if (glm::abs(sleepState.anchorPoint.x - body.pose.position.x) < jitterThreshold &&
            glm::abs(sleepState.anchorPoint.y - body.pose.position.y) < jitterThreshold &&
            glm::abs(sleepState.anchorPoint.z - body.pose.position.z) < jitterThreshold)
        {
            sleepState.anchorTimer += dt;
        }
        else {
            sleepState.anchorTimer = glm::max(0.0f, sleepState.anchorTimer - dt);
        }

        if (sleepState.anchorTimer == 0.0f) {
            sleepState.anchorPoint = body.pose.position;
        }

        if (sleepState.anchorTimer >= anchorTimerThreshold) {
            goingToSleep = true;
            return goingToSleep;
        }

        // Sleep check based on velocity thresholds:
        // if the body is moving slowly, increment the sleep counter.
        std::optional<std::pair<float, float>> thresholds = calculateSleepThresholds(body, sleepState);
        if (!thresholds) {
            return goingToSleep;
        }

        float linearVelocityThreshold = thresholds->first;
        float angularVelocityThreshold = thresholds->second;

        if (glm::length(body.linearVelocity) < linearVelocityThreshold &&
            glm::length(body.angularVelocity) < angularVelocityThreshold)
        {
            sleepState.counter += dt;
        }
        else {
            sleepState.counter = 0.0f;
        }

        if (sleepState.counter >= sleepState.counterThreshold) {
            goingToSleep = true;
        }

        return goingToSleep;
    }

}
}