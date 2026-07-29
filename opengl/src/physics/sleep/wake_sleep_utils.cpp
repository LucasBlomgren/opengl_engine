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
        const RigidBody& A,
        const RigidBody& B,
        float velocityThreshold,
        float angularThreshold)
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
        if (A.asleep && A.type != BodyType::Static && A.allowSleep) {
            if (Bv2 > v2 || Bw2 > w2) {
                info.A = true;
            }
        }

        // Same logic for body B: if it's asleep and allowed to sleep,
        if (B.asleep && B.type != BodyType::Static && B.allowSleep) {
            if (Av2 > v2 || Aw2 > w2) {
                info.B = true;
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
        const WakeUpInfo& info,
        RigidBody& A, RigidBody& B,
        const BodyHandle& handleA,
        const BodyHandle& handleB,
        std::vector<BodyHandle>& toWake)
    {
        if (info.A && !A.inSleepTransition) {
            toWake.push_back(handleA);
            A.inSleepTransition = true;
        }

        if (info.B && !B.inSleepTransition) {
            toWake.push_back(handleB);
            B.inSleepTransition = true;
        }
    }

    //========================================================================
    // Update the sleep state of a rigid body and check if it should
    // go to sleep based on its velocity and anchor point.
    //========================================================================
    bool updateSleepStateAndCheckIfShouldSleep(
        RigidBody& body,
        float dt,
        float jitterThreshold,
        float anchorTimerThreshold)
    {
        bool goingToSleep = false;

        // Sleep check based on anchor point:
        // if the body is staying close to the anchor point, increment the
        // anchor timer.
        // Example: even if the body is moving fast, but is stuck somewhere,
        // it will eventually go to sleep.
        if (glm::abs(body.anchorPoint.x - body.pose.position.x) < jitterThreshold &&
            glm::abs(body.anchorPoint.y - body.pose.position.y) < jitterThreshold &&
            glm::abs(body.anchorPoint.z - body.pose.position.z) < jitterThreshold)
        {
            body.anchorTimer += dt;
        }
        else {
            body.anchorTimer = glm::max(0.0f, body.anchorTimer - dt);
        }

        if (body.anchorTimer == 0.0f) {
            body.anchorPoint = body.pose.position;
        }

        if (body.anchorTimer >= anchorTimerThreshold) {
            goingToSleep = true;
        }

        // Sleep check based on velocity thresholds:
        // if the body is moving slowly, increment the sleep counter.
        if (glm::length(body.linearVelocity) < body.velocityThreshold &&
            glm::length(body.angularVelocity) < body.angularVelocityThreshold)
        {
            body.sleepCounter += dt;
        }
        else {
            body.sleepCounter = 0.0f;
        }

        if (body.sleepCounter >= body.sleepCounterThreshold) {
            goingToSleep = true;
        }

        return goingToSleep;
    }
}

}
