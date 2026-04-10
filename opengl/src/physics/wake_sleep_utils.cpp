#include "pch.h"
#include "wake_sleep_utils.h"

namespace WakeSleep
{
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

        if (A.asleep && A.type != BodyType::Static && A.allowSleep) {
            if (Bv2 > v2 || Bw2 > w2) {
                info.A = true;
            }
        }

        if (B.asleep && B.type != BodyType::Static && B.allowSleep) {
            if (Av2 > v2 || Aw2 > w2) {
                info.B = true;
            }
        }

        return info;
    }

    void enqueueWakeRequests(
        const WakeUpInfo& info,
        RigidBody& A, RigidBody& B,
        const RigidBodyHandle& handleA,
        const RigidBodyHandle& handleB,
        std::vector<RigidBodyHandle>& toWake)
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

    bool updateSleepStateAndCheckIfShouldSleep(
        RigidBody& body,
        const Transform& transform,
        float dt,
        float jitterThreshold,
        float anchorTimerThreshold)
    {
        bool goingToSleep = false;

        if (glm::abs(body.anchorPoint.x - transform.position.x) < jitterThreshold &&
            glm::abs(body.anchorPoint.y - transform.position.y) < jitterThreshold &&
            glm::abs(body.anchorPoint.z - transform.position.z) < jitterThreshold)
        {
            body.anchorTimer += dt;
        }
        else {
            body.anchorTimer = glm::max(0.0f, body.anchorTimer - dt);
        }

        if (body.anchorTimer == 0.0f) {
            body.anchorPoint = transform.position;
        }

        if (body.anchorTimer >= anchorTimerThreshold) {
            goingToSleep = true;
        }

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