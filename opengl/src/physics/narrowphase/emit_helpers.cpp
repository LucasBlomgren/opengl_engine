#include "narrowphase_manager.h"

namespace physics::internal {

bool NarrowphaseManager::computeContributesMotion(
    ContactPartnerType partnerType,
    const RigidBody& body,
    bool willWake) const
{
    if (partnerType == ContactPartnerType::Terrain ||
        body.type == BodyType::Static) {
        return false;
    }

    if (body.type == BodyType::Kinematic) {
        return true;
    }

    if (body.type == BodyType::Dynamic) {
        SleepState& sleepState = physicsWorld->getSleepState(body.sleepStateHandle);

        if (!sleepState.asleep || willWake) {
            return true;
        }
    }

    return false;
}

bool NarrowphaseManager::computeNoSolverResponse(
    const RigidBody& body,
    bool willWake) const
{
    if (body.motionControl == MotionControl::External ||
        body.type == BodyType::Static ||
        body.type == BodyType::Kinematic)
        return true;


    if (body.type == BodyType::Dynamic) {
        SleepState& sleepState = physicsWorld->getSleepState(body.sleepStateHandle);

        if (sleepState.asleep && !willWake) {
            return true;
        }
    }

    return false;
}
}