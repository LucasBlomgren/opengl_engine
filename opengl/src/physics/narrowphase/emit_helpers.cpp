#include "narrowphase_manager.h"

//=======================================================
//     Compute contributes motion
//=======================================================
bool NarrowphaseManager::computeContributesMotion(
    ContactPartnerType partnerType,
    const RigidBody& body,
    bool willWake) const
{
    return
        partnerType == ContactPartnerType::RigidBody &&
        (
            body.type == BodyType::Kinematic ||
            (body.type == BodyType::Dynamic && (!body.asleep || willWake))
            );
}

bool NarrowphaseManager::computeNoSolverResponse(
    const RigidBody& body,
    bool willWake) const
{
    return
        body.motionControl == MotionControl::External ||
        body.type == BodyType::Static ||
        body.type == BodyType::Kinematic ||
        (body.type == BodyType::Dynamic && body.asleep && !willWake);
}