#include "narrowphase_manager.h"

bool NarrowphaseManager::trySpeculativeBoxBox(
    ContactBuildInput& in,
    DynamicContactCandidate& out,
    float dt)
{
    return false;
}

bool NarrowphaseManager::trySpeculativeBoxSphere(
    ContactBuildInput& in,
    DynamicContactCandidate& out,
    float dt)
{
    return false;
}

bool NarrowphaseManager::trySpeculativeSphereSphere(
    ContactBuildInput& in,
    DynamicContactCandidate& out,
    float dt)
{
    if (!SAT::speculativeSphereSphere(
        *in.colliderA,
        *in.colliderB,
        *in.bodyA,
        *in.bodyB,
        dt,
        out.sat))
    {
        return false;
    }

    out.manifoldType = ManifoldType::SphereSphere;
    out.partnerTypeA = ContactPartnerType::RigidBody;
    out.partnerTypeB = ContactPartnerType::RigidBody;

    return true;
}