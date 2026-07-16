#pragma once
#include "narrowphase_manager.h"

//=======================================================
//     Box-Box
//=======================================================
bool NarrowphaseManager::tryBoxBox(
    ContactBuildInput& in,
    DynamicContactCandidate& out)
{
    if (in.colliderA->id > in.colliderB->id) {
        in.swapAB();
    }

    if (!SAT::boxBox(*in.colliderA, *in.colliderB, out.sat)) {
        return false;
    }

    glm::vec3 centerA = std::get<OOBB>(in.colliderA->shape).worldCenter;
    glm::vec3 centerB = std::get<OOBB>(in.colliderB->shape).worldCenter;
    SAT::reverseNormal(centerA, centerB, out.sat.normal);

    out.manifoldType = ManifoldType::BoxBox;
    return true;
}

//=======================================================
//     Box-Sphere
//=======================================================
bool NarrowphaseManager::tryBoxSphere(
    ContactBuildInput& in,
    DynamicContactCandidate& out)
{
    if (in.colliderA->type != ColliderType::CUBOID) {
        in.swapAB();
    }

    in.colliderA->pose.ensureInvModelMatrix();

    if (!SAT::boxSphere(*in.colliderA, *in.colliderB, in.colliderA->pose, out.sat)) {
        return false;
    }

    out.manifoldType = ManifoldType::BoxSphere;
    return true;
}

//=======================================================
//     Sphere-Sphere
//=======================================================
bool NarrowphaseManager::trySphereSphere(
    ContactBuildInput& in,
    DynamicContactCandidate& out)
{
    if (!SAT::sphereSphere(*in.colliderA, *in.colliderB, out.sat)) {
        return false;
    }

    out.manifoldType = ManifoldType::SphereSphere;
    return true;
}