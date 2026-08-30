#pragma once
#include "narrowphase_manager.h"

namespace physics::internal {

//=======================================================
//     Box-Box
//=======================================================
std::optional<OverlapHit> NarrowphaseManager::tryBoxBox(
    ResolvedColliderPair pair)
{
    if (pair.a.collider->id > pair.b.collider->id) {
        pair.swapAB();
    }

    SAT::Result geometry{};
    if (!SAT::boxBox(*pair.a.collider, *pair.b.collider, geometry)) {
        return std::nullopt;
    }

    glm::vec3 centerA = std::get<OOBB>(pair.a.collider->shape).worldCenter;
    glm::vec3 centerB = std::get<OOBB>(pair.b.collider->shape).worldCenter;
    SAT::reverseNormal(centerA, centerB, geometry.normal);

    return OverlapHit{ std::move(pair), std::move(geometry) };
}

//=======================================================
//     Box-Sphere
//=======================================================
std::optional<OverlapHit> NarrowphaseManager::tryBoxSphere(
    ResolvedColliderPair pair)
{
    if (pair.a.collider->type != ColliderType::CUBOID) {
        pair.swapAB();
    }

    pair.a.collider->transformCache.ensureInvModelMatrix(
        pair.a.collider->worldPose
    );

    SAT::Result geometry{};
    if (!SAT::boxSphere(
        *pair.a.collider,
        *pair.b.collider,
        pair.a.collider->transformCache,
        geometry))
    {
        return std::nullopt;
    }

    return OverlapHit{ std::move(pair), std::move(geometry) };
}

//=======================================================
//     Sphere-Sphere
//=======================================================
std::optional<OverlapHit> NarrowphaseManager::trySphereSphere(
    ResolvedColliderPair pair)
{
    SAT::Result geometry{};
    if (!SAT::sphereSphere(
        *pair.a.collider,
        *pair.b.collider,
        geometry))
    {
        return std::nullopt;
    }

    return OverlapHit{ std::move(pair), std::move(geometry) };
}

}
