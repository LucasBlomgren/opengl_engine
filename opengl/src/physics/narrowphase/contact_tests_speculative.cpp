#include "narrowphase_manager.h"

namespace physics::internal {

std::optional<SweepHit> NarrowphaseManager::trySpeculativeBoxBox(
    ResolvedColliderPair pair,
    float dt,
    BodyHandle sweepOwner)
{
    SAT::Result geometry{};
    if (!SAT::speculativeBoxBox(
        *pair.a.collider,
        *pair.b.collider,
        *pair.a.body,
        *pair.b.body,
        dt,
        geometry))
    {
        return std::nullopt;
    }

    return SweepHit{
        std::move(pair),
        std::move(geometry),
        sweepOwner
    };
}

std::optional<SweepHit> NarrowphaseManager::trySpeculativeBoxSphere(
    ResolvedColliderPair pair,
    float dt,
    BodyHandle sweepOwner)
{
    if (pair.a.collider->type != ColliderType::CUBOID) {
        pair.swapAB();
    }

    SAT::Result geometry{};
    if (!SAT::speculativeBoxSphere(
        *pair.a.collider,
        *pair.b.collider,
        *pair.a.body,
        *pair.b.body,
        dt,
        geometry))
    {
        return std::nullopt;
    }

    return SweepHit{
        std::move(pair),
        std::move(geometry),
        sweepOwner
    };
}

std::optional<TerrainSweepHit> NarrowphaseManager::trySpeculativeBoxTriangle(
    ColliderEndpointRef collider,
    Tri* tri,
    float dt,
    BodyHandle sweepOwner)
{
    if (!tri) {
        return std::nullopt;
    }

    SAT::Result geometry{};
    if (!SAT::speculativeBoxTriangle(
        *collider.collider,
        *collider.body,
        *tri,
        dt,
        geometry))
    {
        return std::nullopt;
    }

    return TerrainSweepHit{
        collider,
        std::move(geometry),
        sweepOwner
    };
}

std::optional<SweepHit> NarrowphaseManager::trySpeculativeSphereSphere(
    ResolvedColliderPair pair,
    float dt,
    BodyHandle sweepOwner)
{
    SAT::Result geometry{};
    if (!SAT::speculativeSphereSphere(
        *pair.a.collider,
        *pair.b.collider,
        *pair.a.body,
        *pair.b.body,
        dt,
        geometry))
    {
        return std::nullopt;
    }

    return SweepHit{
        std::move(pair),
        std::move(geometry),
        sweepOwner
    };
}

std::optional<TerrainSweepHit> NarrowphaseManager::trySpeculativeSphereTriangle(
    ColliderEndpointRef collider,
    Tri* tri,
    float dt,
    BodyHandle sweepOwner)
{
    SAT::Result geometry{};
    if (!SAT::speculativeSphereTriangle(
        *collider.collider,
        *collider.body,
        *tri,
        dt,
        geometry))
    {
        return std::nullopt;
    }

    return TerrainSweepHit{
        collider,
        std::move(geometry),
        sweepOwner
    };
}

}
