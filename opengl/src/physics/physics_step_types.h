#pragma once

#include <vector>
#include <cstdint>
#include "slot_map.h"
#include "core/pointer_cache.h"
#include "colliders/collider.h"
#include "rigidbody.h"

struct RuntimeCaches {
    PointerCache<Transform, TransformHandle> transforms;
    PointerCache<Collider, ColliderHandle> colliders;
    PointerCache<RigidBody, RigidBodyHandle> bodies;

    void clear() {
        transforms.clear();
        colliders.clear();
        bodies.clear();
    }
};

enum class StepScopeType {
    Global,
    Island,
    Rest
};

enum class IslandBroadphaseMode {
    BruteForce,
    SAP
};

struct StepScope {
    StepScopeType type = StepScopeType::Global;

    const std::vector<RigidBodyHandle>* bodies = nullptr;

    uint32_t islandId = 0;
    IslandBroadphaseMode islandBroadphaseMode = IslandBroadphaseMode::BruteForce;
};