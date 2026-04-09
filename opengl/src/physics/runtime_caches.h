#pragma once

#include <vector>
#include "core/pointer_cache.h"
#include "core/slot_map.h"
#include "game/transform.h"
#include "physics/colliders/collider.h"
#include "physics/rigid_body.h"

struct RuntimeCaches {
    PointerCache<Transform, TransformHandle> transforms;
    PointerCache<Collider, ColliderHandle> colliders;
    PointerCache<RigidBody, RigidBodyHandle> bodies;
    std::vector<Transform> globalColliderTransforms;

    void clear() {
        transforms.clear();
        colliders.clear();
        bodies.clear();
        globalColliderTransforms.clear();
    }
};