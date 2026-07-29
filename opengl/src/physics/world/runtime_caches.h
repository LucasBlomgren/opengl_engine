#pragma once

#include "core/pointer_cache.h"

#include "physics/bodies/rigidbody.h"
#include "physics/colliders/collider.h"

namespace physics::internal {

struct RuntimeCaches {
    PointerCache<Collider, ColliderHandle> colliders;
    PointerCache<RigidBody, BodyHandle> bodies;

    void clear() {
        colliders.clear();
        bodies.clear();
    }
};

}
