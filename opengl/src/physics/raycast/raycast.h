#pragma once

#include "core/slot_map.h"

#include "physics/bodies/rigidbody.h"
#include "physics/bvh/bvh.h"
#include "physics/public/physics_query_types.h"

namespace physics::internal {

namespace raycast {
    RaycastHit raycastTree(
        const Ray& ray,
        const BVHTree& tree,
        const SlotMap<RigidBody, BodyHandle>& bodyMap,
        BodyHandle ignoredBody
    );
}

}
