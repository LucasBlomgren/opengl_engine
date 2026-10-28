#pragma once

#include "physics/storage/slot_map.h"

#include "physics/bodies/rigidbody.h"
#include "physics/bvh/bvh.h"
#include "physics/public/query_types.h"

namespace physics::internal {

namespace raycast {
RaycastHit raycastTree(
   PhysicsStorage& storage,
   const Ray& ray,
   const BVHTree& tree,
   BodyHandle ignoredBody
);
}

}
