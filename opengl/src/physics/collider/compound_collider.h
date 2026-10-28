#pragma once

#include <vector>

#include "physics/public/handles.h"
#include "physics/collider/aabb.h"

namespace physics::internal {

struct Compound {
   std::vector<ColliderHandle> colliders;
   AABB aabb;

   Compound() = default;

   Compound(
      std::vector<ColliderHandle>& handles)
   : colliders(handles) {}
};

}