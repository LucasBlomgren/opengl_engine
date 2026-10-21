#pragma once

#include "physics/public/handles.h"

namespace physics::internal {

struct BVHLeafRef {
   BodyHandle body;
   ColliderHandle collider;
};

}

template<>
struct std::hash<physics::internal::BVHLeafRef> {
   size_t operator()(
      const physics::internal::BVHLeafRef& ref
      ) const noexcept {
      size_t h1 = std::hash<physics::BodyHandle>{}(ref.body);
      size_t h2 = std::hash<physics::ColliderHandle>{}(ref.collider);

      return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL
         + (h1 << 6) + (h1 >> 2));
   }
};