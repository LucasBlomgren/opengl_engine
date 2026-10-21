#pragma once

#include "physics/storage/physics_storage.h"

namespace physics::internal {

template<typename Func>
void forEachBodyCollider(
   PhysicsStorage& storage,
   BodyColliderRef ref,
   Func&& func)
{
   if (ref.isCompound()) {
      auto& compound = storage.getCompound(ref.compound);

      for (ColliderHandle handle : compound.colliders) {
         func(handle);
      }
   }
   else {
      func(ref.single);
   }
}

}