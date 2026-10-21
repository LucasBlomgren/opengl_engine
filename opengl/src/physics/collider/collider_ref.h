#pragma once

#include <cstdint>

#include "physics/public/handles.h"

namespace physics::internal {

struct BodyColliderRef {
   enum class Type : uint8_t {
      NONE,
      SINGLE,
      COMPOUND
   };

   Type type = Type::NONE;

   union {
      ColliderHandle single;
      CompoundHandle compound;
   };

   BodyColliderRef() : single{} {}

   BodyColliderRef(ColliderHandle h)
      : type(Type::SINGLE), single(h) {}

   BodyColliderRef(CompoundHandle h)
      : type(Type::COMPOUND), compound(h) {}

   bool isSingle() const {
      if (type == Type::SINGLE) return true;
      return false;
   }

   bool isCompound() const {
      if (type == Type::COMPOUND) return true;
      return false;
   }

   bool isValid() const {
      if (type == Type::NONE) {
         return false;
      }
      else if (type == Type::SINGLE) {
         return single.isValid();
      }

      return compound.isValid();
   }
};

}