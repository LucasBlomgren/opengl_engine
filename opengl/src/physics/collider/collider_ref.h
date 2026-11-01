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

   bool isNone() const {
      return type == Type::NONE ? true : false;
   }
   bool isSingle() const {
      return type == Type::SINGLE ? true : false;
   }
   bool isCompound() const {
      return type == Type::COMPOUND ? true : false;
   }
};

}