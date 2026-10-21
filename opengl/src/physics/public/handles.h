#pragma once

#include <cstdint>
#include <functional>

namespace physics {

template<class Tag>
struct HandleT {
   static constexpr uint32_t INVALID = 0xFFFFFFFFu;

   uint32_t slot = INVALID;
   uint32_t gen = INVALID;

   bool isValid() const noexcept {
      return slot != INVALID;
   }

   explicit operator bool() const noexcept {
      return isValid();
   }

   friend bool operator==(const HandleT& a, const HandleT& b) noexcept {
      return a.slot == b.slot && a.gen == b.gen;
   }
};

struct BodyTag;
struct MotionStateTag;
struct SleepStateTag;
struct ColliderTag;
struct CompoundTag;
struct ShapeTag;

using BodyHandle = HandleT<BodyTag>;
using MotionStateHandle = HandleT<MotionStateTag>;
using SleepStateHandle = HandleT<SleepStateTag>;
using ColliderHandle = HandleT<ColliderTag>;
using CompoundHandle = HandleT<CompoundTag>;
using ShapeHandle = HandleT<ShapeTag>;

} // namespace physics


template<class Tag>
struct std::hash<physics::HandleT<Tag>> {
   size_t operator()(const physics::HandleT<Tag>& handle) const noexcept {
      uint64_t value = (uint64_t(handle.gen) << 32) | uint64_t(handle.slot);
      return std::hash<uint64_t>{}(value);
   }
};