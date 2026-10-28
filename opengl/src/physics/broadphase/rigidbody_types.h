#pragma once

#include <cstdint>

namespace physics::internal {

// Broadphase bucket types
enum class BroadphaseBucket : uint8_t {
   None,
   Awake,
   Asleep,
   Static
};

// Handle for broadphase sleep state management
struct BroadphaseHandle {
   BroadphaseBucket bucket = BroadphaseBucket::None;
   int listIdx = -1;
   int leafIdx = -1;
};

}
