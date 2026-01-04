#pragma once
#include <cstdint>

// Broadphase bucket types
enum class BroadphaseBucket : uint8_t { None, Awake, Asleep, Static };

// Handle for broadphase BVH management
struct BroadphaseHandle {
    BroadphaseBucket bucket = BroadphaseBucket::None;
    int listIdx = -1;
    int leafIdx = -1;
};