#pragma once

#include <cstdint>

enum ColliderStorageType {
    SINGLE,
    COMPOUND
};

struct ColliderRef {
    ColliderStorageType type;

    uint32_t slot;
    uint32_t gen;

    // only used for COMPOUND colliders
    uint32_t childIndex;
};