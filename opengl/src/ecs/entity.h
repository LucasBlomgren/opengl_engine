#pragma once
#include <cstdint>

struct Entity {
    uint32_t id = INVALID;
    uint32_t generation = 0;

    static constexpr uint32_t INVALID = 0xFFFFFFFFu;

    bool isValid() const {
        return id != INVALID;
    }

    bool operator==(const Entity& other) const {
        return id == other.id && generation == other.generation;
    }

    bool operator!=(const Entity& other) const {
        return !(*this == other);
    }
};