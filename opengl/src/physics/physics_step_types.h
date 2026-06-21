#pragma once

#include <vector>
#include <cstdint>
#include "slot_map.h"

using IslandID = uint32_t;

enum class StepScopeType {
    Global,
    Island,
    Rest
};

enum class IslandBroadphaseMode {
    BruteForce,
    SAP
};

struct StepScope {
    StepScopeType type = StepScopeType::Global;

    const std::vector<RigidBodyHandle>* bodies = nullptr;

    IslandID islandId = 0;
    IslandBroadphaseMode islandBroadphaseMode = IslandBroadphaseMode::BruteForce;
};