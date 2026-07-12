#pragma once

#include <cstdint>
#include <vector>

#include "broadphase/sweep_and_prune.h"

enum class PhysicsScopeType {
    Island,
    Rest
};

struct PhysicsScope {
    PhysicsScopeType type = PhysicsScopeType::Island;

    uint32_t id = 0;
    int substeps = 1;

    std::vector<RigidBodyHandle> bodies;

    // Bodies in this scope against each other.
    sap::SweepAndPrune internalSap;
    bool internalSapBuilt = false;

    // Only used by restScope.
    sap::SweepAndPrune vsAsleepSap;
    bool vsAsleepSapBuilt = false;
};