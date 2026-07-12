#include "pch.h"
#include "narrowphase_manager.h"

bool NarrowphaseManager::trySpeculativeBoxBox(
    ContactBuildInput& in,
    DynamicContactCandidate& out,
    float dt)
{
    return false;
}

bool NarrowphaseManager::trySpeculativeBoxSphere(
    ContactBuildInput& in,
    DynamicContactCandidate& out,
    float dt)
{
    return false;
}

bool NarrowphaseManager::trySpeculativeSphereSphere(
    ContactBuildInput& in,
    DynamicContactCandidate& out,
    float dt)
{
    return false;
}