#include "physics_engine.h"

//========================================
//   Build pairs for a given step scope
//========================================
void PhysicsEngine::buildPairsForScope(
    const StepScope& scope,
    PairBatch& pairs)
{
    pairs.clear();

    switch (scope.type) {
    case StepScopeType::Island:
        buildIslandPairs(*scope.bodies, pairs);
        break;

    case StepScopeType::Rest:
        buildRestPairs(*scope.bodies, pairs);
        break;
    }
}

//========================================
//   Build pairs for a given island
//========================================
void PhysicsEngine::buildIslandPairs(
    const std::vector<RigidBodyHandle>& bodies,
    PairBatch& pairs)
{
    // dynamic-vs-dynamic intra island
    if (bodies.size() <= 16) {
        broadphaseManager.buildPairsBruteForce(bodies, pairs.dynamicPairs);
    }
    else {
        broadphaseManager.buildPairsSAP(bodies, pairs.dynamicPairs);
    }

    // island bodies vs terrain
    broadphaseManager.buildTerrainPairsForScope(bodies, pairs.terrainPairs);

    // island bodies vs static bodies
    broadphaseManager.buildStaticPairsForScope(bodies, pairs.dynamicPairs);
}

//========================================
//   Build pairs for rest bodies
//========================================
void PhysicsEngine::buildRestPairs(
    const std::vector<RigidBodyHandle>& bodies,
    PairBatch& pairs)
{
    // dynamic vs awake bodies
    broadphaseManager.buildPairsSAP(bodies, pairs.dynamicPairs);

    // dynamic vs asleep bodies
    broadphaseManager.buildPairsSAPTwoSets(
        bodies,
        broadphaseManager.getAsleepList(),
        pairs.dynamicPairs
    );

    // rest bodies vs terrain
    broadphaseManager.buildTerrainPairsForScope(bodies, pairs.terrainPairs);

    // rest bodies vs static bodies
    broadphaseManager.buildStaticPairsForScope(bodies, pairs.dynamicPairs);
}