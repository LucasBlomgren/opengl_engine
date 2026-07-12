#include "pch.h"
#include "physics_engine.h"

//========================================
//   Build pairs for a physics scope
//========================================
void PhysicsEngine::buildPairsForScope(
    PhysicsScope& scope,
    PairBatch& pairs)
{
    pairs.clear();

    if (stepMode == StepMode::Global) {
        // In global mode, we can use the broadphase manager to build pairs for the entire world
        broadphaseManager.buildGlobalPairs(pairs);
        return;
    }

    // Bodies inside this scope against each other
    if (scope.bodies.size() <= 16) {
        broadphaseManager.buildPairsBruteForce(
            scope.bodies,
            pairs.dynamicPairs
        );
    }
    else {
        broadphaseManager.querySAP(
            scope.internalSap,
            pairs.dynamicPairs
        );
    }

    // Rest bodies must also be tested against sleeping bodies
    if (scope.type == PhysicsScopeType::Rest) {
        broadphaseManager.buildAsleepPairsForScope(
            scope.bodies,
            pairs.dynamicPairs
        );
    }

    // Scope bodies vs terrain
    broadphaseManager.buildTerrainPairsForScope(
        scope.bodies,
        pairs.terrainPairs
    );

    // Scope bodies vs static rigid bodies
    broadphaseManager.buildStaticPairsForScope(
        scope.bodies,
        pairs.dynamicPairs
    );
}