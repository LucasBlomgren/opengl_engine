#include "pch.h"

#include "physics/public/physics_engine.h"
#include "physics/engine/physics_engine_impl.h"

//=====================================
// Temporary legacy facade
//=====================================
PhysicsWorld* PhysicsEngine::getPhysicsWorld() {
    return impl->getPhysicsWorld();
}

void PhysicsEngine::syncBodyFromTransform(
    RigidBodyHandle body) {
    impl->submitSyncBodyFromTransform(body);
}

void PhysicsEngine::setBVHDirty(
    RigidBodyHandle& handle) {
    impl->setBVHDirty(handle);
}
