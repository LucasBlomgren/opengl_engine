#include "pch.h"

#include "physics/engine/physics_engine_impl.h"

//=====================================
// Public facade and submission: scene-wide commands
//=====================================
void PhysicsEngine::sleepAllObjects() {
    impl->submitSleepAllObjects();
}

void PhysicsEngine::awakenAllObjects() {
    impl->submitAwakenAllObjects();
}

void PhysicsEngine::Impl::submitSleepAllObjects() {
    commandBuffer.recordSleepAllObjects();
}

void PhysicsEngine::Impl::submitAwakenAllObjects() {
    commandBuffer.recordAwakenAllObjects();
}

//=====================================
// Temporary legacy submission
//=====================================
void PhysicsEngine::syncBodyFromTransform(RigidBodyHandle body) {
    impl->submitSyncBodyFromTransform(body);
}

void PhysicsEngine::Impl::submitSyncBodyFromTransform(
    RigidBodyHandle body) {
    commandBuffer.recordSyncBodyFromTransform(body);
}

//=====================================
// BVH management
//=====================================
void PhysicsEngine::setBVHDirty(RigidBodyHandle& handle) {
    impl->setBVHDirty(handle);
}

void PhysicsEngine::Impl::setBVHDirty(
    const RigidBodyHandle& handle) {
    broadphaseManager.setBVHDirty(handle);
}
