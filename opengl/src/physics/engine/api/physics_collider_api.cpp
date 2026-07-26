#include "pch.h"

#include "physics/public/physics_engine.h"
#include "physics/engine/physics_engine_impl.h"

//=========================================
// Collider lifecycle
//=========================================
ColliderHandle PhysicsEngine::createCollider(
    RigidBodyHandle body,
    const ColliderDesc& desc) {
    return impl->submitCreateCollider(body, desc);
}

bool PhysicsEngine::destroyCollider(
    ColliderHandle collider) {
    return impl->submitDestroyCollider(collider);
}

//=========================================
// Collider commands
//=========================================
bool PhysicsEngine::setColliderLocalPose(
    ColliderHandle collider,
    const PhysicsPose& localPose) {
    return impl->submitSetColliderLocalPose(collider, localPose);
}

bool PhysicsEngine::setColliderEnabled(
    ColliderHandle collider,
    bool enabled) {
    return impl->submitSetColliderEnabled(collider, enabled);
}

bool PhysicsEngine::setColliderTrigger(
    ColliderHandle collider,
    bool isTrigger) {
    return impl->submitSetColliderTrigger(collider, isTrigger);
}
