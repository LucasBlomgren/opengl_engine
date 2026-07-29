#include "pch.h"

#include "physics/public/physics_engine.h"
#include "physics/engine/physics_engine_impl.h"

namespace physics {

//=========================================
// Collider lifecycle
//=========================================
ColliderHandle Engine::createCollider(
    BodyHandle body,
    const ColliderDesc& desc) {
    return impl->submitCreateCollider(body, desc);
}

bool Engine::destroyCollider(
    ColliderHandle collider) {
    return impl->submitDestroyCollider(collider);
}

//=========================================
// Collider commands
//=========================================
bool Engine::setColliderLocalPose(
    ColliderHandle collider,
    const Pose& localPose) {
    return impl->submitSetColliderLocalPose(collider, localPose);
}

bool Engine::setColliderLocalTransform(
    ColliderHandle collider,
    const Pose& localPose,
    const glm::vec3& localScale) {
    return impl->submitSetColliderLocalTransform(
        collider,
        localPose,
        localScale
    );
}

bool Engine::setColliderShape(
    ColliderHandle collider,
    const ColliderShapeDesc& shape) {
    return impl->submitSetColliderShape(collider, shape);
}

bool Engine::setColliderEnabled(
    ColliderHandle collider,
    bool enabled) {
    return impl->submitSetColliderEnabled(collider, enabled);
}

bool Engine::setColliderTrigger(
    ColliderHandle collider,
    bool isTrigger) {
    return impl->submitSetColliderTrigger(collider, isTrigger);
}

}
