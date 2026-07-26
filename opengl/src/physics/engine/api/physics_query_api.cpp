#include "pch.h"

#include "physics/public/physics_engine.h"
#include "physics/engine/physics_engine_impl.h"

//=========================================
// State queries
//=========================================
std::optional<RigidBodyState> PhysicsEngine::getRigidBodyState(
    RigidBodyHandle body) const {
    return impl->getRigidBodyState(body);
}

std::optional<ColliderState> PhysicsEngine::getColliderState(
    ColliderHandle collider) const {
    return impl->getColliderState(collider);
}

//=========================================
// Spatial queries
//=========================================
Raycast::RaycastHit PhysicsEngine::raycast(
    Raycast::Ray& ray) {
    return impl->raycast(ray);
}

//=========================================
// Simulation output
//=========================================
std::vector<ExternalMotionContact>&
PhysicsEngine::getExternalMotionContacts() {
    return impl->getExternalMotionContacts();
}