#include "pch.h"

#include "physics/public/physics_engine.h"
#include "physics/engine/physics_engine_impl.h"

namespace physics {

//=========================================
// State queries
//=========================================
std::optional<BodyState> Engine::getRigidBodyState(
    BodyHandle body) const {
    return impl->getRigidBodyState(body);
}

std::optional<ColliderState> Engine::getColliderState(
    ColliderHandle collider) const {
    return impl->getColliderState(collider);
}

//=========================================
// Spatial queries
//=========================================
RaycastHit Engine::raycast(
    const Ray& ray,
    BodyHandle ignoredBody) {
    return impl->raycast(ray, ignoredBody);
}

std::vector<BodyHandle> Engine::queryBodies(
    const physics::AABB& bounds,
    BodySet bodySet) const {
    return impl->queryBodies(bounds, bodySet);
}

//=========================================
// Simulation output
//=========================================
std::vector<ExternalMotionContact>&
Engine::getExternalMotionContacts() {
    return impl->getExternalMotionContacts();
}

}
