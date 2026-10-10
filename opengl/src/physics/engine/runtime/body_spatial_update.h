#pragma once

namespace physics::internal {

class RigidBody;

void updateCollidersAndBodyAABB(
    PhysicsStorage& storage,
    RigidBody* body);

} // namespace physics::internal
