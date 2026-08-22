#pragma once

namespace physics::internal {

class RigidBody;

void updateCollidersAndBodyAABB(
    PhysicsWorld& world,
    RigidBody* body);

} // namespace physics::internal
