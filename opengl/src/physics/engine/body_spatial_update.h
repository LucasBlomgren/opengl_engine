#pragma once

namespace physics::internal {

class RigidBody;
struct RuntimeCaches;

void updateCollidersAndBodyAABB(
    RuntimeCaches& caches,
    RigidBody* body);

}
