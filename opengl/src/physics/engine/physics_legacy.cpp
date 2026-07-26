#include "pch.h"

#include "physics/engine/physics_engine_impl.h"

//=====================================
// Temporary legacy implementation
//=====================================
PhysicsWorld* PhysicsEngine::Impl::getPhysicsWorld() {
    return &physicsWorld;
}
