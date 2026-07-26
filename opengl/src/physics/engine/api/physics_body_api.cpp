#include "pch.h"

#include "physics/public/physics_engine.h"
#include "physics/engine/physics_engine_impl.h"

//=========================================
// Rigid body lifecycle
//=========================================
RigidBodyHandle PhysicsEngine::createRigidBody(
    const RigidBodyDesc& desc) {
    return impl->submitCreateRigidBody(desc);
}

bool PhysicsEngine::destroyRigidBody(
    RigidBodyHandle body) {
    return impl->submitDestroyRigidBody(body);
}

//=========================================
// Rigid body commands
//=========================================
bool PhysicsEngine::applyLinearImpulse(
    RigidBodyHandle body,
    const glm::vec3& impulse) {
    return impl->submitApplyLinearImpulse(body, impulse);
}

bool PhysicsEngine::setLinearVelocity(
    RigidBodyHandle body,
    const glm::vec3& velocity) {
    return impl->submitSetLinearVelocity(body, velocity);
}

bool PhysicsEngine::setAngularVelocity(
    RigidBodyHandle body,
    const glm::vec3& velocity) {
    return impl->submitSetAngularVelocity(body, velocity);
}

bool PhysicsEngine::setKinematicTarget(
    RigidBodyHandle body,
    const PhysicsPose& target) {
    return impl->submitSetKinematicTarget(body, target);
}

bool PhysicsEngine::setRigidBodyAwake(
    RigidBodyHandle body) {
    return impl->submitSetRigidBodyAwake(body, true);
}

bool PhysicsEngine::setRigidBodyAsleep(
    RigidBodyHandle body) {
    return impl->submitSetRigidBodyAwake(body, false);
}

bool PhysicsEngine::setRigidBodyType(
    RigidBodyHandle body,
    BodyType type) {
    return impl->submitSetRigidBodyType(body, type);
}

bool PhysicsEngine::setRigidBodyMotionControl(
    RigidBodyHandle body,
    MotionControl motionControl) {
    return impl->submitSetRigidBodyMotionControl(body, motionControl);
}
