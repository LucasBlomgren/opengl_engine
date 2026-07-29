#include "pch.h"

#include "physics/public/physics_engine.h"
#include "physics/engine/physics_engine_impl.h"

namespace physics {

//=========================================
// Rigid body lifecycle
//=========================================
BodyHandle Engine::createRigidBody(
    const BodyDesc& desc) {
    return impl->submitCreateRigidBody(desc);
}

bool Engine::destroyRigidBody(
    BodyHandle body) {
    return impl->submitDestroyRigidBody(body);
}

//=========================================
// Rigid body commands
//=========================================
bool Engine::applyLinearImpulse(
    BodyHandle body,
    const glm::vec3& impulse) {
    return impl->submitApplyLinearImpulse(body, impulse);
}

bool Engine::setLinearVelocity(
    BodyHandle body,
    const glm::vec3& velocity) {
    return impl->submitSetLinearVelocity(body, velocity);
}

bool Engine::setAngularVelocity(
    BodyHandle body,
    const glm::vec3& velocity) {
    return impl->submitSetAngularVelocity(body, velocity);
}

bool Engine::setKinematicTarget(
    BodyHandle body,
    const Pose& target) {
    return impl->submitSetKinematicTarget(body, target);
}

bool Engine::setRigidBodyTransform(
    BodyHandle body,
    const Pose& pose,
    const glm::vec3& scale) {
    return impl->submitSetRigidBodyTransform(body, pose, scale);
}

bool Engine::setRigidBodySleepState(
    BodyHandle body,
    bool asleep) {
    return impl->submitSetRigidBodySleepState(body, asleep);
}

bool Engine::setRigidBodyType(
    BodyHandle body,
    BodyType type) {
    return impl->submitSetRigidBodyType(body, type);
}

bool Engine::setRigidBodyMotionControl(
    BodyHandle body,
    MotionControl motionControl) {
    return impl->submitSetRigidBodyMotionControl(body, motionControl);
}

bool Engine::setRigidBodyResponseMode(
    BodyHandle body,
    ResponseMode responseMode) {
    return impl->submitSetRigidBodyResponseMode(body, responseMode);
}

bool Engine::setRigidBodyMass(
    BodyHandle body,
    float mass) {
    return impl->submitSetRigidBodyMass(body, mass);
}

bool Engine::setRigidBodyAllowGravity(
    BodyHandle body,
    bool allowGravity) {
    return impl->submitSetRigidBodyAllowGravity(body, allowGravity);
}

bool Engine::setRigidBodyAllowSleep(
    BodyHandle body,
    bool allowSleep) {
    return impl->submitSetRigidBodyAllowSleep(body, allowSleep);
}

bool Engine::setRigidBodyCanMoveLinearly(
    BodyHandle body,
    bool canMoveLinearly) {
    return impl->submitSetRigidBodyCanMoveLinearly(
        body,
        canMoveLinearly
    );
}

}
