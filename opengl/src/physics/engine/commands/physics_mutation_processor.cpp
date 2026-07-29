#include "pch.h"

#include <type_traits>
#include <variant>

#include "physics/engine/physics_engine_impl.h"

namespace physics::internal {


//================================================
// Helper functions
//================================================
namespace {
    Collider* findFirstEnabledCollider(
        PhysicsWorld& physicsWorld,
        const RigidBody& body)
    {
        for (ColliderHandle colliderHandle : body.colliderHandles) {
            if (!physicsWorld.isColliderActive(colliderHandle)) {
                continue;
            }

            Collider* collider = physicsWorld.getCollider(colliderHandle);

            if (collider && collider->enabled) {
                return collider;
            }
        }

        return nullptr;
    }

    bool applyShape(
        Collider& collider,
        const ColliderShapeDesc& shape)
    {
        bool validShape = true;

        std::visit([&](const auto& shapeDesc) {
            using ShapeDescType = std::decay_t<decltype(shapeDesc)>;

            if constexpr (std::is_same_v<ShapeDescType, BoxShapeDesc>) {
                collider.type = ColliderType::CUBOID;
                collider.shape = OOBB(
                    shapeDesc.halfExtents,
                    shapeDesc.center
                );
            }
            else if constexpr (std::is_same_v<ShapeDescType, SphereShapeDesc>) {
                collider.type = ColliderType::SPHERE;
                collider.shape = Sphere(
                    shapeDesc.radius,
                    shapeDesc.center
                );
            }
            else {
                validShape = false;
            }
        }, shape);

        return validShape;
    }
}

void EngineImpl::refreshBodyInertia(
    RigidBody& body)
{
    if (body.type != BodyType::Dynamic) {
        body.invInertiaLocal = glm::mat3(0.0f);
        body.invInertiaWorld = glm::mat3(0.0f);
        return;
    }

    Collider* collider = findFirstEnabledCollider(
        physicsWorld,
        body
    );

    if (!collider) {
        body.invInertiaLocal = glm::mat3(0.0f);
        body.invInertiaWorld = glm::mat3(0.0f);
        return;
    }

    glm::vec3 inertiaScale = collider->worldScale;

    if (collider->type == ColliderType::SPHERE) {
        const Sphere& sphere = std::get<Sphere>(collider->shape);
        inertiaScale = glm::vec3(sphere.radiusWorld * 2.0f);
    }

    body.calculateInverseInertia(
        collider->type,
        *collider,
        inertiaScale
    );
    body.updateInertiaWorld();
}

void EngineImpl::refreshBodySpatialState(
    BodyHandle bodyHandle,
    bool shouldRefreshInertia)
{
    RigidBody* body = physicsWorld.getRigidBody(bodyHandle);

    if (!body || !physicsWorld.isRigidBodyActive(bodyHandle)) {
        return;
    }

    updateCollidersAndBodyAABB(body);

    if (shouldRefreshInertia) {
        refreshBodyInertia(*body);
    }

    const bool hasEnabledCollider =
        findFirstEnabledCollider(physicsWorld, *body) != nullptr;

    if (!hasEnabledCollider) {
        if (body->broadphaseHandle.bucket != BroadphaseBucket::None) {
            broadphaseManager.remove(bodyHandle);
        }

        return;
    }

    if (body->broadphaseHandle.bucket == BroadphaseBucket::None) {
        BroadphaseBucket bucket = BroadphaseBucket::Awake;

        if (body->type == BodyType::Static) {
            bucket = BroadphaseBucket::Static;
        }
        else if (
            body->type == BodyType::Dynamic &&
            body->asleep &&
            body->motionControl != MotionControl::External) {
            bucket = BroadphaseBucket::Asleep;
        }

        broadphaseManager.add(bodyHandle, bucket);
        return;
    }

    setBVHDirty(bodyHandle);
}

//================================================
// Command processing
//================================================
void EngineImpl::applyMutationCommands(
    const std::vector<PhysicsCommandBuffer::Mutation>& mutations)
{
    for (const PhysicsCommandBuffer::Mutation& mutation : mutations) {
        std::visit([this](const auto& command) {
            applyCommand(command);
        }, mutation);
    }
}

//================================================
// Rigid body command processing
//================================================
void EngineImpl::applyCommand(
    const PhysicsCommandBuffer::ApplyLinearImpulse& command)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (!body ||
        !physicsWorld.isRigidBodyActive(command.body) ||
        body->type != BodyType::Dynamic) {
        return;
    }

    if (body->asleep) {
        broadphaseManager.moveToAwake(command.body);
    }

    body->applyImpulseLinear(command.impulse);
}

void EngineImpl::applyCommand(
    const PhysicsCommandBuffer::SetLinearVelocity& command)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (!body ||
        !physicsWorld.isRigidBodyActive(command.body) ||
        body->type == BodyType::Static) {
        return;
    }

    body->linearVelocity = command.velocity;
}

void EngineImpl::applyCommand(
    const PhysicsCommandBuffer::SetAngularVelocity& command)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (!body ||
        !physicsWorld.isRigidBodyActive(command.body) ||
        body->type == BodyType::Static) {
        return;
    }

    body->angularVelocity = command.velocity;
}

void EngineImpl::applyCommand(
    const PhysicsCommandBuffer::SetKinematicTarget& command)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (!body ||
        !physicsWorld.isRigidBodyActive(command.body) ||
        body->type != BodyType::Kinematic) {
        return;
    }

    body->pose = command.target;
    body->updateInertiaWorld();
    refreshBodySpatialState(command.body, false);
}

void EngineImpl::applyCommand(
    const PhysicsCommandBuffer::SetRigidBodyTransform& command)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (!body || !physicsWorld.isRigidBodyActive(command.body)) {
        return;
    }

    body->pose = command.pose;
    body->scale = command.scale;

    refreshBodySpatialState(command.body);
}

void EngineImpl::applyCommand(
    const PhysicsCommandBuffer::SetRigidBodySleepState& command)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (!body || !physicsWorld.isRigidBodyActive(command.body)) {
        return;
    }

    if (command.asleep) {
        body->setAsleep();
        broadphaseManager.moveToAsleep(command.body);
    }
    else {
        body->setAwake();
        broadphaseManager.moveToAwake(command.body);
    }
}

void EngineImpl::applyCommand(
    const PhysicsCommandBuffer::SetRigidBodyType& command)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (!body || !physicsWorld.isRigidBodyActive(command.body)) {
        return;
    }

    body->type = command.type;

    switch (command.type) {
    case BodyType::Dynamic:
        if (body->mass <= 0.0f) {
            body->mass = 1.0f;
        }

        body->invMass = 1.0f / body->mass;

        if (body->sleepCounterThreshold <= 0.0f) {
            body->sleepCounterThreshold = 1.5f;
        }

        refreshBodyInertia(*body);

        if (body->allowSleep && body->asleep) {
            body->setAsleep();
            broadphaseManager.moveToAsleep(command.body);
        }
        else {
            body->setAwake();
            broadphaseManager.moveToAwake(command.body);
        }

        break;

    case BodyType::Kinematic:
        body->invMass = 0.0f;
        body->invInertiaLocal = glm::mat3(0.0f);
        body->invInertiaWorld = glm::mat3(0.0f);
        body->linearVelocity = glm::vec3(0.0f);
        body->angularVelocity = glm::vec3(0.0f);
        body->setAwake();
        broadphaseManager.moveToAwake(command.body);
        break;

    case BodyType::Static:
        body->mass = 0.0f;
        body->invMass = 0.0f;
        body->invInertiaLocal = glm::mat3(0.0f);
        body->invInertiaWorld = glm::mat3(0.0f);
        body->linearVelocity = glm::vec3(0.0f);
        body->angularVelocity = glm::vec3(0.0f);
        broadphaseManager.moveToStatic(command.body);
        break;
    }
}

void EngineImpl::applyCommand(
    const PhysicsCommandBuffer::SetRigidBodyMotionControl& command)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (!body || !physicsWorld.isRigidBodyActive(command.body)) {
        return;
    }

    body->setExternalControl(
        command.motionControl == MotionControl::External
    );
}

void EngineImpl::applyCommand(
    const PhysicsCommandBuffer::SetRigidBodyResponseMode& command)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (!body || !physicsWorld.isRigidBodyActive(command.body)) {
        return;
    }

    body->responseMode = command.responseMode;
}

void EngineImpl::applyCommand(
    const PhysicsCommandBuffer::SetRigidBodyMass& command)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (!body ||
        !physicsWorld.isRigidBodyActive(command.body) ||
        body->type != BodyType::Dynamic ||
        command.mass <= 0.0f) {
        return;
    }

    body->mass = command.mass;
    body->invMass = 1.0f / command.mass;
    refreshBodyInertia(*body);
}

void EngineImpl::applyCommand(
    const PhysicsCommandBuffer::SetRigidBodyAllowGravity& command)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (body && physicsWorld.isRigidBodyActive(command.body)) {
        body->allowGravity = command.allowGravity;
    }
}

void EngineImpl::applyCommand(
    const PhysicsCommandBuffer::SetRigidBodyAllowSleep& command)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (body && physicsWorld.isRigidBodyActive(command.body)) {
        body->allowSleep = command.allowSleep;
    }
}

void EngineImpl::applyCommand(
    const PhysicsCommandBuffer::SetRigidBodyCanMoveLinearly& command)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (body && physicsWorld.isRigidBodyActive(command.body)) {
        body->canMoveLinearly = command.canMoveLinearly;
    }
}

//================================================
// Collider command processing
//================================================
void EngineImpl::applyCommand(
    const PhysicsCommandBuffer::SetColliderLocalPose& command)
{
    Collider* collider = physicsWorld.getCollider(command.collider);

    if (!collider || !physicsWorld.isColliderActive(command.collider)) {
        return;
    }

    collider->localPose = command.localPose;
    refreshBodySpatialState(collider->rigidBodyHandle);
}

void EngineImpl::applyCommand(
    const PhysicsCommandBuffer::SetColliderLocalTransform& command)
{
    Collider* collider = physicsWorld.getCollider(command.collider);

    if (!collider || !physicsWorld.isColliderActive(command.collider)) {
        return;
    }

    collider->localPose = command.localPose;
    collider->localScale = command.localScale;
    refreshBodySpatialState(collider->rigidBodyHandle);
}

void EngineImpl::applyCommand(
    const PhysicsCommandBuffer::SetColliderShape& command)
{
    Collider* collider = physicsWorld.getCollider(command.collider);

    if (!collider ||
        !physicsWorld.isColliderActive(command.collider) ||
        !applyShape(*collider, command.shape)) {
        return;
    }

    refreshBodySpatialState(collider->rigidBodyHandle);
}

void EngineImpl::applyCommand(
    const PhysicsCommandBuffer::SetColliderEnabled& command)
{
    Collider* collider = physicsWorld.getCollider(command.collider);

    if (!collider || !physicsWorld.isColliderActive(command.collider)) {
        return;
    }

    collider->enabled = command.enabled;
    refreshBodySpatialState(collider->rigidBodyHandle);
}

void EngineImpl::applyCommand(
    const PhysicsCommandBuffer::SetColliderTrigger& command)
{
    Collider* collider = physicsWorld.getCollider(command.collider);

    if (collider && physicsWorld.isColliderActive(command.collider)) {
        collider->isTrigger = command.isTrigger;
    }
}

//================================================
// Scene-wide command processing
//================================================
void EngineImpl::applyCommand(
    const PhysicsCommandBuffer::SleepAllObjects&)
{
    auto& bodyMap = physicsWorld.getRigidBodiesMap();
    auto& dense = bodyMap.dense();

    for (uint32_t i = 0; i < static_cast<uint32_t>(dense.size()); ++i) {
        RigidBody& body = dense[i];

        if (body.asleep) continue;
        if (body.type == BodyType::Static) continue;
        if (body.type == BodyType::Kinematic) continue;
        if (body.motionControl == MotionControl::External) continue;

        broadphaseManager.moveToAsleep(
            bodyMap.handle_from_dense_index(i)
        );
    }
}

void EngineImpl::applyCommand(
    const PhysicsCommandBuffer::AwakenAllObjects&)
{
    auto& bodyMap = physicsWorld.getRigidBodiesMap();
    auto& dense = bodyMap.dense();

    for (uint32_t i = 0; i < static_cast<uint32_t>(dense.size()); ++i) {
        RigidBody& body = dense[i];

        if (!body.asleep) continue;
        if (body.type == BodyType::Static) continue;
        if (body.type == BodyType::Kinematic) continue;
        if (body.motionControl == MotionControl::External) continue;

        broadphaseManager.moveToAwake(
            bodyMap.handle_from_dense_index(i)
        );
    }
}

}
