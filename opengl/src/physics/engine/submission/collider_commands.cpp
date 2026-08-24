#include "pch.h"

#include <type_traits>
#include <utility>
#include <variant>

#include "physics/public/engine.h"

namespace physics {

using namespace internal;

namespace {

const RigidBody* resolveBody(
    const cmd::Buffer& commandBuffer,
    const PhysicsWorld& physicsWorld,
    BodyHandle handle)
{
    if (commandBuffer.isBodyPendingDestroy(handle)) {
        return nullptr;
    }

    if (const RigidBody* body =
        commandBuffer.tryGetPendingBodyCreate(handle)) {
        return body;
    }

    return physicsWorld.tryGetBody(handle);
}

const Collider* resolveCollider(
    const cmd::Buffer& commandBuffer,
    const PhysicsWorld& physicsWorld,
    ColliderHandle handle)
{
    if (commandBuffer.isColliderPendingDestroy(handle)) {
        return nullptr;
    }

    const Collider* collider =
        commandBuffer.tryGetPendingColliderCreate(handle);

    if (!collider) {
        collider = physicsWorld.tryGetCollider(handle);
    }

    if (!collider ||
        commandBuffer.isBodyPendingDestroy(
            collider->rigidBodyHandle)) {
        return nullptr;
    }

    return collider;
}

}

//=========================================
// Collider creation and destruction
//=========================================
ColliderHandle Engine::createCollider(
    BodyHandle bodyHandle,
    const ColliderDesc& desc)
{
    const RigidBody* body = resolveBody(
        commandBuffer,
        physicsWorld,
        bodyHandle
    );

    if (!body) {
        return {};
    }

    ColliderHandle colliderHandle =
        physicsWorld.reserveColliderHandle();

    Collider collider;
    collider.rigidBodyHandle = bodyHandle;
    collider.localPose = desc.localPose;
    collider.localScale = desc.localScale;
    collider.enabled = desc.enabled;
    collider.isTrigger = desc.isTrigger;
    collider.userTag = desc.userTag;

    bool validShape = true;

    std::visit([&](const auto& shapeDesc) {
        using ShapeDescType = std::decay_t<decltype(shapeDesc)>;

        if constexpr (std::is_same_v<ShapeDescType, BoxShapeDesc>) {
            collider.type = ColliderType::CUBOID;
            collider.shape = OOBB(shapeDesc.halfExtents, shapeDesc.center);
        }
        else if constexpr (std::is_same_v<ShapeDescType, SphereShapeDesc>) {
            collider.type = ColliderType::SPHERE;
            collider.shape = Sphere(shapeDesc.radius, shapeDesc.center);
        }
        else {
            validShape = false;
        }
    }, desc.shape);

    if (!validShape) {
        physicsWorld.releaseColliderReservation(colliderHandle);
        return {};
    }

    collider.updateWorldPose(body->pose, body->scale);
    collider.updateShape();
    collider.updateAABB();

    commandBuffer.recordColliderCreate(
        colliderHandle,
        std::move(collider)
    );

    return colliderHandle;
}

bool Engine::destroyCollider(
    ColliderHandle colliderHandle) {
    if (!resolveCollider(
        commandBuffer,
        physicsWorld,
        colliderHandle)) {
        return false;
    }

    return commandBuffer.recordColliderDestroy(
        colliderHandle,
        physicsWorld
    );
}

//=========================================
// Collider commands
//=========================================
bool Engine::setColliderLocalPose(
    ColliderHandle handle,
    const Pose& localPose)
{
    const Collider* collider = resolveCollider(
        commandBuffer,
        physicsWorld,
        handle
    );

    if (!collider) {
        return false;
    }

    commandBuffer.recordSetColliderLocalPose(handle, localPose);
    return true;
}

bool Engine::setColliderLocalTransform(
    ColliderHandle handle,
    const Pose& localPose,
    const glm::vec3& localScale)
{
    const Collider* collider = resolveCollider(
        commandBuffer,
        physicsWorld,
        handle
    );

    if (!collider) {
        return false;
    }

    commandBuffer.recordSetColliderLocalTransform(
        handle,
        localPose,
        localScale
    );
    return true;
}

bool Engine::setColliderShape(
    ColliderHandle handle,
    const ColliderShapeDesc& shape)
{
    const Collider* collider = resolveCollider(
        commandBuffer,
        physicsWorld,
        handle
    );

    if (!collider) {
        return false;
    }

    commandBuffer.recordSetColliderShape(handle, shape);
    return true;
}

bool Engine::setColliderEnabled(
    ColliderHandle handle,
    bool enabled)
{
    const Collider* collider = resolveCollider(
        commandBuffer,
        physicsWorld,
        handle
    );

    if (!collider) {
        return false;
    }

    commandBuffer.recordSetColliderEnabled(handle, enabled);
    return true;
}

bool Engine::setColliderTrigger(
    ColliderHandle handle,
    bool isTrigger)
{
    const Collider* collider = resolveCollider(
        commandBuffer,
        physicsWorld,
        handle
    );

    if (!collider) {
        return false;
    }

    commandBuffer.recordSetColliderTrigger(handle, isTrigger);
    return true;
}

}
