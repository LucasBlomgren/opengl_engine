#include "pch.h"

#include <type_traits>
#include <variant>

#include "physics/engine/physics_engine_impl.h"

namespace physics::internal {

//=========================================
// Collider creation and destruction
//=========================================
ColliderHandle EngineImpl::submitCreateCollider(
    BodyHandle bodyHandle,
    const ColliderDesc& desc)
{
    RigidBody* body = physicsWorld.getRigidBody(bodyHandle);

    if (!body || 
        commandBuffer.isBodyPendingDestroy(bodyHandle)) {
        return {};
    }

    ColliderHandle colliderHandle = physicsWorld.createPendingCollider();
    Collider* collider = physicsWorld.getCollider(colliderHandle);

    if (!collider) {
        return {};
    }

    collider->rigidBodyHandle = bodyHandle;
    collider->localPose = desc.localPose;
    collider->localScale = desc.localScale;
    collider->enabled = desc.enabled;
    collider->isTrigger = desc.isTrigger;
    collider->userTag = desc.userTag;

    bool validShape = true;

    std::visit([&](const auto& shapeDesc) {
        using ShapeDescType = std::decay_t<decltype(shapeDesc)>;

        if constexpr (std::is_same_v<ShapeDescType, BoxShapeDesc>) {
            collider->type = ColliderType::CUBOID;
            collider->shape = OOBB(shapeDesc.halfExtents, shapeDesc.center);
        }
        else if constexpr (std::is_same_v<ShapeDescType, SphereShapeDesc>) {
            collider->type = ColliderType::SPHERE;
            collider->shape = Sphere(shapeDesc.radius, shapeDesc.center);
        }
        else {
            validShape = false;
        }
    }, desc.shape);

    if (!validShape) {
        physicsWorld.discardPendingCollider(colliderHandle);
        return {};
    }

    collider->updateWorldPose(body->pose, body->scale);
    collider->updateShape();
    collider->updateAABB();

    commandBuffer.recordColliderCreate(colliderHandle);

    return colliderHandle;
}

bool EngineImpl::submitDestroyCollider(
    ColliderHandle colliderHandle) {
    const Collider* collider = physicsWorld.getCollider(colliderHandle);

    if (!collider) {
        return false;
    }

    return commandBuffer.recordColliderDestroy(colliderHandle);
}

//=========================================
// Collider commands
//=========================================
bool EngineImpl::submitSetColliderLocalPose(
    ColliderHandle handle,
    const Pose& localPose)
{
    const Collider* collider = physicsWorld.getCollider(handle);

    if (!collider ||
        commandBuffer.isColliderPendingDestroy(handle) ||
        commandBuffer.isBodyPendingDestroy(collider->rigidBodyHandle)) {
        return false;
    }

    commandBuffer.recordSetColliderLocalPose(handle, localPose);
    return true;
}

bool EngineImpl::submitSetColliderLocalTransform(
    ColliderHandle handle,
    const Pose& localPose,
    const glm::vec3& localScale)
{
    const Collider* collider = physicsWorld.getCollider(handle);

    if (!collider ||
        commandBuffer.isColliderPendingDestroy(handle) ||
        commandBuffer.isBodyPendingDestroy(collider->rigidBodyHandle)) {
        return false;
    }

    commandBuffer.recordSetColliderLocalTransform(
        handle,
        localPose,
        localScale
    );
    return true;
}

bool EngineImpl::submitSetColliderShape(
    ColliderHandle handle,
    const ColliderShapeDesc& shape)
{
    const Collider* collider = physicsWorld.getCollider(handle);

    if (!collider ||
        commandBuffer.isColliderPendingDestroy(handle) ||
        commandBuffer.isBodyPendingDestroy(collider->rigidBodyHandle)) {
        return false;
    }

    commandBuffer.recordSetColliderShape(handle, shape);
    return true;
}

bool EngineImpl::submitSetColliderEnabled(
    ColliderHandle handle,
    bool enabled)
{
    const Collider* collider = physicsWorld.getCollider(handle);

    if (!collider ||
        commandBuffer.isColliderPendingDestroy(handle) ||
        commandBuffer.isBodyPendingDestroy(collider->rigidBodyHandle)) {
        return false;
    }

    commandBuffer.recordSetColliderEnabled(handle, enabled);
    return true;
}

bool EngineImpl::submitSetColliderTrigger(
    ColliderHandle handle,
    bool isTrigger)
{
    const Collider* collider = physicsWorld.getCollider(handle);

    if (!collider ||
        commandBuffer.isColliderPendingDestroy(handle) ||
        commandBuffer.isBodyPendingDestroy(collider->rigidBodyHandle)) {
        return false;
    }

    commandBuffer.recordSetColliderTrigger(handle, isTrigger);
    return true;
}

}
