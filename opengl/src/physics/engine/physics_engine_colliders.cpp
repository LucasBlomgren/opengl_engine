#include "pch.h"

#include <algorithm>
#include <type_traits>
#include <variant>

#include "physics/public/physics_engine.h"
#include "physics/engine/physics_engine_impl.h"

//=========================================
// Collider creation and destruction
//=========================================
ColliderHandle PhysicsEngine::createCollider(RigidBodyHandle body, const ColliderDesc& desc) {
    return impl->createCollider(body, desc);
}

bool PhysicsEngine::destroyCollider(ColliderHandle collider) {
    return impl->destroyCollider(collider);
}

ColliderHandle PhysicsEngine::Impl::createCollider(RigidBodyHandle bodyHandle, const ColliderDesc& desc) {
    RigidBody* body = physicsWorld.getRigidBody(bodyHandle);

    if (!body || externalCommands.isBodyPendingDestroy(bodyHandle)) {
        return {};
    }

    ColliderHandle colliderHandle = physicsWorld.createCollider();
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

            Sphere sphere(shapeDesc.radius);
            sphere.centerLocal = shapeDesc.center;
            collider->shape = sphere;
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
    
    externalCommands.recordColliderCreate(colliderHandle);

    return colliderHandle;
}

bool PhysicsEngine::Impl::destroyCollider(ColliderHandle colliderHandle) {
    const Collider* collider = physicsWorld.getCollider(colliderHandle);

    if (!collider) {
        return false;
    }

    return externalCommands.recordColliderDestroy(colliderHandle);
}

//=========================================
// Collider commands
//=========================================
bool PhysicsEngine::setColliderLocalPose(ColliderHandle collider, const PhysicsPose& localPose) {
    return impl->setColliderLocalPose(collider, localPose);
}

bool PhysicsEngine::setColliderEnabled(ColliderHandle collider, bool enabled) {
    return impl->setColliderEnabled(collider, enabled);
}

bool PhysicsEngine::setColliderTrigger(ColliderHandle collider, bool isTrigger) {
    return impl->setColliderTrigger(collider, isTrigger);
}

bool PhysicsEngine::Impl::setColliderLocalPose(ColliderHandle handle, const PhysicsPose& localPose) {
    const Collider* collider = physicsWorld.getCollider(handle);

    if (!collider ||
        externalCommands.isColliderPendingDestroy(handle) ||
        externalCommands.isBodyPendingDestroy(collider->rigidBodyHandle)) {
        return false;
    }

    externalCommands.queueSetColliderLocalPose(handle, localPose);
    return true;
}

bool PhysicsEngine::Impl::setColliderEnabled(ColliderHandle handle, bool enabled) {
    const Collider* collider = physicsWorld.getCollider(handle);

    if (!collider ||
        externalCommands.isColliderPendingDestroy(handle) ||
        externalCommands.isBodyPendingDestroy(collider->rigidBodyHandle)) {
        return false;
    }

    externalCommands.queueSetColliderEnabled(handle, enabled);
    return true;
}

bool PhysicsEngine::Impl::setColliderTrigger(ColliderHandle handle, bool isTrigger) {
    const Collider* collider = physicsWorld.getCollider(handle);

    if (!collider ||
        externalCommands.isColliderPendingDestroy(handle) ||
        externalCommands.isBodyPendingDestroy(collider->rigidBodyHandle)) {
        return false;
    }

    externalCommands.queueSetColliderTrigger(handle, isTrigger);
    return true;
}

//=========================================
// Collider state queries
//=========================================
std::optional<ColliderState> PhysicsEngine::getColliderState(ColliderHandle collider) const {
    return impl->getColliderState(collider);
}

std::optional<ColliderState> PhysicsEngine::Impl::getColliderState(ColliderHandle handle) const {
    const Collider* collider = physicsWorld.getCollider(handle);

    if (!collider ||
        externalCommands.isColliderPendingDestroy(handle) ||
        externalCommands.isBodyPendingDestroy(collider->rigidBodyHandle)) {
        return std::nullopt;
    }

    ColliderState state;
    state.body = collider->rigidBodyHandle;
    state.localPose = collider->localPose;
    state.localScale = collider->localScale;
    state.worldPose.position = collider->worldPose.position;
    state.worldPose.orientation = collider->worldPose.orientation;
    state.worldScale = collider->worldScale;
    state.enabled = collider->enabled;
    state.isTrigger = collider->isTrigger;
    state.userTag = collider->userTag;

    return state;
}
