#include "pch.h"

#include "physics/public/physics_engine.h"
#include "physics/engine/physics_engine_impl.h"

#include <algorithm>
#include <type_traits>
#include <variant>

ColliderHandle PhysicsEngine::createCollider(RigidBodyHandle body, const ColliderDesc& desc) {
    return impl->createCollider(body, desc);
}

bool PhysicsEngine::destroyCollider(ColliderHandle collider) {
    return impl->destroyCollider(collider);
}

std::optional<ColliderState> PhysicsEngine::getColliderState(ColliderHandle collider) const {
    return impl->getColliderState(collider);
}

bool PhysicsEngine::setColliderLocalPose(ColliderHandle collider, const PhysicsPose& localPose) {
    return impl->setColliderLocalPose(collider, localPose);
}

bool PhysicsEngine::setColliderEnabled(ColliderHandle collider, bool enabled) {
    return impl->setColliderEnabled(collider, enabled);
}

bool PhysicsEngine::setColliderTrigger(ColliderHandle collider, bool isTrigger) {
    return impl->setColliderTrigger(collider, isTrigger);
}

ColliderHandle PhysicsEngine::Impl::createCollider(RigidBodyHandle bodyHandle, const ColliderDesc& desc) {
    RigidBody* body = physicsWorld.getRigidBody(bodyHandle);

    if (!body) {
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
        physicsWorld.deleteCollider(colliderHandle);
        return {};
    }

    collider->updateWorldPose(body->pose, body->scale);
    collider->updateShape();
    collider->updateAABB();

    body->colliderHandles.push_back(colliderHandle);
    body->aabb = physicsWorld.computeBodyAABB(*body);

    if (body->colliderHandles.size() == 1) {
        Transform inertiaTransform;
        if (collider->type == ColliderType::SPHERE) {
            const Sphere& sphere = std::get<Sphere>(collider->shape);
            inertiaTransform.scale = glm::vec3(sphere.radiusWorld * 2.0f);
        }
        else {
            inertiaTransform.scale = collider->worldScale;
        }
        body->calculateInverseInertia(collider->type, *collider, inertiaTransform);
    }

    const bool addAlreadyPending = std::any_of(
        pending.begin(),
        pending.end(),
        [bodyHandle](const PhysCmd& cmd) {
            return cmd.type == PhysCmd::Type::Add && cmd.handle == bodyHandle;
        });

    if (body->broadphaseHandle.bucket == BroadphaseBucket::None &&
        !addAlreadyPending) {
        BroadphaseBucket bucket = BroadphaseBucket::Awake;

        if (body->type == BodyType::Static) {
            bucket = BroadphaseBucket::Static;
        }
        else if (body->asleep) {
            bucket = BroadphaseBucket::Asleep;
        }

        queueAdd(bodyHandle, bucket);
    }
    else if (body->broadphaseHandle.bucket != BroadphaseBucket::None) {
        setBVHDirty(bodyHandle);
    }

    return colliderHandle;
}

bool PhysicsEngine::Impl::destroyCollider(ColliderHandle colliderHandle) {
    Collider* collider = physicsWorld.getCollider(colliderHandle);

    if (!collider) {
        return false;
    }

    RigidBodyHandle bodyHandle = collider->rigidBodyHandle;
    RigidBody* body = physicsWorld.getRigidBody(bodyHandle);

    if (body) {
        std::vector<ColliderHandle>& handles = body->colliderHandles;
        handles.erase(std::remove(handles.begin(), handles.end(), colliderHandle), handles.end());
    }

    contactCache.clear();
    physicsWorld.deleteCollider(colliderHandle);

    if (body) {
        if (!body->colliderHandles.empty()) {
            body->aabb = physicsWorld.computeBodyAABB(*body);
        }
        else {
            body->aabb = AABB{};
        }

        if (body->colliderHandles.empty()) {
            pending.erase(
                std::remove_if(
                    pending.begin(),
                    pending.end(),
                    [bodyHandle](const PhysCmd& cmd) { return cmd.handle == bodyHandle; }),
                pending.end());
            if (body->broadphaseHandle.bucket != BroadphaseBucket::None) {
                broadphaseManager.remove(bodyHandle);
            }
        }
        else if (body->broadphaseHandle.bucket != BroadphaseBucket::None) {
            setBVHDirty(bodyHandle);
        }

        if (body->asleep) {
            body->setAwake();
            broadphaseManager.moveToAwake(bodyHandle);
        }
    }

    return true;
}

std::optional<ColliderState> PhysicsEngine::Impl::getColliderState(ColliderHandle handle) const {
    const Collider* collider = physicsWorld.getCollider(handle);

    if (!collider) {
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

bool PhysicsEngine::Impl::setColliderLocalPose(ColliderHandle handle, const PhysicsPose& localPose) {
    Collider* collider = physicsWorld.getCollider(handle);

    if (!collider) {
        return false;
    }

    RigidBody* body = physicsWorld.getRigidBody(collider->rigidBodyHandle);

    if (!body) {
        return false;
    }

    collider->localPose = localPose;
    collider->updateWorldPose(body->pose, body->scale);
    collider->updateShape();
    collider->updateAABB();

    body->aabb = physicsWorld.computeBodyAABB(*body);

    if (body->broadphaseHandle.bucket != BroadphaseBucket::None) {
        setBVHDirty(collider->rigidBodyHandle);
    }

    if (body->asleep) {
        body->setAwake();
        broadphaseManager.moveToAwake(collider->rigidBodyHandle);
    }

    return true;
}

bool PhysicsEngine::Impl::setColliderEnabled(ColliderHandle handle, bool enabled) {
    Collider* collider = physicsWorld.getCollider(handle);

    if (!collider) {
        return false;
    }

    collider->enabled = enabled;

    RigidBodyHandle bodyHandle = collider->rigidBodyHandle;
    RigidBody* body = physicsWorld.getRigidBody(bodyHandle);

    if (body) {
        body->aabb = physicsWorld.computeBodyAABB(*body);

        if (body->broadphaseHandle.bucket != BroadphaseBucket::None) {
            setBVHDirty(bodyHandle);
        }
    }

    return true;
}

bool PhysicsEngine::Impl::setColliderTrigger(ColliderHandle handle, bool isTrigger) {
    Collider* collider = physicsWorld.getCollider(handle);

    if (!collider) {
        return false;
    }

    collider->isTrigger = isTrigger;
    contactCache.clear();

    return true;
}
