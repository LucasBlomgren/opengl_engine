#include "pch.h"

#include "physics/engine/physics_engine_impl.h"

//=====================================
// Scene-wide commands
//=====================================
void PhysicsEngine::Impl::sleepAllObjects() {
    auto& bodyMap = physicsWorld.getRigidBodiesMap();
    auto& dense = bodyMap.dense();

    for (uint32_t i = 0; i < static_cast<uint32_t>(dense.size()); ++i) {
        RigidBody& body = dense[i];

        if (body.asleep) continue;
        if (body.type == BodyType::Static) continue;
        if (body.type == BodyType::Kinematic) continue;
        if (body.motionControl == MotionControl::External) continue;

        RigidBodyHandle handle = bodyMap.handle_from_dense_index(i);
        BroadphaseBucket bucket = BroadphaseBucket::Asleep;
        broadphaseManager.moveToAsleep(handle);
    }
}

void PhysicsEngine::Impl::awakenAllObjects() {
    auto& bodyMap = physicsWorld.getRigidBodiesMap();
    auto& dense = bodyMap.dense();

    for (uint32_t i = 0; i < static_cast<uint32_t>(dense.size()); ++i) {
        RigidBody& body = dense[i];

        if (!body.asleep) continue;
        if (body.type == BodyType::Static) continue;
        if (body.type == BodyType::Kinematic) continue;
        if (body.motionControl == MotionControl::External) continue;

        RigidBodyHandle handle = bodyMap.handle_from_dense_index(i);
        broadphaseManager.moveToAwake(handle);
    }
}

//=====================================
// 
//=====================================
void PhysicsEngine::Impl::setBVHDirty(RigidBodyHandle& handle) {
    broadphaseManager.setBVHDirty(handle);
}

//=====================================
// Debug visualization
//=====================================
void PhysicsEngine::Impl::updateBVHRenderData(const BVHType& type, bool update) {
    broadphaseManager.updateBVHRenderData(type, update);
}

//=====================================
// Temporary legacy synchronization
//=====================================
void PhysicsEngine::Impl::syncBodyFromTransform(RigidBodyHandle bodyHandle) {
    RigidBody* body = caches.bodies.get(bodyHandle, FUNC_NAME);

    if (!body) {
        return;
    }

    Transform* rootTransform = caches.transforms.get(body->rootTransformHandle, FUNC_NAME);

    if (!rootTransform) {
        return;
    }

    rootTransform->updateCache();

    body->pose.position = rootTransform->position;
    body->pose.orientation = rootTransform->orientation;
    body->scale = rootTransform->scale;

    updateCollidersAndBodyAABB(body, rootTransform);

    if (!body->colliderHandles.empty()) {
        Collider* mainCollider = caches.colliders.get(body->colliderHandles[0], FUNC_NAME);

        if (mainCollider) {
            Transform inertiaTransform;

            if (mainCollider->type == ColliderType::SPHERE) {
                const Sphere& sphere = std::get<Sphere>(mainCollider->shape);
                inertiaTransform.scale = glm::vec3(sphere.radiusWorld * 2.0f);
            }
            else {
                inertiaTransform.scale = mainCollider->worldScale;
            }

            body->calculateInverseInertia(mainCollider->type, *mainCollider, inertiaTransform);
        }
    }

    body->updateInertiaWorld();

    if (body->broadphaseHandle.bucket != BroadphaseBucket::None) {
        setBVHDirty(bodyHandle);
    }
}
