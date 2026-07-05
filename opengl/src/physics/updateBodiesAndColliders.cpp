#include "pch.h"
#include "physics_engine.h""

//==============================================================
//   Update Bodies and Colliders
//==============================================================
void PhysicsEngine::updateBodiesAndColliders(const std::vector<RigidBodyHandle>& bodies, float dt) {
    for (const RigidBodyHandle& bodyH : bodies) {
        RigidBody* body = caches.bodies.get(bodyH, FUNC_NAME);
        Transform* rootTransform = caches.transforms.get(body->rootTransformHandle, FUNC_NAME);
        Collider* mainCollider = caches.colliders.get(body->colliderHandles[0], FUNC_NAME);

        // once per body
        if (!body->colliderHandles.empty()) {
            // for solo spheres
            if (body->colliderHandles.size() == 1) {
                body->applyRollingFriction(mainCollider->type, dt);
            }

            body->applyVelocityDamping(dt);
            body->applyGravity(dt);
            //body->applyAntistuckFriction(dt);
            body->integrateVelocities(*rootTransform, dt);
            rootTransform->updateCache();
            body->updateInertiaWorld(*rootTransform);
        }

        // per collider
        for (const ColliderHandle& colH : body->colliderHandles) {
            Collider* collider = caches.colliders.get(colH, FUNC_NAME);
            Transform* localTransform = caches.transforms.get(collider->localTransformHandle, FUNC_NAME);

            collider->pose.combineIntoColliderPose(*rootTransform, *localTransform);
            collider->pose.ensureModelMatrix();
            collider->updateAABB(collider->pose);
            collider->updateCollider(collider->pose);
        }

        body->aabb = mainCollider->getAABB();

        // update compound body AABB
        if (body->isCompound()) {
            for (size_t i = 1; i < body->colliderHandles.size(); ++i) {
                Collider* c = caches.colliders.get(body->colliderHandles[i], FUNC_NAME);
                body->aabb.growToInclude(c->getAABB().worldMin);
                body->aabb.growToInclude(c->getAABB().worldMax);
            }

            body->aabb.worldCenter = (body->aabb.worldMin + body->aabb.worldMax) * 0.5f;
            body->aabb.worldHalfExtents = (body->aabb.worldMax - body->aabb.worldMin) * 0.5f;
            body->aabb.setSurfaceArea();
        }
    }
}