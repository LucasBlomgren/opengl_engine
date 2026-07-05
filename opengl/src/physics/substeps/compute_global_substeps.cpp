#include "pch.h"

#include "physics_engine.h"

//===================================
//    Compute global substeps
//===================================
int PhysicsEngine::computeGlobalSubsteps(float dt) {
    //ScopedTimer t(*frameTimers, "Adaptive substep computation");

    constexpr float safeFraction = 0.50f;
    constexpr float minSafeDistance = 0.02f;

    int halfMaxSubsteps = maxSubsteps / 2;
    int globalSubsteps = 1;

    const std::vector<RigidBodyHandle>& awakeHandles = broadphaseManager.getAwakeList();

    for (const RigidBodyHandle& handle : awakeHandles) {
        RigidBody* body = caches.bodies.get(handle, FUNC_NAME);
        Collider* mainCollider = caches.colliders.get(body->colliderHandles[0], FUNC_NAME);
        if (!body) continue;
        if (body->type != BodyType::Dynamic) continue;
        if (body->asleep) continue;

        Transform* rootTransform = caches.transforms.get(body->rootTransformHandle, FUNC_NAME);

        if (!rootTransform) continue;

        // -----------------------------
        // 1. Estimate object size
        // -----------------------------
        glm::vec3& scale = rootTransform->scale;
        float minExtent = std::min(scale.x, std::min(scale.y, scale.z));
        float boundingRadius = 0.5f * glm::length(scale);
        float safeDistance = std::max(minExtent * safeFraction, minSafeDistance);

        // -----------------------------
        // 2. Estimate motion this frame
        // -----------------------------
        float linearMotion = glm::length(body->linearVelocity) * dt;
        float angularMotion = glm::length(body->angularVelocity) * boundingRadius * dt;
        float totalMotion = linearMotion + angularMotion;

        // Not fast enough to matter.
        if (totalMotion <= safeDistance) {
            continue;
        }

        int wantedSubsteps = static_cast<int>(std::ceil(totalMotion / safeDistance));
        wantedSubsteps = std::clamp(wantedSubsteps, 1, maxSubsteps);

        // If this body cannot increase the current global value, skip expensive query.
        if (wantedSubsteps <= globalSubsteps) {
            continue;
        }

        // -----------------------------
        // 3. Build swept AABB
        // -----------------------------
        AABB& currentAABB = body->aabb;

        glm::vec3 delta = body->linearVelocity * dt;

        AABB endAABB = currentAABB;
        endAABB.worldMin += delta;
        endAABB.worldMax += delta;

        AABB sweptAABB;
        sweptAABB.worldMin = glm::min(currentAABB.worldMin, endAABB.worldMin);
        sweptAABB.worldMax = glm::max(currentAABB.worldMax, endAABB.worldMax);

        // Optional expansion for rotation.
        if (mainCollider->type != ColliderType::SPHERE || body->isCompound()) {
            float angularExpansion = glm::length(body->angularVelocity) * boundingRadius * dt;
            sweptAABB.worldMin -= glm::vec3(angularExpansion);
            sweptAABB.worldMax += glm::vec3(angularExpansion);
        }

        // Optional small skin.
        constexpr float sweptSkin = 0.01f;
        sweptAABB.worldMin -= glm::vec3(sweptSkin);
        sweptAABB.worldMax += glm::vec3(sweptSkin);

        // -----------------------------
        // 4. Check if swept AABB actually hits anything
        // -----------------------------
        bool hitAwake =
            broadphaseManager.getAwakeBVH().queryAny(sweptAABB, handle);

        bool hitAsleep = false;
        if (!hitAwake) {
            hitAsleep = broadphaseManager.getAsleepBVH().queryAny(sweptAABB, handle);
        }

        bool hitTerrain = false;
        if (!hitAwake && !hitAsleep) {
            hitTerrain = broadphaseManager.getTerrainBVH().queryAny(sweptAABB);
        }

        bool hitStatic = false;
        if (!hitAwake && !hitAsleep && !hitTerrain) {
            hitStatic = broadphaseManager.getStaticBVH().queryAny(sweptAABB, handle);
        }

        if (!hitAwake && !hitAsleep && !hitTerrain && !hitStatic) {
            continue;
        }

        // Static-only hits are cheaper: no dynamic target needs to wake up or propagate impulses.
        if (hitStatic) {
            wantedSubsteps = std::min(wantedSubsteps, halfMaxSubsteps);
        }

        // This fast object actually risks collision this frame.
        globalSubsteps = std::max(globalSubsteps, wantedSubsteps);
    }

    return globalSubsteps;
}