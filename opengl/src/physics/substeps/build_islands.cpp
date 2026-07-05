#include "pch.h"
#include "physics_engine.h"

MotionRisk PhysicsEngine::computeMotionRisk(
    RigidBodyHandle h,
    float frameDt)
{
    MotionRisk risk;

    RigidBody* body = caches.bodies.get(h, FUNC_NAME);
    if (!body) return risk;
    if (body->type != BodyType::Dynamic) return risk;
    if (body->asleep) return risk;
    if (body->motionControl == MotionControl::External) return risk;
    if (body->colliderHandles.empty()) return risk;

    Collider* mainCollider =
        caches.colliders.get(body->colliderHandles[0], FUNC_NAME);

    if (!mainCollider) return risk;

    Transform* rootTransform =
        caches.transforms.get(body->rootTransformHandle, FUNC_NAME);

    if (!rootTransform) return risk;

    constexpr float safeFraction = 0.50f;
    constexpr float minSafeDistance = 0.02f;

    glm::vec3 scale = rootTransform->scale;

    float minExtent = std::min(scale.x, std::min(scale.y, scale.z));
    float boundingRadius = 0.5f * glm::length(scale);

    float safeDistance =
        std::max(minExtent * safeFraction, minSafeDistance);

    float linearMotion =
        glm::length(body->linearVelocity) * frameDt;

    float angularMotion =
        glm::length(body->angularVelocity) * boundingRadius * frameDt;

    float totalMotion = linearMotion + angularMotion;

    if (totalMotion <= safeDistance) {
        return risk;
    }

    int wanted =
        static_cast<int>(std::ceil(totalMotion / safeDistance));

    wanted = std::clamp(wanted, 1, maxSubsteps);

    if (wanted <= 1) {
        return risk;
    }

    AABB current = body->aabb;

    glm::vec3 delta = body->linearVelocity * frameDt;

    AABB end = current;
    end.worldMin += delta;
    end.worldMax += delta;

    AABB swept;
    swept.worldMin = glm::min(current.worldMin, end.worldMin);
    swept.worldMax = glm::max(current.worldMax, end.worldMax);

    if (mainCollider->type != ColliderType::SPHERE || body->isCompound()) {
        float angularExpansion =
            glm::length(body->angularVelocity) *
            boundingRadius *
            frameDt;

        swept.worldMin -= glm::vec3(angularExpansion);
        swept.worldMax += glm::vec3(angularExpansion);
    }

    constexpr float sweptSkin = 0.01f;
    swept.worldMin -= glm::vec3(sweptSkin);
    swept.worldMax += glm::vec3(sweptSkin);

    risk.risky = true;
    risk.wantedSubsteps = wanted;
    risk.sweptAABB = swept;

    return risk;
}

void PhysicsEngine::createPredictedIslandsMVP(float frameDt)
{
    predictedIslands.clear();
    restBodies.clear();

    const std::vector<RigidBodyHandle>& awake =
        broadphaseManager.getAwakeList();

    size_t slotCap =
        physicsWorld.getRigidBodiesMap().slot_capacity();

    if (isIslandBody.size() < slotCap) {
        isIslandBody.resize(slotCap, 0);
    }

    // Reset marks för awake bodies.
    for (RigidBodyHandle h : awake) {
        isIslandBody[h.slot] = 0;
    }

    static IslandDSU dsu;
    dsu.init(slotCap);

    static std::vector<RigidBodyHandle> candidates;
    static std::vector<RigidBodyHandle> asleepCandidates;

    candidates.reserve(128);
    asleepCandidates.reserve(128);

    candidates.clear();
    asleepCandidates.clear();

    // -------------------------------
    // Compute motion risk and build DSU
    // -------------------------------
    for (RigidBodyHandle h : awake) {
        MotionRisk risk = computeMotionRisk(h, frameDt);

        if (!risk.risky) {
            continue;
        }

        int selfIdx = -1;

        // -------------------------------
        // Dynamic awake candidates
        // -------------------------------
        candidates.clear();

        broadphaseManager.getAwakeBVH().singleQuery(
            risk.sweptAABB,
            candidates
        );

        for (RigidBodyHandle otherH : candidates) {
            if (otherH == h) continue;

            RigidBody* other =
                caches.bodies.get(otherH, FUNC_NAME);

            if (!other) continue;
            if (other->type != BodyType::Dynamic) continue;
            if (other->asleep) continue;
            if (other->motionControl == MotionControl::External) continue;

            if (selfIdx == -1) {
                selfIdx = dsu.add(h, risk.wantedSubsteps);
            }

            int otherIdx = dsu.add(otherH, 1);

            dsu.unite(selfIdx, otherIdx);
        }

        // -------------------------------
        // Asleep dynamic candidates
        // -------------------------------
        asleepCandidates.clear();

        broadphaseManager.getAsleepBVH().singleQuery(
            risk.sweptAABB,
            asleepCandidates
        );

        for (RigidBodyHandle otherH : asleepCandidates) {
            if (otherH == h) continue;

            RigidBody* other =
                caches.bodies.get(otherH, FUNC_NAME);

            if (!other) continue;
            if (other->type != BodyType::Dynamic) continue;
            if (other->motionControl == MotionControl::External) continue;

            if (selfIdx == -1) {
                selfIdx = dsu.add(h, risk.wantedSubsteps);
            }

            int otherIdx = dsu.add(otherH, 1);
            dsu.unite(selfIdx, otherIdx);
        }

        // -------------------------------
        // Static / terrain hit
        // Då blir fast body en singleton island.
        // -------------------------------
        bool hitTerrain =
            broadphaseManager.getTerrainBVH().queryAny(risk.sweptAABB);

        bool hitStatic =
            broadphaseManager.getStaticBVH().queryAny(risk.sweptAABB, h);

        if (hitTerrain || hitStatic) {
            dsu.add(h, risk.wantedSubsteps);
        }
    }

    // -------------------------------
    // DSU root -> Island + mark island bodies
    // -------------------------------
    std::unordered_map<int, int> rootToIsland;
    rootToIsland.reserve(dsu.parent.size());

    for (int i = 0; i < static_cast<int>(dsu.parent.size()); ++i) {
        int root = dsu.find(i);

        auto it = rootToIsland.find(root);

        if (it == rootToIsland.end()) {
            Island island;
            island.id = static_cast<uint32_t>(predictedIslands.size());
            island.substeps = 1;

            predictedIslands.push_back(std::move(island));

            int islandIndex =
                static_cast<int>(predictedIslands.size()) - 1;

            rootToIsland[root] = islandIndex;
            it = rootToIsland.find(root);
        }

        Island& island = predictedIslands[it->second];

        RigidBodyHandle bodyH = dsu.bodies[i];

        island.bodies.push_back(bodyH);
        island.substeps =
            std::max(island.substeps, dsu.wantedSubsteps[i]);

        // Mark this body as part of an island
        isIslandBody[bodyH.slot] = 1;
    }

    // -------------------------------
    // Build restBodies
    // -------------------------------
    for (RigidBodyHandle h : awake) {
        bool marked =
            h.slot < isIslandBody.size() && 
            isIslandBody[h.slot];

        if (!marked) {
            restBodies.push_back(h);
        }
    }
}