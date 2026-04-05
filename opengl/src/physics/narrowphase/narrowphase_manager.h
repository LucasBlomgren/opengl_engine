#pragma once

#include "pointer_cache.h"
#include "bvh/bvh.h"
#include "broadphase/broadphase_pairs.h"
#include "collision_manifold.h"
#include "sat.h"

class PhysicsEngine;

struct ExternalMotionContact {
    RigidBodyHandle bodyA{};
    RigidBodyHandle bodyB{};
    glm::vec3 normal{};
    float penetration = 0.0f;

    ExternalMotionContact(const RigidBodyHandle& bodyA,
        const RigidBodyHandle& bodyB,
        const glm::vec3& normal,
        float penetration)
        : bodyA(bodyA), bodyB(bodyB), normal(normal), penetration(penetration) {
    }
    ExternalMotionContact() = default;
};

class NarrowphaseManager {
public:
    void init(
        CollisionManifold* collisionManifold,
        std::unordered_map<size_t, Contact>* contactCache,
        PointerCache<GameObject, GameObjectHandle>* gameObjectPtrCache,
        PointerCache<Collider, ColliderHandle>* colliderPtrCache,
        PointerCache<RigidBody, RigidBodyHandle>* bodyPtrCache
    );

    std::vector<ExternalMotionContact>& getExternalContacts() {
        return externalContacts;
    }

    void narrowPhase(const std::vector<TerrainPair>& terrainHits, const std::vector<DynamicPair>& dynamicHits);

    void processTerrainPair(const TerrainPair& terrainPair);
    void processDynamicPair(const DynamicPair& dynamicPair);

    void processTerrainBox(const TerrainPair& th, Collider* collider, RigidBody* body, GameObject* obj);
    void processTerrainSphere(const TerrainPair& th, Collider* collider, RigidBody* body, GameObject* obj);

    void processBoxBox(RigidBody* bodyA, RigidBody* bodyB, Collider* colliderA, Collider* colliderB, GameObject* objA, GameObject* objB);
    void processBoxSphere(RigidBody* bodyA, RigidBody* bodyB, Collider* colliderA, Collider* colliderB, GameObject* objA, GameObject* objB);
    void processSphereSphere(RigidBody* bodyA, RigidBody* bodyB, Collider* colliderA, Collider* colliderB, GameObject* objA, GameObject* objB);

    // dynamic vs dynamic contact runtime data
    ContactRuntime makeRuntimeData(
        RigidBody* bodyA, RigidBody* bodyB,
        Collider* colliderA, Collider* colliderB,
        GameObject* objA, GameObject* objB) const;

    // dynamic vs terrain contact runtime data
    ContactRuntime makeRuntimeData(
        RigidBody* bodyA,
        Collider* colliderA,
        GameObject* objA) const;

private:
    // references to pointer caches
    CollisionManifold* collisionManifold;
    std::unordered_map<size_t, Contact>* contactCache;
    PointerCache<GameObject, GameObjectHandle>* gameObjectCache;
    PointerCache<Collider, ColliderHandle>* colliderCache;
    PointerCache<RigidBody, RigidBodyHandle>* bodyCache;

    std::vector<SAT::Result> SAT_resultsList;
    std::vector<ExternalMotionContact> externalContacts;

    glm::vec3 getAvgNormal(const std::vector<SAT::Result>& SAT_resultsList) const;
};