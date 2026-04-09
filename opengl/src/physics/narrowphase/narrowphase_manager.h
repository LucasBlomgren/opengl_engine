#pragma once

#include "bvh/bvh.h"
#include "broadphase/broadphase_pairs.h"
#include "collision_manifold.h"
#include "sat.h"
#include "runtime_caches.h"

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
        RuntimeCaches* caches
    );

    std::vector<ExternalMotionContact>& getExternalContacts() {
        return externalContacts;
    }

    void collectTerrainTriCandidates(
        Collider* collider,
        const std::vector<Tri*>& inputTris,
        std::vector<Tri*>& outCandidates
    );
    std::vector<Tri*> terrainTriCandidates;

    void narrowPhase(const std::vector<TerrainPair>& terrainHits, const std::vector<DynamicPair>& dynamicHits);

    void processTerrainPair(const TerrainPair& terrainPair);
    void processDynamicPair(const DynamicPair& dynamicPair);

    void processTerrainTriBox(RigidBodyHandle bodyH, Collider* collider, RigidBody* body, Transform* colliderWorldTransform);
    void processTerrainTriSphere(RigidBodyHandle bodyH, Collider* collider, RigidBody* body, Transform* colliderWorldTransform);

    void processBoxBox(RigidBody* bodyA, RigidBody* bodyB, Collider* colliderA, Collider* colliderB, Transform* transformA, Transform* transformB);
    void processBoxSphere(RigidBody* bodyA, RigidBody* bodyB, Collider* colliderA, Collider* colliderB, Transform* transformA, Transform* transformB);
    void processSphereSphere(RigidBody* bodyA, RigidBody* bodyB, Collider* colliderA, Collider* colliderB, Transform* transformA, Transform* transformB);

    // dynamic vs dynamic contact runtime data
    ContactRuntime makeRuntimeData(
        RigidBody* bodyA, RigidBody* bodyB,
        Collider* colliderA, Collider* colliderB,
        Transform* bodyRootA, Transform* bodyRootB,
        Transform* colliderWorldA, Transform* colliderWorldB) const;

    // dynamic vs terrain contact runtime data
    ContactRuntime makeRuntimeData(
        RigidBody* bodyA,
        Collider* colliderA,
        Transform* bodyRootA,
        Transform* colliderWorldA) const;

private:
    // references to pointer caches
    CollisionManifold* collisionManifold = nullptr;
    std::unordered_map<size_t, Contact>* contactCache = nullptr;
    RuntimeCaches* caches = nullptr;

    std::vector<SAT::Result> SAT_resultsList;
    std::vector<ExternalMotionContact> externalContacts;

    glm::vec3 getAvgNormal(const std::vector<SAT::Result>& SAT_resultsList) const;
};