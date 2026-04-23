#pragma once

#include "rigid_body.h"
#include "broadphase/broadphase_pairs.h"
#include "collision_manifold.h"
#include "sat.h"
#include "runtime_caches.h"

struct ExternalMotionContact {
    RigidBodyHandle bodyA;
    RigidBodyHandle bodyB;
    glm::vec3 normal{ 0.0f };
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
        RuntimeCaches* caches,
        std::vector<RigidBodyHandle>* toWake
    );

    std::vector<ExternalMotionContact>& getExternalContacts() {
        return externalContacts;
    }

    // main function
    void narrowPhase(const std::vector<TerrainPair>& terrainHits, const std::vector<DynamicPair>& dynamicHits);

private:
    // references to caches
    CollisionManifold* collisionManifold = nullptr;
    std::unordered_map<size_t, Contact>* contactCache = nullptr;
    RuntimeCaches* caches = nullptr;

    void processTerrainPair(const TerrainPair& terrainPair);
    void processDynamicPair(const DynamicPair& dynamicPair);

    void processTerrainTriBox(
        RigidBodyHandle bodyH, 
        Collider* collider, 
        RigidBody* body,
        const std::vector<Tri*>& candidates
    );

    void processTerrainTriSphere(
        RigidBodyHandle bodyH, 
        Collider* collider, 
        RigidBody* body,
        const std::vector<Tri*>& candidates
    );

    void processBoxBox(
        RigidBodyHandle bodyHandleA,
        RigidBodyHandle bodyHandleB,
        ColliderHandle colliderHandleA,
        ColliderHandle colliderHandleB,
        RigidBody* bodyA,
        RigidBody* bodyB,
        Collider* colliderA,
        Collider* colliderB
    );

    void processBoxSphere(
        RigidBodyHandle bodyHandleA,
        RigidBodyHandle bodyHandleB,
        ColliderHandle colliderHandleA,
        ColliderHandle colliderHandleB,
        RigidBody* bodyA,
        RigidBody* bodyB,
        Collider* colliderA,
        Collider* colliderB
    );

    void processSphereSphere(
        RigidBodyHandle bodyHandleA,
        RigidBodyHandle bodyHandleB,
        ColliderHandle colliderHandleA,
        ColliderHandle colliderHandleB,
        RigidBody* bodyA,
        RigidBody* bodyB,
        Collider* colliderA,
        Collider* colliderB
    );

    // dynamic vs dynamic contact runtime data
    ContactRuntime makeRuntimeData(
        RigidBody* bodyA, RigidBody* bodyB,
        Collider* colliderA, Collider* colliderB,
        Transform* bodyRootA, Transform* bodyRootB
    ) const;

    // dynamic vs terrain contact runtime data
    ContactRuntime makeRuntimeData(
        RigidBody* bodyA,
        Collider* colliderA,
        Transform* bodyRootA
    ) const;

    std::vector<RigidBodyHandle>* toWake = nullptr; // for wake-up requests for bodies that should be woken up after processing dynamic pairs

    std::vector<SAT::Result> SAT_resultsList; // for storing multiple SAT results for a single collider vs terrain pair
    std::vector<ExternalMotionContact> externalContacts; // contacts to be sent to character controller for external motion handling

    // Helpers
    glm::vec3 getAvgNormal(const std::vector<SAT::Result>& SAT_resultsList) const;

    void collectTerrainTriCandidates(
        Collider* collider,
        const std::vector<Tri*>& inputTris,
        std::vector<Tri*>& outCandidates
    );
    std::vector<Tri*> terrainTriCandidates;
};