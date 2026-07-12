#pragma once

#include "narrowphase_types.h"
#include "runtime_caches.h"
#include "rigidbody.h"
#include "collision_manifold.h"
#include "sat.h"
#include "broadphase/broadphase_types.h"

class NarrowphaseManager {
public:
    void init(
        CollisionManifold* collisionManifold,
        std::unordered_map<size_t, Contact>* contactCache,
        RuntimeCaches* caches,
        std::vector<RigidBodyHandle>* toWake
    );
    void clear();

    std::vector<ExternalMotionContact>& getExternalContacts() {
        return externalContacts;
    }

    // main function
    void narrowPhase(
        const PairBatch& pairs,
        ContactBatch& batch,
        float dt
    );

private:
    // references to caches
    CollisionManifold* collisionManifold = nullptr;
    std::unordered_map<size_t, Contact>* contactCache = nullptr;
    RuntimeCaches* caches = nullptr;

    //=======================================================
    //     Dispatching to specific pair processing functions
    //=======================================================
    void processTerrainPair(const TerrainPair& terrainPair, ContactBatch& batch, float dt, NarrowphasePass pass);
    void processDynamicPair(const DynamicPair& dynamicPair, ContactBatch& batch, float dt, NarrowphasePass pass);

    void processColliderPair(
        ContactBatch& batch,
        ContactBuildInput in,
        float dt,
        NarrowphasePass pass
    );

    //=======================================================
    //     SAT tests
    //=======================================================
    bool tryBoxBox(ContactBuildInput& in, DynamicContactCandidate& out);
    bool tryBoxSphere(ContactBuildInput& in, DynamicContactCandidate& out);
    bool trySphereSphere(ContactBuildInput& in, DynamicContactCandidate& out);

    bool trySpeculativeBoxBox(ContactBuildInput& in, DynamicContactCandidate& out, float dt);
    bool trySpeculativeBoxSphere(ContactBuildInput& in, DynamicContactCandidate& out, float dt);
    bool trySpeculativeSphereSphere(ContactBuildInput& in, DynamicContactCandidate& out, float dt);

    //=======================================================
    //     Contact emission
    //=======================================================
    void emitRigidContact(
        ContactBatch& batch,
        ContactBuildInput& in,
        DynamicContactCandidate& candidate
    );

    bool tryExportExternalContact(
        const ContactBuildInput& in,
        const SAT::Result& satResult
    );

    bool computeContributesMotion(
        ContactPartnerType partnerType,
        const RigidBody& body,
        bool willWake
    ) const;

    bool computeNoSolverResponse(
        const RigidBody& body,
        bool willWake
    ) const;

    Contact* createManifold(
        Contact& contact,
        DynamicContactCandidate& candidate
    );

    //=======================================================
    //     Terrain contact processing
    //=======================================================
    void processTerrainTriBox(
        ContactBatch& batch,
        RigidBodyHandle bodyH, 
        Collider* collider, 
        RigidBody* body,
        const std::vector<Tri*>& candidates
    );

    void processTerrainTriSphere(
        ContactBatch& batch,
        RigidBodyHandle bodyH, 
        Collider* collider, 
        RigidBody* body,
        const std::vector<Tri*>& candidates
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