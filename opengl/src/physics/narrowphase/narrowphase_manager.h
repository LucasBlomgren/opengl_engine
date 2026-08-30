#pragma once

#include <optional>
#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "narrowphase_types.h"
#include "collision_manifold.h"

#include "physics/world/physics_world.h"
#include "physics/broadphase/contact_types.h"

namespace physics::internal {

class NarrowphaseManager {
public:
    void init(
        PhysicsWorld* physicsWorld,
        CollisionManifold* collisionManifold,
        std::vector<DebugSpeculativeContact>* debugSpeculativeContacts,
        std::unordered_map<size_t, Contact>* contactCache,
        std::vector<BodyHandle>* toWake
    );
    void clear();

    const std::vector<ExternalMotionContact>& getExternalContacts() const {
        return externalContacts;
    }

    // main function
    void narrowPhase(
        const PairBatch& pairs,
        ContactBatch& batch,
        float dt
    );

private:
    // references to external systems
    PhysicsWorld* physicsWorld = nullptr;
    CollisionManifold* collisionManifold = nullptr;
    std::vector<DebugSpeculativeContact>* debugSpeculativeContacts = nullptr;
    std::unordered_map<size_t, Contact>* contactCache = nullptr;

    std::unordered_set<PairKey, PairKeyHash> normalHitPairs;
    std::vector<PendingSweepHit> pendingSweepHits;

    //=======================================================
    //     Dispatching to specific pair processing functions
    //=======================================================
    void processTerrainPairs(
        const TerrainPair& terrainPair,
        ContactBatch& batch,
        float dt
    );
    void processDynamicPairs(
        const DynamicPair& pair,
        ContactBatch& batch,
        float dt);

    void processSpeculativeDynamicPairs(

        const SpeculativeDynamicPair& pair,
        float dt
    );
    void processSpeculativeTerrainPairs(
        const SpeculativeTerrainPair& pair,
        float dt
    );

    void processColliderPairNormal(
        ContactBatch& batch,
        ResolvedColliderPair pair
    );

    void processColliderPairSpeculative(
        ResolvedColliderPair pair,
        float dt,
        BodyHandle sweepOwner
    );

    //=======================================================
    //     SAT tests
    //=======================================================
    std::optional<OverlapHit> tryBoxBox(ResolvedColliderPair pair);
    std::optional<OverlapHit> tryBoxSphere(ResolvedColliderPair pair);
    std::optional<OverlapHit> trySphereSphere(ResolvedColliderPair pair);

    std::optional<SweepHit> trySpeculativeBoxBox(
        ResolvedColliderPair pair,
        float dt,
        BodyHandle sweepOwner
    );
    std::optional<SweepHit> trySpeculativeBoxSphere(
        ResolvedColliderPair pair,
        float dt,
        BodyHandle sweepOwner
    );
    std::optional<TerrainSweepHit> trySpeculativeBoxTriangle(
        ColliderEndpointRef collider,
        Tri* tri,
        float dt,
        BodyHandle sweepOwner
    );
    std::optional<SweepHit> trySpeculativeSphereSphere(
        ResolvedColliderPair pair,
        float dt,
        BodyHandle sweepOwner
    );
    std::optional<TerrainSweepHit> trySpeculativeSphereTriangle(
        ColliderEndpointRef collider,
        Tri* tri,
        float dt,
        BodyHandle sweepOwner
    );

    //=======================================================
    //     Contact emission
    //=======================================================
    void emitRigidContact(
        ContactBatch& batch,
        OverlapHit& hit
    );

    void emitSpeculativeContact(
        ContactBatch& batch,
        const SweepHit& hit
    );

    void emitSpeculativeContact(
        ContactBatch& batch,
        const TerrainSweepHit& hit
    );

    void emitSpeculativeContact(
        ContactBatch& batch,
        const ColliderEndpointRef& a,
        const ColliderEndpointRef* b,
        ContactPartnerType partnerTypeB,
        const SAT::Result& geometry
    );

    void flushPendingSweepHits(ContactBatch& batch);

    bool tryExportExternalContact(
        const ResolvedColliderPair& pair,
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
        OverlapHit& hit
    );

    PairKey makeColliderPairKey(
        ColliderHandle a,
        ColliderHandle b
    );
    uint64_t packColliderHandle(ColliderHandle handle);
    uint64_t packBodyHandle(BodyHandle h);

    //=======================================================
    //     Terrain contact processing
    //=======================================================
    void processTerrainTriBox(
        ContactBatch& batch,
        BodyHandle bodyH,
        Collider* collider,
        RigidBody* body,
        const std::vector<Tri*>& candidates
    );

    void processTerrainTriSphere(
        ContactBatch& batch,
        BodyHandle bodyH,
        Collider* collider,
        RigidBody* body,
        const std::vector<Tri*>& candidates
    );

    // dynamic vs dynamic contact runtime data
    ContactRuntime makeRuntimeData(
        RigidBody* bodyA, RigidBody* bodyB,
        Collider* colliderA, Collider* colliderB
    ) const;

    // dynamic vs terrain contact runtime data
    ContactRuntime makeRuntimeData(
        RigidBody* bodyA,
        Collider* colliderA
    ) const;

    // for wake-up requests for bodies that should be 
    // woken up after processing dynamic pairs
    std::vector<BodyHandle>* toWake = nullptr; 

    // for storing multiple SAT results for 
    // a single collider vs terrain pair
    std::vector<SAT::Result> SAT_resultsList;

    // contacts to be sent to character controller 
    // for external motion handling
    std::vector<ExternalMotionContact> externalContacts;

    // Helpers
    glm::vec3 getAvgNormal(const std::vector<SAT::Result>& SAT_resultsList) const;

    void collectTerrainTriCandidates(
        Collider& collider,
        const std::vector<Tri*>& inputTris,
        std::vector<Tri*>& outCandidates
    );
    std::vector<Tri*> terrainTriCandidates;
};

}
