#pragma once

#include "solver_types.h"
#include <narrowphase/narrowphase_types.h>

class PGSSolver {
public:
    void init();
    void clear();

    void solve(
        ContactBatch& batch,
        RuntimeCaches& caches,
        const int PGSiterations,
        const float dt
    );

private:
    static constexpr uint32_t InvalidSolverBody = std::numeric_limits<uint32_t>::max();

    std::vector<SolverBody> solverBodies;          // hot data
    std::vector<ContactConstraint> contactConstraints;
    std::vector<ContactConstraintPoint> contactPoints;
    std::vector<uint32_t> solverBodyIndexBySlot; // maps RigidBody slot index to solverBodies index

    std::vector<RigidBody*> solverBodySources;
    std::vector<uint8_t> solverBodyWriteBack; // yes or no

    std::vector<Contact*> constraintSources;
    std::vector<ContactPoint*> pointSources;

    // solver_builder.cpp
    // Builds the solver data structures from the contact batch and runtime caches.
    void buildSolverData(
        ContactBatch& batch, 
        RuntimeCaches& caches,
        float dt
    );
    uint32_t getOrCreateSolverBody(RigidBodyHandle bodyHandle, RigidBody* body);
    void makeSolverBody(RigidBody* body);
    static void packInvInertia(const glm::mat3& m, float out[6]);

    bool prepareContactPointBaumgarte(
        const ContactConstraint& contact,
        ContactPoint& src,
        ContactConstraintPoint& dst,
        float dt
    );

    void warmStartContactPoint(
        const ContactConstraint& contact,
        const ContactConstraintPoint& cp
    );

    void applyImpulseToSolverBody(
        SolverBody* body,
        const glm::vec3& linearImpulse,
        const glm::vec3& angularImpulse
    );

    void applyBiasImpulseToSolverBody(
        SolverBody* body,
        const glm::vec3& linearImpulse,
        const glm::vec3& angularImpulse
    );

    glm::vec3 multiplyInvInertia(
        const SolverBody* body,
        const glm::vec3& v
    );

    SolverBody* getSolverBodyOrNull(uint32_t idx);
    static bool hasFlag(uint8_t flags, uint8_t flag);

    // solve constraints using Projected Gauss-Seidel (PGS) method
    void resolveContacts(
        const int PGSiterations,
        const float dt
    );

    // post-solve: update RigidBody velocities and bias velocities based on solver results
    void postSolve(
        RuntimeCaches& caches, 
        const float dt
    );

};