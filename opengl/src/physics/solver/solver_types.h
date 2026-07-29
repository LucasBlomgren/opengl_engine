#pragma once

namespace physics::internal {

static constexpr uint32_t InvalidSolverBody = std::numeric_limits<uint32_t>::max();

struct SolverBody {
    glm::vec3 linVelocity;
    glm::vec3 angVelocity;
    glm::vec3 biasLinVelocity;
    glm::vec3 biasAngVelocity;
    float invMass;
    float InvI[6];
};

enum ContactFlags : uint8_t {
    ContributesMotionA = 1 << 0,
    ContributesMotionB = 1 << 1,
    NoSolverResponseA = 1 << 2,
    NoSolverResponseB = 1 << 3,
    CanApplyImpulseA = 1 << 4,
    CanApplyImpulseB = 1 << 5,
};
struct ContactConstraint {
    uint32_t bodyA = InvalidSolverBody; // index into solverBodies
    uint32_t bodyB = InvalidSolverBody;
    glm::vec3 normal{ 0.0f };
    glm::vec3 t1{ 0.0f };
    glm::vec3 t2{ 0.0f };
    float invMassTwist = 0.0f;
    float accumulatedTwistImpulse = 0.0f;
    uint32_t firstPoint = 0; // index into contactPoints
    uint8_t pointCount = 0;  // number of contact points in this constraint
    uint8_t flags = 0;       // ContactFlags as bitmask
}; // 60 bytes = 1 cache line

struct ContactConstraintPoint {
    glm::vec3 rA{ 0.0f };
    glm::vec3 rB{ 0.0f };
    float m_eff = 0.0f;
    float invMassT1 = 0.0f;
    float invMassT2 = 0.0f;
    float targetBounce = 0.0f;
    float biasVelocity = 0.0f;
    float nImpulse = 0.0f;
    float fImpulse1 = 0.0f;
    float fImpulse2 = 0.0f;
    float biasImpulse = 0.0f;
}; // 60 bytes = 1 cache line

struct SpeculativeConstraint {
    uint32_t bodyA = InvalidSolverBody;
    uint32_t bodyB = InvalidSolverBody;
    glm::vec3 normal{ 0.0f };
    float separation = 0.0f;
    float accumulatedImpulse = 0.0f;

    // stored before speculative solve for post-restitution velocity calculation
    float incomingNormalVelocity = 0.0f; 
    float restitution = 0.1f;

    uint8_t flags = 0;
}; // 44 bytes = 1 cache line

}
