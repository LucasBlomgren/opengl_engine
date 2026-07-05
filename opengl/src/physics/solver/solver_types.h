#pragma once

#include <glm/glm.hpp>

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
    uint32_t bodyA; // index into solverBodies
    uint32_t bodyB;
    glm::vec3 normal;
    glm::vec3 t1;
    glm::vec3 t2;
    float invMassTwist;
    float accumulatedTwistImpulse;
    uint32_t firstPoint; // index into contactPoints
    uint8_t pointCount;  // number of contact points in this constraint
    uint8_t flags;       // ContactFlags as bitmask
}; // 60 bytes = 1 cache line

struct ContactConstraintPoint {
    glm::vec3 rA;
    glm::vec3 rB;
    float m_eff;
    float invMassT1;
    float invMassT2;
    float targetBounce;
    float biasVelocity;
    float nImpulse;
    float fImpulse1;
    float fImpulse2;
    float biasImpulse;
}; // 60 bytes = 1 cache line