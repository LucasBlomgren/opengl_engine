#include "pch.h"
#include "pgs_solver.h"
#include "substeps/physics_step_types.h"
#include "rigidbody.h"
#include "narrowphase/collision_manifold.h"
#include "narrowphase/narrowphase_types.h"

void PGSSolver::init() {
    solverBodies.reserve(1024);
    contactConstraints.reserve(1024);
    contactPoints.reserve(4096);
}

void PGSSolver::clear() {
    solverBodies.clear();
    contactConstraints.clear();
    contactPoints.clear();
    solverBodyIndexBySlot.clear();
    solverBodySources.clear();
    solverBodyWriteBack.clear();
    constraintSources.clear();
    pointSources.clear();
}

//================================================================
//  Main solver entry point: 
//  build solver data, resolve contacts, and post-solve updates
//================================================================
void PGSSolver::solve(
    const StepScope& scope,
    ContactBatch& batch, 
    RuntimeCaches& caches, 
    const int PGSiterations, 
    const float dt
) {
    buildSolverData(scope, batch, caches, dt);
    resolveContacts(PGSiterations, dt);
    postSolve(caches, dt);
}

//================================================================
//  Apply impulses to solver bodies
//================================================================
inline void PGSSolver::applyImpulseToSolverBody(
    SolverBody* body,
    const glm::vec3& linearImpulse,
    const glm::vec3& angularImpulse
) {
    body->linVelocity += linearImpulse * body->invMass;
    body->angVelocity += multiplyInvInertia(body, angularImpulse);
}

inline void PGSSolver::applyBiasImpulseToSolverBody(
    SolverBody* body,
    const glm::vec3& linearImpulse,
    const glm::vec3& angularImpulse
) {
    body->biasLinVelocity += linearImpulse * body->invMass;
    body->biasAngVelocity += multiplyInvInertia(body, angularImpulse);
}

inline glm::vec3 PGSSolver::multiplyInvInertia(
    const SolverBody* body,
    const glm::vec3& v
) {
    return glm::vec3(
        body->InvI[0] * v.x + body->InvI[3] * v.y + body->InvI[4] * v.z,
        body->InvI[3] * v.x + body->InvI[1] * v.y + body->InvI[5] * v.z,
        body->InvI[4] * v.x + body->InvI[5] * v.y + body->InvI[2] * v.z
    );
}

//=================================================================
//  Distinguish between body and terrain contacts
//=================================================================
SolverBody* PGSSolver::getSolverBodyOrNull(uint32_t idx) {
    if (idx == InvalidSolverBody) return nullptr;
    if (idx >= solverBodies.size()) return nullptr;
    return &solverBodies[idx];
}

//================================================================
//  Solve contacts using Projected Gauss-Seidel (PGS) method
//================================================================
void PGSSolver::resolveContacts(
    const int PGSiterations, 
    const float dt
) {
    // #TODO: solver islands
    // för att undvika att lösa stora staplar av kontakter som inte påverkar varandra, vilket kan hända i t.ex. en pyramid av boxar där varje box har kontakt med flera
    // det påverkar också determinism 

    constexpr float velocityImpulseEps = 1e-8f;
    constexpr float biasImpulseEps = 1e-8f;

    constexpr float normalImpulseEps = 1e-6f;

    constexpr float staticFriction = 0.6f;
    constexpr float dynamicFriction = 0.4f;
    constexpr float twistFriction = 0.1f;

    constexpr float defaultSlop = 0.0007f;
    constexpr float noResponseSlop = 0.0007f;

    constexpr float defaultBaumgarte = 0.3f;
    constexpr float noResponseBaumgarte = 0.6f;

    constexpr float persistentSlop = 0.005f;
    constexpr float angularBiasScale = 0.2f;

    // ------ PGS solver ------
    int iterationsUsed = 0;
    for (int i = 0; i < PGSiterations; i++) {
        float maxVelocityDelta = 0.0f;
        float maxBiasDelta = 0.0f;

        iterationsUsed++;

        // reverse order every other iteration to reduce directional bias in the solver
        int contactCount = static_cast<int>(contactConstraints.size());

        for (int cc = 0; cc < contactCount; cc++) {
            int ci = (i % 2 == 0)
                ? cc
                : contactCount - 1 - cc;

            ContactConstraint& contact = contactConstraints[ci];
            SolverBody* bodyA = getSolverBodyOrNull(contact.bodyA);
            SolverBody* bodyB = getSolverBodyOrNull(contact.bodyB);
            int count = static_cast<int>(contact.pointCount);

            // reverse order every other iteration to reduce directional bias in the solver
            for (int jj = 0; jj < count; jj++) {
                int j = (i % 2 == 0) ? jj : (count - 1 - jj);

                ContactConstraintPoint& cp = contactPoints[contact.firstPoint + j];

                // --- Relative velocity before normal solve ---
                glm::vec3 relVelA{ 0.0f };
                glm::vec3 relVelB{ 0.0f };
                glm::vec3 angVelA{ 0.0f };
                glm::vec3 angVelB{ 0.0f };

                if ((contact.flags & ContributesMotionA) && bodyA) {
                    relVelA = bodyA->linVelocity;
                    angVelA = bodyA->angVelocity;
                }
                if ((contact.flags & ContributesMotionB) && bodyB) {
                    relVelB = bodyB->linVelocity;
                    angVelB = bodyB->angVelocity;
                }

                glm::vec3 relativeVelocity =
                    (relVelB + glm::cross(angVelB, cp.rB)) -
                    (relVelA + glm::cross(angVelA, cp.rA));

                float normalVelocity = glm::dot(relativeVelocity, contact.normal);

                // ----- Normal impulse -----
                float v_target = cp.targetBounce;

                float J = -(normalVelocity - v_target) * cp.m_eff;

                float oldNormalImpulse = cp.nImpulse;
                cp.nImpulse = glm::max(oldNormalImpulse + J, 0.0f);

                float deltaImpulse = cp.nImpulse - oldNormalImpulse;
                glm::vec3 deltaNormalImpulse = deltaImpulse * contact.normal;

                if ((contact.flags & CanApplyImpulseA) != 0 && bodyA) {
                    applyImpulseToSolverBody(bodyA, -deltaNormalImpulse, -glm::cross(cp.rA, deltaNormalImpulse));
                }
                if ((contact.flags & CanApplyImpulseB) && bodyB) {
                    applyImpulseToSolverBody(bodyB, deltaNormalImpulse, glm::cross(cp.rB, deltaNormalImpulse));
                }

                maxVelocityDelta = std::max(maxVelocityDelta, std::abs(deltaImpulse));

                // ----- Bias impulse (Baumgarte) -----
                if (cp.biasVelocity != 0.0f || cp.biasImpulse > 0.0f) {
                    glm::vec3 relVelA_bias{ 0.0f };
                    glm::vec3 relVelB_bias{ 0.0f };
                    glm::vec3 angVelA_bias{ 0.0f };
                    glm::vec3 angVelB_bias{ 0.0f };

                    if ((contact.flags & ContributesMotionA) && bodyA) {
                        relVelA_bias = bodyA->biasLinVelocity;
                        angVelA_bias = bodyA->biasAngVelocity;
                    }
                    if ((contact.flags & ContributesMotionB) && bodyB) {
                        relVelB_bias = bodyB->biasLinVelocity;
                        angVelB_bias = bodyB->biasAngVelocity;
                    }

                    glm::vec3 relativeBiasVelocity =
                        (relVelB_bias + glm::cross(angVelB_bias, cp.rB)) -
                        (relVelA_bias + glm::cross(angVelA_bias, cp.rA));

                    float normalBiasVelocity = glm::dot(relativeBiasVelocity, contact.normal);

                    float Jb = -(normalBiasVelocity + cp.biasVelocity) * cp.m_eff;

                    float oldB = cp.biasImpulse;
                    cp.biasImpulse = glm::max(oldB + Jb, 0.0f);

                    float deltaB = cp.biasImpulse - oldB;
                    glm::vec3 impulseB = deltaB * contact.normal;

                    if ((contact.flags & CanApplyImpulseA) && bodyA) {
                        glm::vec3 angularBiasA = -glm::cross(cp.rA, impulseB);

                        applyBiasImpulseToSolverBody(
                            bodyA,
                            -impulseB,
                            angularBiasScale * angularBiasA
                        );
                    }

                    if ((contact.flags & CanApplyImpulseB) && bodyB) {
                        glm::vec3 angularBiasB = glm::cross(cp.rB, impulseB);

                        applyBiasImpulseToSolverBody(
                            bodyB,
                            impulseB,
                            angularBiasScale * angularBiasB
                        );
                    }

                    maxBiasDelta = std::max(maxBiasDelta, std::abs(deltaB));
                }

                // ----- Friction -----
                if (cp.nImpulse > normalImpulseEps) {
                    constexpr float recomputeThreshold = 1e-4f;

                    float v_t1, v_t2;

                    if (std::abs(deltaImpulse) > recomputeThreshold) {
                        // Recompute relative velocity after normal impulse
                        glm::vec3 relVelA2{ 0.0f };
                        glm::vec3 relVelB2{ 0.0f };
                        glm::vec3 angVelA2{ 0.0f };
                        glm::vec3 angVelB2{ 0.0f };

                        if ((contact.flags & ContributesMotionA) && bodyA) {
                            relVelA2 = bodyA->linVelocity;
                            angVelA2 = bodyA->angVelocity;
                        }

                        if ((contact.flags & ContributesMotionB) && bodyB) {
                            relVelB2 = bodyB->linVelocity;
                            angVelB2 = bodyB->angVelocity;
                        }

                        glm::vec3 relativeVelocity2 =
                            (relVelB2 + glm::cross(angVelB2, cp.rB)) -
                            (relVelA2 + glm::cross(angVelA2, cp.rA));

                        v_t1 = glm::dot(relativeVelocity2, contact.t1);
                        v_t2 = glm::dot(relativeVelocity2, contact.t2);
                    }
                    else {
                        // Reuse tangential velocity from old relative velocity
                        v_t1 = glm::dot(relativeVelocity, contact.t1);
                        v_t2 = glm::dot(relativeVelocity, contact.t2);
                    }

                    // Desired friction delta
                    float dF1 = -v_t1 * cp.invMassT1;
                    float dF2 = -v_t2 * cp.invMassT2;

                    // Candidate accumulated friction impulse
                    float newF1 = cp.fImpulse1 + dF1;
                    float newF2 = cp.fImpulse2 + dF2;

                    float Jn = std::abs(cp.nImpulse);
                    float maxStatic = staticFriction * Jn;
                    float maxStatic2 = maxStatic * maxStatic;

                    float newLen2 = newF1 * newF1 + newF2 * newF2;
                    float dT = 0.0f;

                    if (newLen2 <= maxStatic2) {
                        // Static friction
                        cp.fImpulse1 = newF1;
                        cp.fImpulse2 = newF2;

                        glm::vec3 dFt = dF1 * contact.t1 + dF2 * contact.t2;
                        dT = std::sqrt(dF1 * dF1 + dF2 * dF2);

                        if ((contact.flags & CanApplyImpulseA) && bodyA) {
                            applyImpulseToSolverBody(
                                bodyA,
                                -dFt,
                                -glm::cross(cp.rA, dFt)
                            );
                        }
                        if ((contact.flags & CanApplyImpulseB) && bodyB) {
                            applyImpulseToSolverBody(
                                bodyB,
                                dFt,
                                glm::cross(cp.rB, dFt)
                            );
                        }
                    }
                    else {
                        // Dynamic friction
                        float maxDyn = dynamicFriction * Jn;
                        float len = std::sqrt(newLen2);

                        if (len > 1e-6f) {
                            float s = maxDyn / len;
                            float clampedF1 = newF1 * s;
                            float clampedF2 = newF2 * s;

                            float d1 = clampedF1 - cp.fImpulse1;
                            float d2 = clampedF2 - cp.fImpulse2;

                            cp.fImpulse1 = clampedF1;
                            cp.fImpulse2 = clampedF2;

                            glm::vec3 dFt = d1 * contact.t1 + d2 * contact.t2;
                            dT = std::sqrt(d1 * d1 + d2 * d2);

                            if ((contact.flags & CanApplyImpulseA) && bodyA) {
                                applyImpulseToSolverBody(
                                    bodyA,
                                    -dFt,
                                    -glm::cross(cp.rA, dFt)
                                );
                            }
                            if ((contact.flags & CanApplyImpulseB) && bodyB) {
                                applyImpulseToSolverBody(
                                    bodyB,
                                    dFt,
                                    glm::cross(cp.rB, dFt)
                                );
                            }
                        }
                    }

                    maxVelocityDelta = std::max(maxVelocityDelta, std::abs(dT));
                }
            }

            // ---------- Twist friction (per manifold) ----------
            // 1) Relativ rotationshastighet kring normalen
            glm::vec3 angVelA{ 0.0f };
            glm::vec3 angVelB{ 0.0f };
            if ((contact.flags & ContributesMotionA) && bodyA) {
                angVelA = bodyA->angVelocity;
            }
            if ((contact.flags & ContributesMotionB) && bodyB) {
                angVelB = bodyB->angVelocity;
            }

            float v_twist = glm::dot((angVelB - angVelA), contact.normal);

            // 3) Friktionsbudget baserad på TOTAL normalimpuls i manifoldet
            float Jn_total = 0.0f;
            for (int jj = 0; jj < contact.pointCount; jj++) {
                ContactConstraintPoint& cp = contactPoints[contact.firstPoint + jj];
                Jn_total += std::abs(cp.nImpulse);
            }

            float maxTwistImpulse = twistFriction * Jn_total;

            // 4) PGS-uppdatering (delta) för en enda twist-λ per manifold
            float oldTwist = contact.accumulatedTwistImpulse;
            float dLambda = -v_twist * contact.invMassTwist;
            float newTwist = glm::clamp(oldTwist + dLambda, -maxTwistImpulse, maxTwistImpulse);
            float delta = newTwist - oldTwist;
            contact.accumulatedTwistImpulse = newTwist;

            // 5) Applicera moment kring n
            glm::vec3 tau = delta * contact.normal;
            if (glm::dot(tau, tau) > 1e-6f) 
            {
                if ((contact.flags & CanApplyImpulseA) != 0 && bodyA) {
                    SolverBody* bodyA = getSolverBodyOrNull(contact.bodyA);
                    applyImpulseToSolverBody(bodyA, glm::vec3(0.0f), -tau);
                }
                if ((contact.flags & CanApplyImpulseB) != 0 && bodyB) {
                    SolverBody* bodyB = getSolverBodyOrNull(contact.bodyB);
                    applyImpulseToSolverBody(bodyB, glm::vec3(0.0f), tau);
                }
            }
        }

        if (maxVelocityDelta < velocityImpulseEps &&
            maxBiasDelta < biasImpulseEps)
        {
            break;
        }
    }
}

//===========================================================================
//  Post-solve: write back solver results to original Contact and RigidBody
//===========================================================================
void PGSSolver::postSolve(
    RuntimeCaches& caches,
    const float dt
) {
    // 1. Skriv tillbaka ackumulerade impulser till original ContactPoint.
    for (size_t i = 0; i < contactPoints.size(); i++) {
        ContactPoint* dst = pointSources[i];
        if (!dst) {
            continue;
        }

        const ContactConstraintPoint& src = contactPoints[i];

        dst->accumulatedNormalImpulse = src.nImpulse;
        dst->accumulatedFrictionImpulse1 = src.fImpulse1;
        dst->accumulatedFrictionImpulse2 = src.fImpulse2;
        dst->accumulatedBiasImpulse = src.biasImpulse;
        dst->biasVelocity = src.biasVelocity;
    }

    // 2. Skriv tillbaka twist impulse till original Contact.
    for (size_t i = 0; i < contactConstraints.size(); i++) {
        Contact* dst = constraintSources[i];
        if (!dst) {
            continue;
        }

        const ContactConstraint& src = contactConstraints[i];

        dst->accumulatedTwistImpulse = src.accumulatedTwistImpulse;
    }

    // 3. Skriv tillbaka solver velocities till RigidBody.
    for (size_t i = 0; i < solverBodies.size(); i++) {
        if (!solverBodyWriteBack[i]) {
            continue;
        }

        RigidBody* body = solverBodySources[i];
        if (!body) {
            continue;
        }

        if (body->type == BodyType::Static) {
            continue;
        }

        SolverBody& sb = solverBodies[i];

        body->linearVelocity = sb.linVelocity;
        body->angularVelocity = sb.angVelocity;

        body->biasLinearVelocity = sb.biasLinVelocity;
        body->biasAngularVelocity = sb.biasAngVelocity;

        bool hasBias =
            glm::dot(sb.biasLinVelocity, sb.biasLinVelocity) > 1e-12f ||
            glm::dot(sb.biasAngVelocity, sb.biasAngVelocity) > 1e-12f;

        if (hasBias) {
            Transform& t = *caches.transforms.get(
                body->rootTransformHandle,
                FUNC_NAME
            );

            body->commitBiasImpulses(t, dt);
            t.updateCache();
            body->updateInertiaWorld(t);
        }
    }
}