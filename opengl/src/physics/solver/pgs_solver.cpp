#include "pch.h"
#include "pgs_solver.h"
#include "physics_step_types.h"
#include "rigidbody.h"
#include "narrowphase/collision_manifold.h"
#include "narrowphase/narrowphase_types.h"

void PGSSolver::resolveContacts(const StepScope& scope, ContactBatch& batch, const int PGSiterations, const float dt) {
    // #TODO: solver islands
    // för att undvika att lösa stora staplar av kontakter som inte påverkar varandra, vilket kan hända i t.ex. en pyramid av boxar där varje box har kontakt med flera
    // det påverkar också determinism 

    std::vector<Contact*>& contacts = batch.contacts;

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

    for (Contact* contact : contacts) {
        ContactRuntime& rt = contact->runtimeData;
        RigidBody* bodyA = rt.bodyA;
        RigidBody* bodyB = rt.bodyB;

        float contactSlop = defaultSlop;
        float contactBaumgarte = defaultBaumgarte;

        if (contact->noSolverResponseA || contact->noSolverResponseB) {
            contactSlop = noResponseSlop;
            contactBaumgarte = noResponseBaumgarte;
        }

        for (ContactPoint& cp : contact->points) {
            cp.accumulatedBiasImpulse = 0.0f;

            bool active = cp.depth > -persistentSlop;

            if (!active) {
                cp.accumulatedNormalImpulse = 0.0f;
                cp.accumulatedFrictionImpulse1 = 0.0f;
                cp.accumulatedFrictionImpulse2 = 0.0f;
                cp.accumulatedBiasImpulse = 0.0f;
                cp.biasVelocity = 0.0f;
                cp.active = false;
                continue;
            }

            cp.active = true;

            float allowed = cp.depth - contactSlop;

            if (allowed > 0.0f) {
                float correctionSpeed = (contactBaumgarte * allowed) / dt;
                cp.biasVelocity = -correctionSpeed;
            }
            else {
                cp.biasVelocity = 0.0f;
            }

            float maxFriction = staticFriction * cp.accumulatedNormalImpulse;

            float f1 = cp.accumulatedFrictionImpulse1;
            float f2 = cp.accumulatedFrictionImpulse2;
            float len2 = f1 * f1 + f2 * f2;
            float max2 = maxFriction * maxFriction;

            if (len2 > max2) {
                float len = std::sqrt(len2);
                if (len > 1e-6f) {
                    float s = maxFriction / len;
                    cp.accumulatedFrictionImpulse1 *= s;
                    cp.accumulatedFrictionImpulse2 *= s;
                }
            }

            glm::vec3 Pn = cp.accumulatedNormalImpulse * contact->normal;
            glm::vec3 Pt =
                cp.accumulatedFrictionImpulse1 * contact->t1 +
                cp.accumulatedFrictionImpulse2 * contact->t2;

            glm::vec3 J = Pn + Pt;

            if (contact->partnerTypeA == ContactPartnerType::RigidBody &&
                !contact->noSolverResponseA) {
                bodyA->applyImpulseLinear(-J);
                bodyA->applyImpulseAngular(-glm::cross(cp.rA, J));
            }

            if (contact->partnerTypeB == ContactPartnerType::RigidBody &&
                !contact->noSolverResponseB) {
                bodyB->applyImpulseLinear(J);
                bodyB->applyImpulseAngular(glm::cross(cp.rB, J));
            }
        }
    }

    // ------ PGS solver ------
    int iterationsUsed = 0;
    for (int i = 0; i < PGSiterations; i++) {
        float maxVelocityDelta = 0.0f;
        float maxBiasDelta = 0.0f;

        iterationsUsed++;

        // reverse order every other iteration to reduce directional bias in the solver
        int contactCount = static_cast<int>(contacts.size());

        for (int cc = 0; cc < contactCount; cc++) {
            int ci = (i % 2 == 0)
                ? cc
                : contactCount - 1 - cc;

            Contact* contact = contacts[ci];

            ContactRuntime& rt = contact->runtimeData;
            RigidBody* bodyA = rt.bodyA;
            RigidBody* bodyB = rt.bodyB;

            int count = static_cast<int>(contact->points.size());

            // reverse order every other iteration to reduce directional bias in the solver
            for (int jj = 0; jj < count; jj++) {
                int j = (i % 2 == 0) ? jj : (count - 1 - jj);

                ContactPoint& cp = contact->points[j];

                if (!cp.active) continue;

                // --- Relative velocity before normal solve ---
                glm::vec3 relVelA{ 0.0f };
                glm::vec3 relVelB{ 0.0f };
                glm::vec3 angVelA{ 0.0f };
                glm::vec3 angVelB{ 0.0f };

                if (contact->contributesMotionA) {
                    relVelA = bodyA->linearVelocity;
                    angVelA = bodyA->angularVelocity;
                }
                if (contact->contributesMotionB) {
                    relVelB = bodyB->linearVelocity;
                    angVelB = bodyB->angularVelocity;
                }

                glm::vec3 relativeVelocity =
                    (relVelB + glm::cross(angVelB, cp.rB)) -
                    (relVelA + glm::cross(angVelA, cp.rA));

                float normalVelocity = glm::dot(relativeVelocity, contact->normal);

                // ----- Normal impulse -----
                float v_target = cp.targetBounceVelocity;

                float J = -(normalVelocity - v_target) * cp.m_eff;

                float oldNormalImpulse = cp.accumulatedNormalImpulse;
                cp.accumulatedNormalImpulse = glm::max(oldNormalImpulse + J, 0.0f);

                float deltaImpulse = cp.accumulatedNormalImpulse - oldNormalImpulse;
                glm::vec3 deltaNormalImpulse = deltaImpulse * contact->normal;

                if (contact->partnerTypeA == ContactPartnerType::RigidBody && !contact->noSolverResponseA) {
                    bodyA->applyImpulseLinear(-deltaNormalImpulse);
                    bodyA->applyImpulseAngular(-glm::cross(cp.rA, deltaNormalImpulse));
                }
                if (contact->partnerTypeB == ContactPartnerType::RigidBody && !contact->noSolverResponseB) {
                    bodyB->applyImpulseLinear(deltaNormalImpulse);
                    bodyB->applyImpulseAngular(glm::cross(cp.rB, deltaNormalImpulse));
                }

                maxVelocityDelta = std::max(maxVelocityDelta, std::abs(deltaImpulse));

                // ----- Bias impulse (Baumgarte) -----
                if (cp.biasVelocity != 0.0f || cp.accumulatedBiasImpulse > 0.0f) {
                    glm::vec3 relVelA_bias{ 0.0f };
                    glm::vec3 relVelB_bias{ 0.0f };
                    glm::vec3 angVelA_bias{ 0.0f };
                    glm::vec3 angVelB_bias{ 0.0f };

                    if (contact->contributesMotionA) {
                        relVelA_bias = bodyA->biasLinearVelocity;
                        angVelA_bias = bodyA->biasAngularVelocity;
                    }
                    if (contact->contributesMotionB) {
                        relVelB_bias = bodyB->biasLinearVelocity;
                        angVelB_bias = bodyB->biasAngularVelocity;
                    }

                    glm::vec3 relativeBiasVelocity =
                        (relVelB_bias + glm::cross(angVelB_bias, cp.rB)) -
                        (relVelA_bias + glm::cross(angVelA_bias, cp.rA));

                    float normalBiasVelocity = glm::dot(relativeBiasVelocity, contact->normal);

                    float Jb = -(normalBiasVelocity + cp.biasVelocity) * cp.m_eff;

                    float oldB = cp.accumulatedBiasImpulse;
                    cp.accumulatedBiasImpulse = glm::max(oldB + Jb, 0.0f);

                    float deltaB = cp.accumulatedBiasImpulse - oldB;
                    glm::vec3 impulseB = deltaB * contact->normal;

                    if (contact->partnerTypeA == ContactPartnerType::RigidBody && !contact->noSolverResponseA) {
                        bodyA->pushBiasImpulseLinear(-impulseB);
                        glm::vec3 angularBiasA = -glm::cross(cp.rA, impulseB);
                        bodyA->pushBiasImpulseAngular(angularBiasScale * angularBiasA);
                    }
                    if (contact->partnerTypeB == ContactPartnerType::RigidBody && !contact->noSolverResponseB) {
                        bodyB->pushBiasImpulseLinear(impulseB);
                        glm::vec3 angularBiasB = glm::cross(cp.rB, impulseB);
                        bodyB->pushBiasImpulseAngular(angularBiasScale * angularBiasB);
                    }

                    maxBiasDelta = std::max(maxBiasDelta, std::abs(deltaB));
                }

                // ----- Friction -----
                if (cp.accumulatedNormalImpulse > normalImpulseEps) {
                    constexpr float recomputeThreshold = 1e-4f;

                    float v_t1, v_t2;

                    if (std::abs(deltaImpulse) > recomputeThreshold) {
                        // Recompute relative velocity after normal impulse
                        glm::vec3 relVelA2{ 0.0f };
                        glm::vec3 relVelB2{ 0.0f };
                        glm::vec3 angVelA2{ 0.0f };
                        glm::vec3 angVelB2{ 0.0f };

                        if (contact->contributesMotionA) {
                            relVelA2 = bodyA->linearVelocity;
                            angVelA2 = bodyA->angularVelocity;
                        }
                        if (contact->contributesMotionB) {
                            relVelB2 = bodyB->linearVelocity;
                            angVelB2 = bodyB->angularVelocity;
                        }

                        glm::vec3 relativeVelocity2 =
                            (relVelB2 + glm::cross(angVelB2, cp.rB)) -
                            (relVelA2 + glm::cross(angVelA2, cp.rA));

                        v_t1 = glm::dot(relativeVelocity2, contact->t1);
                        v_t2 = glm::dot(relativeVelocity2, contact->t2);
                    }
                    else {
                        // Reuse tangential velocity from old relative velocity
                        v_t1 = glm::dot(relativeVelocity, contact->t1);
                        v_t2 = glm::dot(relativeVelocity, contact->t2);
                    }

                    // Desired friction delta
                    float dF1 = -v_t1 * cp.invMassT1;
                    float dF2 = -v_t2 * cp.invMassT2;

                    // Candidate accumulated friction impulse
                    float newF1 = cp.accumulatedFrictionImpulse1 + dF1;
                    float newF2 = cp.accumulatedFrictionImpulse2 + dF2;

                    float Jn = std::abs(cp.accumulatedNormalImpulse);
                    float maxStatic = staticFriction * Jn;
                    float maxStatic2 = maxStatic * maxStatic;

                    float newLen2 = newF1 * newF1 + newF2 * newF2;
                    float dT = 0.0f;

                    if (newLen2 <= maxStatic2) {
                        // Static friction
                        cp.accumulatedFrictionImpulse1 = newF1;
                        cp.accumulatedFrictionImpulse2 = newF2;

                        glm::vec3 dFt = dF1 * contact->t1 + dF2 * contact->t2;
                        dT = std::sqrt(dF1 * dF1 + dF2 * dF2);

                        if (contact->partnerTypeA == ContactPartnerType::RigidBody && !contact->noSolverResponseA) {
                            bodyA->applyImpulseLinear(-dFt);
                            bodyA->applyImpulseAngular(-glm::cross(cp.rA, dFt));
                        }
                        if (contact->partnerTypeB == ContactPartnerType::RigidBody && !contact->noSolverResponseB) {
                            bodyB->applyImpulseLinear(dFt);
                            bodyB->applyImpulseAngular(glm::cross(cp.rB, dFt));
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

                            float d1 = clampedF1 - cp.accumulatedFrictionImpulse1;
                            float d2 = clampedF2 - cp.accumulatedFrictionImpulse2;

                            cp.accumulatedFrictionImpulse1 = clampedF1;
                            cp.accumulatedFrictionImpulse2 = clampedF2;

                            glm::vec3 dFt = d1 * contact->t1 + d2 * contact->t2;
                            dT = std::sqrt(d1 * d1 + d2 * d2);

                            if (contact->partnerTypeA == ContactPartnerType::RigidBody && !contact->noSolverResponseA) {
                                bodyA->applyImpulseLinear(-dFt);
                                bodyA->applyImpulseAngular(-glm::cross(cp.rA, dFt));
                            }
                            if (contact->partnerTypeB == ContactPartnerType::RigidBody && !contact->noSolverResponseB) {
                                bodyB->applyImpulseLinear(dFt);
                                bodyB->applyImpulseAngular(glm::cross(cp.rB, dFt));
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
            if (contact->contributesMotionA) angVelA = bodyA->angularVelocity;
            if (contact->contributesMotionB) angVelB = bodyB->angularVelocity;

            float v_twist = glm::dot((angVelB - angVelA), contact->normal);

            // 3) Friktionsbudget baserad på TOTAL normalimpuls i manifoldet
            float Jn_total = 0.0f;
            for (const ContactPoint& cp : contact->points) {
                Jn_total += std::abs(cp.accumulatedNormalImpulse);
            }

            float maxTwistImpulse = twistFriction * Jn_total;

            // 4) PGS-uppdatering (delta) för en enda twist-λ per manifold
            float oldTwist = contact->accumulatedTwistImpulse;
            float dLambda = -v_twist * contact->invMassTwist;
            float newTwist = glm::clamp(oldTwist + dLambda, -maxTwistImpulse, maxTwistImpulse);
            float delta = newTwist - oldTwist;
            contact->accumulatedTwistImpulse = newTwist;

            // 5) Applicera moment kring n
            glm::vec3 tau = delta * contact->normal;
            if (glm::dot(tau, tau) > 1e-6f) {
                if (contact->partnerTypeA == ContactPartnerType::RigidBody && !contact->noSolverResponseA) {
                    bodyA->applyImpulseAngular(-tau);
                }
                if (contact->partnerTypeB == ContactPartnerType::RigidBody && !contact->noSolverResponseB) {
                    bodyB->applyImpulseAngular(tau);
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

void PGSSolver::postSolve(const StepScope& scope, ContactBatch& batch, RuntimeCaches& caches, int currentFrame, const float dt) 
{
    std::vector<Contact*>& contacts = batch.contacts;

    // commit bias impulses so they affect velocity in the next frame's collision detection and solving, which improves stability especially for stacked objects
    for (Contact* contact : contacts) {
        RigidBody* bodies[2] = {
            contact->runtimeData.bodyA,
            contact->runtimeData.bodyB
        };

        for (RigidBody* body : bodies) {
            if (!body) continue;
            if (body->type == BodyType::Static) continue;
            if (body->lastBiasCommitFrame == currentFrame) continue;

            body->lastBiasCommitFrame = currentFrame;

            Transform& t = *caches.transforms.get(body->rootTransformHandle, FUNC_NAME);
            body->commitBiasImpulses(t, dt);
            t.updateCache();
            body->updateInertiaWorld(t);
        }
    }
}