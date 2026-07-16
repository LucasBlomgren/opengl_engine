#include "pch.h"
#include "pgs_solver.h"

//=====================================================================
//  Build solver data structures from contact batch and runtime caches
//=====================================================================
void PGSSolver::buildSolverData(
    ContactBatch& batch,
    RuntimeCaches& caches,
    float dt
) {
    solverBodies.clear();
    contactConstraints.clear();
    contactPoints.clear();
    speculativeConstraints.clear();

    solverBodyIndexBySlot.clear();
    solverBodySources.clear();
    solverBodyWriteBack.clear();
    constraintSources.clear();
    pointSources.clear();

    // #TODO: bör vara touched bodies i scope, inte alla bodies i caches
    // om slot_capacity() är mycket större än antalet aktiva bodies, nollställs
    // mer än nödvändigt varje frame.
    size_t slotCap = caches.bodies.sm->slot_capacity();
    solverBodyIndexBySlot.assign(slotCap, InvalidSolverBody);

    solverBodies.reserve(caches.bodies.sm->dense().size());
    contactConstraints.reserve(batch.contacts.size());
    contactPoints.reserve(batch.contacts.size() * 4);
    speculativeConstraints.reserve(batch.speculativeContacts.size());

    solverBodySources.reserve(caches.bodies.sm->dense().size());
    solverBodyWriteBack.reserve(caches.bodies.sm->dense().size());
    constraintSources.reserve(batch.contacts.size());
    pointSources.reserve(batch.contacts.size() * 4);

    // #TODO: inte super effektivt att loopa igenom en linked list av contacts. 
    // Kan optimera senare om det behövs.
    for (Contact* contact : batch.contacts) {
        if (!contact) {
            continue;
        }

        ContactRuntime& rt = contact->runtimeData;

        uint32_t bodyAIndex = InvalidSolverBody;
        uint32_t bodyBIndex = InvalidSolverBody;

        if (contact->partnerTypeA == ContactPartnerType::RigidBody && rt.bodyA) {
            bodyAIndex = getOrCreateSolverBody(contact->bodyA, rt.bodyA);
        }
        if (contact->partnerTypeB == ContactPartnerType::RigidBody && rt.bodyB) {
            bodyBIndex = getOrCreateSolverBody(contact->bodyB, rt.bodyB);
        }

        ContactConstraint cc{};

        cc.bodyA = bodyAIndex;
        cc.bodyB = bodyBIndex;

        cc.normal = contact->normal;
        cc.t1 = contact->t1;
        cc.t2 = contact->t2;

        cc.invMassTwist = contact->invMassTwist;
        cc.accumulatedTwistImpulse = contact->accumulatedTwistImpulse;
        cc.firstPoint = static_cast<uint32_t>(contactPoints.size());

        if (bodyAIndex != InvalidSolverBody && contact->contributesMotionA) {
            cc.flags |= ContributesMotionA;
        }
        if (bodyBIndex != InvalidSolverBody && contact->contributesMotionB) {
            cc.flags |= ContributesMotionB;
        }
        if (contact->noSolverResponseA) cc.flags |= NoSolverResponseA;
        if (contact->noSolverResponseB) cc.flags |= NoSolverResponseB;

        if (bodyAIndex != InvalidSolverBody &&
            contact->partnerTypeA == ContactPartnerType::RigidBody &&
            !contact->noSolverResponseA)
        {
            cc.flags |= CanApplyImpulseA;
            solverBodyWriteBack[bodyAIndex] = 1;
        }

        if (bodyBIndex != InvalidSolverBody &&
            contact->partnerTypeB == ContactPartnerType::RigidBody &&
            !contact->noSolverResponseB)
        {
            cc.flags |= CanApplyImpulseB;
            solverBodyWriteBack[bodyBIndex] = 1;
        }

        for (size_t i = 0; i < contact->numPoints; ++i) {
            ContactPoint& cp = contact->points[i];
            ContactConstraintPoint& ccp = contactPoints.emplace_back();

            bool active = prepareContactPointBaumgarte(cc, cp, ccp, dt);

            if (!active) {
                contactPoints.pop_back();
                continue;
            }

            warmStartContactPoint(cc, ccp);

            pointSources.push_back(&cp);
            cc.pointCount++;
        }

        if (cc.pointCount > 0) {
            contactConstraints.push_back(cc);
            constraintSources.push_back(contact);
        }
    }

    for (const SpeculativeContact& src : batch.speculativeContacts) {
        uint32_t bodyAIndex = InvalidSolverBody;
        uint32_t bodyBIndex = InvalidSolverBody;

        if (src.partnerTypeA == ContactPartnerType::RigidBody && src.bodyA) {
            bodyAIndex = getOrCreateSolverBody(src.bodyHandleA, src.bodyA);
        }

        if (src.partnerTypeB == ContactPartnerType::RigidBody && src.bodyB) {
            bodyBIndex = getOrCreateSolverBody(src.bodyHandleB, src.bodyB);
        }

        SpeculativeConstraint c{};
        c.bodyA = bodyAIndex;
        c.bodyB = bodyBIndex;

        c.normal = src.normal;
        c.separation = src.separation;

        // Compute incoming normal velocity for post-solve restitution calculation
        glm::vec3 vA{ 0.0f };
        glm::vec3 vB{ 0.0f };
        if (src.partnerTypeA == ContactPartnerType::RigidBody &&
            src.bodyA &&
            src.contributesMotionA) 
        {
            vA = src.bodyA->linearVelocity;
        }
        if (src.partnerTypeB == ContactPartnerType::RigidBody &&
            src.bodyB &&
            src.contributesMotionB) 
        {
            vB = src.bodyB->linearVelocity;
        }

        glm::vec3 relativeVelocity = vB - vA;
        c.incomingNormalVelocity = glm::dot(relativeVelocity, c.normal);

        // No cross-frame warmstart for MVP.
        c.accumulatedImpulse = 0.0f;

        if (bodyAIndex != InvalidSolverBody && src.contributesMotionA) {
            c.flags |= ContributesMotionA;
        }
        if (bodyBIndex != InvalidSolverBody && src.contributesMotionB) {
            c.flags |= ContributesMotionB;
        }
        if (src.noSolverResponseA) c.flags |= NoSolverResponseA;
        if (src.noSolverResponseB) c.flags |= NoSolverResponseB;
        if (bodyAIndex != InvalidSolverBody &&
            src.partnerTypeA == ContactPartnerType::RigidBody &&
            !src.noSolverResponseA)
        {
            c.flags |= CanApplyImpulseA;
            solverBodyWriteBack[bodyAIndex] = 1;
        }
        if (bodyBIndex != InvalidSolverBody &&
            src.partnerTypeB == ContactPartnerType::RigidBody &&
            !src.noSolverResponseB)
        {
            c.flags |= CanApplyImpulseB;
            solverBodyWriteBack[bodyBIndex] = 1;
        }

        if ((c.flags & (CanApplyImpulseA | CanApplyImpulseB)) == 0) {
            continue;
        }

        speculativeConstraints.push_back(c);
    }
}

//==========================================================
//  Returns the index of the solver body in solverBodies, 
//  or InvalidSolverBody if the body is null.
//==========================================================
uint32_t PGSSolver::getOrCreateSolverBody(
    RigidBodyHandle bodyHandle, 
    RigidBody* body) 
{
    if (!body) {
        return InvalidSolverBody;
    }

    uint32_t& cachedIndex = solverBodyIndexBySlot[bodyHandle.slot];

    if (cachedIndex != InvalidSolverBody) {
        return cachedIndex;
    }

    uint32_t newIndex = static_cast<uint32_t>(solverBodies.size());

    makeSolverBody(body);

    solverBodySources.push_back(body);
    solverBodyWriteBack.push_back(0);

    cachedIndex = newIndex;
    return newIndex;
}
void PGSSolver::makeSolverBody(RigidBody* body) {
    SolverBody& sb = solverBodies.emplace_back();
    sb.linVelocity = body->linearVelocity;
    sb.angVelocity = body->angularVelocity;
    sb.biasLinVelocity = glm::vec3(0.0f);
    sb.biasAngVelocity = glm::vec3(0.0f);
    sb.invMass = body->invMass;
    packInvInertia(body->invInertiaWorld, sb.InvI);
}
void PGSSolver::packInvInertia(const glm::mat3& m, float out[6]) {
    out[0] = m[0][0]; // xx
    out[1] = m[1][1]; // yy
    out[2] = m[2][2]; // zz

    out[3] = m[0][1]; // xy
    out[4] = m[0][2]; // xz
    out[5] = m[1][2]; // yz
}

//===================================================================================
//  Prepare contact point for Baumgarte stabilization and warm starting.
//  Returns true if the contact point is active and should be included in the solver
//===================================================================================
bool PGSSolver::hasFlag(uint8_t flags, uint8_t flag) {
    return (flags & flag) != 0;
}
bool PGSSolver::prepareContactPointBaumgarte(
    const ContactConstraint& contact,
    ContactPoint& src,
    ContactConstraintPoint& dst,
    float dt
) {
    constexpr float staticFriction = 0.6f;

    constexpr float defaultSlop = 0.0007f;
    constexpr float noResponseSlop = 0.0007f;

    constexpr float defaultBaumgarte = 0.6f;
    constexpr float noResponseBaumgarte = 0.6f;

    constexpr float persistentSlop = 0.005f;

    float contactSlop = defaultSlop;
    float contactBaumgarte = defaultBaumgarte;

    if (hasFlag(contact.flags, NoSolverResponseA) ||
        hasFlag(contact.flags, NoSolverResponseB))
    {
        contactSlop = noResponseSlop;
        contactBaumgarte = noResponseBaumgarte;
    }

    bool active = src.depth > -persistentSlop;

    if (!active) {
        src.accumulatedNormalImpulse = 0.0f;
        src.accumulatedFrictionImpulse1 = 0.0f;
        src.accumulatedFrictionImpulse2 = 0.0f;
        src.accumulatedBiasImpulse = 0.0f;
        src.biasVelocity = 0.0f;
        src.active = false;

        return false;
    }

    src.active = true;

    dst.rA = src.rA;
    dst.rB = src.rB;

    dst.m_eff = src.m_eff;
    dst.invMassT1 = src.invMassT1;
    dst.invMassT2 = src.invMassT2;

    dst.targetBounce = src.targetBounceVelocity;

    dst.nImpulse = src.accumulatedNormalImpulse;
    dst.fImpulse1 = src.accumulatedFrictionImpulse1;
    dst.fImpulse2 = src.accumulatedFrictionImpulse2;

    // Bias impulse warmstartas inte.
    dst.biasImpulse = 0.0f;

    float allowed = src.depth - contactSlop;

    if (allowed > 0.0f) {
        float correctionSpeed = (contactBaumgarte * allowed) / dt;
        dst.biasVelocity = -correctionSpeed;
    }
    else {
        dst.biasVelocity = 0.0f;
    }

    src.biasVelocity = dst.biasVelocity;

    // Clamp cached friction warm-start to static friction cone.
    float maxFriction = staticFriction * dst.nImpulse;

    float f1 = dst.fImpulse1;
    float f2 = dst.fImpulse2;

    float len2 = f1 * f1 + f2 * f2;
    float max2 = maxFriction * maxFriction;

    if (len2 > max2) {
        float len = std::sqrt(len2);

        if (len > 1e-6f) {
            float s = maxFriction / len;
            dst.fImpulse1 *= s;
            dst.fImpulse2 *= s;
        }
    }

    return true;
}

//===========================================================================
//  Warm-start the solver by applying cached impulses to the solver bodies
//===========================================================================
void PGSSolver::warmStartContactPoint(
    const ContactConstraint& contact,
    const ContactConstraintPoint& cp
) {
    glm::vec3 Pn = cp.nImpulse * contact.normal;

    glm::vec3 Pt =
        cp.fImpulse1 * contact.t1 +
        cp.fImpulse2 * contact.t2;

    glm::vec3 J = Pn + Pt;

    if (contact.flags & CanApplyImpulseA) {
        SolverBody* bodyA = getSolverBodyOrNull(contact.bodyA);

        applyImpulseToSolverBody(
            bodyA,
            -J,
            -glm::cross(cp.rA, J)
        );
    }

    if (contact.flags & CanApplyImpulseB) {
        SolverBody* bodyB = getSolverBodyOrNull(contact.bodyB);

        applyImpulseToSolverBody(
            bodyB,
            J,
            glm::cross(cp.rB, J)
        );
    }
}