#include "pch.h"
#include "pgs_solver.h"

namespace physics::internal {

bool PGSSolver::computeContributesMotion(
    ContactPartnerType partnerType,
    const RigidBody& body) const
{
    if (partnerType == ContactPartnerType::Terrain ||
        body.type == BodyType::Static) {
        return false;
    }

    if (body.type == BodyType::Kinematic) {
        return true;
    }

    if (body.type == BodyType::Dynamic) {
        SleepState& sleepState =
            physicsWorld->getSleepState(body.sleepStateHandle);

        if (!sleepState.asleep) {
            return true;
        }
    }

    return false;
}
bool PGSSolver::computeNoSolverResponse(
    const RigidBody& body) const
{
    if (body.type != BodyType::Dynamic)
        return true;

    if (body.type == BodyType::Dynamic) {
        SleepState& sleepState =
            physicsWorld->getSleepState(body.sleepStateHandle);

        if (sleepState.asleep) {
            return true;
        }
    }

    return false;
}

//=====================================================================
//  Build solver data structures from contact batch and runtime caches
//=====================================================================
void PGSSolver::buildSolverData(
    ContactBatch& batch,
    float dt) 
{
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
    size_t slotCap = physicsWorld->bodyStorage().slot_capacity();
    solverBodyIndexBySlot.assign(slotCap, InvalidSolverBody);

    solverBodies.reserve(physicsWorld->bodyStorage().dense().size());
    contactConstraints.reserve(batch.contacts.size());
    contactPoints.reserve(batch.contacts.size() * 4);
    speculativeConstraints.reserve(batch.speculativeContacts.size());

    solverBodySources.reserve(physicsWorld->bodyStorage().dense().size());
    solverBodyWriteBack.reserve(physicsWorld->bodyStorage().dense().size());
    constraintSources.reserve(batch.contacts.size());
    pointSources.reserve(batch.contacts.size() * 4);

    // #TODO: inte super effektivt att loopa igenom en linked list av contacts.
    // Kan optimera senare om det behövs.
    for (Contact* contact : batch.contacts) {
        if (!contact) {
            continue;
        }

        ContactRuntime& rt = contact->runtimeData;

        // A is always a rigid body
        bool noSolverResponseA = computeNoSolverResponse(*rt.bodyA);
        bool contributesMotionA =
            computeContributesMotion(
                contact->partnerTypeA,
                *rt.bodyA
            );

        // B can be terrain or a rigid body
        bool noSolverResponseB = true;
        bool contributesMotionB = false;

        if (rt.bodyB) {
            noSolverResponseB = computeNoSolverResponse(*rt.bodyB);
            contributesMotionB =
                computeContributesMotion(
                    contact->partnerTypeB,
                    *rt.bodyB
                );
        }

        if (noSolverResponseA && noSolverResponseB) {
            continue;
        }

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

        // #TODO: warm-start twist impulse?
        cc.invMassTwist = 0.0f;
        cc.accumulatedTwistImpulse = 0.0f;

        cc.firstPoint = static_cast<uint32_t>(contactPoints.size());

        if (bodyAIndex != InvalidSolverBody && contributesMotionA) {
            cc.flags |= ContributesMotionA;
        }
        if (bodyBIndex != InvalidSolverBody && contributesMotionB) {
            cc.flags |= ContributesMotionB;
        }
        if (noSolverResponseA) cc.flags |= NoSolverResponseA;
        if (noSolverResponseB) cc.flags |= NoSolverResponseB;

        if (bodyAIndex != InvalidSolverBody &&
            contact->partnerTypeA == ContactPartnerType::RigidBody &&
            !noSolverResponseA)
        {
            cc.flags |= CanApplyImpulseA;
            solverBodyWriteBack[bodyAIndex] = 1;
        }

        if (bodyBIndex != InvalidSolverBody &&
            contact->partnerTypeB == ContactPartnerType::RigidBody &&
            !noSolverResponseB)
        {
            cc.flags |= CanApplyImpulseB;
            solverBodyWriteBack[bodyBIndex] = 1;
        }

        for (size_t i = 0; i < contact->numPoints; ++i) {
            ContactPoint& cp = contact->points[i];
            ContactConstraintPoint& ccp = contactPoints.emplace_back();

            precomputePointData(
                rt.bodyA,
                rt.bodyB,
                ccp,
                cp.wasWarmStarted,
                cp.worldPos,
                cc,
                contact->framesSinceUsed
            );

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

        bool noSolverResponseA =
            computeNoSolverResponse(*src.bodyA);

        bool contributesMotionA =
            computeContributesMotion(
                src.partnerTypeA,
                *src.bodyA
            );

        // terrain is always B
        bool noSolverResponseB = true;
        bool contributesMotionB = false;

        if (src.bodyB) {
            noSolverResponseB =
                computeNoSolverResponse(*src.bodyB);

            contributesMotionB =
                computeContributesMotion(
                    src.partnerTypeB,
                    *src.bodyB
                );
        }

        // Compute incoming normal velocity for post-solve restitution calculation
        glm::vec3 vA{ 0.0f };
        glm::vec3 vB{ 0.0f };
        if (src.partnerTypeA == ContactPartnerType::RigidBody &&
            src.bodyA &&
            contributesMotionA)
        {
            vA = src.bodyA->linearVelocity;
        }
        if (src.partnerTypeB == ContactPartnerType::RigidBody &&
            src.bodyB &&
            contributesMotionB)
        {
            vB = src.bodyB->linearVelocity;
        }

        glm::vec3 relativeVelocity = vB - vA;
        c.incomingNormalVelocity = glm::dot(relativeVelocity, c.normal);

        // No cross-frame warmstart for MVP.
        c.accumulatedImpulse = 0.0f;

        if (bodyAIndex != InvalidSolverBody && contributesMotionA) {
            c.flags |= ContributesMotionA;
        }
        if (bodyBIndex != InvalidSolverBody && contributesMotionB) {
            c.flags |= ContributesMotionB;
        }
        if (noSolverResponseA) c.flags |= NoSolverResponseA;
        if (noSolverResponseB) c.flags |= NoSolverResponseB;
        if (bodyAIndex != InvalidSolverBody &&
            src.partnerTypeA == ContactPartnerType::RigidBody &&
            !noSolverResponseA)
        {
            c.flags |= CanApplyImpulseA;
            solverBodyWriteBack[bodyAIndex] = 1;
        }
        if (bodyBIndex != InvalidSolverBody &&
            src.partnerTypeB == ContactPartnerType::RigidBody &&
            !noSolverResponseB)
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
    BodyHandle bodyHandle,
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

//=====================================================
//  Precompute point data
//=====================================================
void PGSSolver::precomputePointData(
    RigidBody* bodyA,
    RigidBody* bodyB,
    ContactConstraintPoint& cp,
    bool warmStarted,
    glm::vec3& worldPos,
    ContactConstraint& contact,
    int framesSinceUsed)
{
    // smallest normal velocity to allow restitution (bounce)
    constexpr float restitutionThreshold = 0.2f;
    float restitution = 0.0f; // example material

    glm::vec3& normal = contact.normal;

    float invMassA = 0.0f;
    float invMassB = 0.0f;
    glm::vec3 rA{ 0.0f };
    glm::vec3 rB{ 0.0f };
    glm::mat3 invInertiaA{ 0.0f };
    glm::mat3 invInertiaB{ 0.0f };
    glm::vec3 linearVelocityA{ 0.0f };
    glm::vec3 linearVelocityB{ 0.0f };
    glm::vec3 angularVelocityA{ 0.0f };
    glm::vec3 angularVelocityB{ 0.0f };

    // bodyA solver response behavior
    if (contact.flags & ContactFlags::NoSolverResponseA) {
        invMassA = 0.0f;
        invInertiaA = glm::mat3(0.0f);
    }
    else {
        invMassA = bodyA->invMass;
        invInertiaA = bodyA->invInertiaWorld;
    }
    // bodyA motion behavior
    if (contact.flags & ContactFlags::ContributesMotionA) {
        rA = worldPos - bodyA->pose.position; // #TODO: use center of mass.
        linearVelocityA = bodyA->linearVelocity;
        angularVelocityA = bodyA->angularVelocity;
    }
    else {
        rA = glm::vec3(0.0f);
        linearVelocityA = glm::vec3(0.0f);
        angularVelocityA = glm::vec3(0.0f);
    }

    // bodyB solver response behavior
    if (contact.flags & ContactFlags::NoSolverResponseB) {
        invMassB = 0.0f;
        invInertiaB = glm::mat3(0.0f);
    }
    else {
        invMassB = bodyB->invMass;
        invInertiaB = bodyB->invInertiaWorld;
    }
    // bodyB motion behavior
    if (contact.flags & ContactFlags::ContributesMotionB) {
        rB = worldPos - bodyB->pose.position; // #TODO: use center of mass.
        linearVelocityB = bodyB->linearVelocity;
        angularVelocityB = bodyB->angularVelocity;
    }
    else {
        rB = glm::vec3(0.0f);
        linearVelocityB = glm::vec3(0.0f);
        angularVelocityB = glm::vec3(0.0f);
    }

    // pre-calculate rA, rB, EffectiveMass
    cp.rA = rA;
    cp.rB = rB;

    glm::vec3 rA_cross_n = glm::cross(rA, normal);
    glm::vec3 rB_cross_n = glm::cross(rB, normal);
    cp.m_eff = 1.0f / (invMassA + invMassB +
        glm::dot(rA_cross_n, invInertiaA * rA_cross_n) +
        glm::dot(rB_cross_n, invInertiaB * rB_cross_n));

    //if (cp.m_eff <= 1e-8f) {
    //    std::cout 
    // << "Warning: contact point with near-zero effective mass!" 
    // << std::endl;
    //}

    // compute relative velocity at contact point based on current body states
    glm::vec3 relativeVelocity =
        (linearVelocityB + glm::cross(angularVelocityB, rB)) -
        (linearVelocityA + glm::cross(angularVelocityA, rA));

    // if the contact is warm-started (i.e. "old"), we disable 
    // restitution to avoid bounce due to accumulated penetration 
    // correction impulses from previous frames, which can cause jitter. 
    // This also means that only new contacts with sufficient 
    // impact velocity will bounce, which is a common and 
    // stable approach in physics engines.
    bool allowRestitution = true;
    if (warmStarted || framesSinceUsed > 0) {
        allowRestitution = false;
    }

    float normalVelocity = glm::dot(relativeVelocity, normal);
    if (allowRestitution and normalVelocity < -restitutionThreshold) {
        cp.targetBounce = -restitution * normalVelocity;
    }
    else {
        cp.targetBounce = 0.0f;
    }

    glm::vec3 rA_t1 = glm::cross(rA, contact.t1);
    glm::vec3 rB_t1 = glm::cross(rB, contact.t1);
    glm::vec3 rA_t2 = glm::cross(rA, contact.t2);
    glm::vec3 rB_t2 = glm::cross(rB, contact.t2);

    glm::vec3 invIA_rA_t1 = invInertiaA * rA_t1;
    glm::vec3 invIB_rB_t1 = invInertiaB * rB_t1;
    glm::vec3 invIA_rA_t2 = invInertiaA * rA_t2;
    glm::vec3 invIB_rB_t2 = invInertiaB * rB_t2;

    // Compute effective mass along cp.t1 and cp.t2 for friction
    // calculations in the solver. 
    // This is needed to determine how much tangential impulse 
    // to apply for a given desired change in tangential velocity, 
    // similar to how cp.m_eff is used for normal impulses.
    float k_t1 =
        (invMassA + invMassB) +
        glm::dot(rA_t1, invIA_rA_t1) +
        glm::dot(rB_t1, invIB_rB_t1);

    cp.invMassT1 = 1.0f / k_t1;

    float k_t2 =
        (invMassA + invMassB) +
        glm::dot(rA_t2, invIA_rA_t2) +
        glm::dot(rB_t2, invIB_rB_t2);

    cp.invMassT2 = 1.0f / k_t2;
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
    float dt) 
{
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

}
