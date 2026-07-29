#include "narrowphase_manager.h"
#include "physics/sleep/wake_sleep_utils.h"

namespace physics::internal {

//=======================================================
//   Flush pending speculative contacts, 
//   filtering by earliest TOI per sweep owner
//=======================================================
void NarrowphaseManager::flushPendingSpeculativeContacts(
    ContactBatch& batch,
    float dt)
{
    std::unordered_map<uint64_t, float> minToiBySweepOwner;
    minToiBySweepOwner.reserve(pendingSpeculativeContacts.size());

    // Pass 1: find min TOI per sweep owner
    for (const PendingSpeculativeContact& pending : pendingSpeculativeContacts) {
        const float toi = pending.candidate.sat.toi;
        uint64_t ownerKey = packBodyHandle(pending.sweepOwner);

        auto it = minToiBySweepOwner.find(ownerKey);
        if (it == minToiBySweepOwner.end() || toi < it->second) {
            minToiBySweepOwner[ownerKey] = toi;
        }
    }

    // Pass 2: emit contacts close to the earliest TOI
    std::unordered_set<PairKey, PairKeyHash> emittedSpeculativePairs;
    emittedSpeculativePairs.reserve(pendingSpeculativeContacts.size());

    // #TODO: Ska egentligen vara ett field framför 
    // collidern som bestämmer vilka träffar som ska generera kontakt.
    const float toiSlop = 5.5f;

    for (PendingSpeculativeContact& pending : pendingSpeculativeContacts) {
        const uint64_t ownerKey = packBodyHandle(pending.sweepOwner);

        const float minToi = minToiBySweepOwner[ownerKey];
        const float toi = pending.candidate.sat.toi;

        if (toi > minToi + toiSlop) {
            continue;
        }

        PairKey key = makeColliderPairKey(
            pending.input.colliderHandleA,
            pending.input.colliderHandleB
        );

        // Avoid emitting duplicate speculative contacts for the same collider pair.
        // Both (A,B) and (B,A) show up because in swept-vs-swept between two 
        // moving bodies, we don't want to lose the information that the collision 
        // can be relevant for both bodies' own 'first hit' filter.
        if (!emittedSpeculativePairs.insert(key).second) {
            continue;
        }

        emitSpeculativeContact(
            batch,
            pending.input,
            pending.candidate
        );
    }
}

//=======================================================
//     Emit speculative contact and wake up bodies if needed
//=======================================================
void NarrowphaseManager::emitSpeculativeContact(
    ContactBatch& batch,
    ContactBuildInput& in,
    DynamicContactCandidate& candidate)
{
    const bool isRigidA =
        candidate.partnerTypeA == ContactPartnerType::RigidBody && in.bodyA;

    const bool isRigidB =
        candidate.partnerTypeB == ContactPartnerType::RigidBody && in.bodyB;

    if (!isRigidA && !isRigidB) {
        return;
    }

    // Count speculative collisions too, if you want them included in 
    // collision debug/stats.
    if (isRigidA) {
        in.bodyA->totalCollisionCount++;
    }

    if (isRigidB) {
        in.bodyB->totalCollisionCount++;
    }

    bool wakeA = false;
    bool wakeB = false;

    // Only dynamic rigid-vs-rigid speculative contacts can wake both bodies.
    // Sphere-vs-terrain does not wake terrain and does not need 
    // WakeSleep::computeWakeUpInfo.
    if (isRigidA && isRigidB) {
        WakeSleep::WakeUpInfo wakeInfo =
            WakeSleep::computeWakeUpInfo(*in.bodyA, *in.bodyB);

        WakeSleep::enqueueWakeRequests(
            wakeInfo,
            *in.bodyA,
            *in.bodyB,
            in.bodyHandleA,
            in.bodyHandleB,
            *toWake
        );

        wakeA = wakeInfo.A;
        wakeB = wakeInfo.B;
    }

    SpeculativeContact contact{};

    contact.partnerTypeA = candidate.partnerTypeA;
    contact.partnerTypeB = candidate.partnerTypeB;

    contact.bodyHandleA = in.bodyHandleA;
    contact.bodyHandleB = in.bodyHandleB;

    contact.bodyA = in.bodyA;
    contact.bodyB = in.bodyB;

    contact.normal = candidate.sat.normal;
    contact.separation = candidate.sat.separation;
    contact.toi = candidate.sat.toi;

    if (isRigidA) {
        contact.noSolverResponseA =
            computeNoSolverResponse(*in.bodyA, wakeA);

        contact.contributesMotionA =
            computeContributesMotion(
                candidate.partnerTypeA,
                *in.bodyA,
                wakeA
            );
    }
    else {
        contact.noSolverResponseA = true;
        contact.contributesMotionA = false;
    }

    if (isRigidB) {
        contact.noSolverResponseB =
            computeNoSolverResponse(*in.bodyB, wakeB);

        contact.contributesMotionB =
            computeContributesMotion(
                candidate.partnerTypeB,
                *in.bodyB,
                wakeB
            );
    }
    else {
        // Terrain / missing body
        contact.noSolverResponseB = true;
        contact.contributesMotionB = false;
    }

    if (contact.noSolverResponseA && contact.noSolverResponseB) {
        return;
    }

    batch.speculativeContacts.push_back(contact);

    if (debugSpeculativeContacts) {
        DebugSpeculativeContact debugContact{};
        debugContact.bodyA = in.bodyHandleA;
        debugContact.bodyB = in.bodyHandleB;
        debugContact.worldPos = candidate.sat.point;

        debugSpeculativeContacts->push_back(debugContact);
    }
}

}
