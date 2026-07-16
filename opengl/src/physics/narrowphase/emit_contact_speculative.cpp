#include "narrowphase_manager.h"
#include "sleep/wake_sleep_utils.h"

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

    // #TODO: Fundera på vad TOI slop ska vara:
    // kanske beroende på dt, kropparnas hastighet, kropparnas storlek.
    const float toiSlop = 1e-6f;

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
        // Both (A,B) and (B,A) show up because in swept-vs-swept between two moving bodies,
        // we don't want to lose the information that the collision can be relevant for both 
        // bodies' own 'first hit' filter.
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
    in.bodyA->totalCollisionCount++;
    in.bodyB->totalCollisionCount++;

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

    bool noSolverResponseA = computeNoSolverResponse(*in.bodyA, wakeInfo.A);
    bool noSolverResponseB = computeNoSolverResponse(*in.bodyB, wakeInfo.B);

    if (noSolverResponseA && noSolverResponseB) {
        return;
    }

    batch.speculativeContacts.emplace_back();
    SpeculativeContact& c = batch.speculativeContacts.back();
    c.bodyHandleA = in.bodyHandleA;
    c.bodyHandleB = in.bodyHandleB;
    c.bodyA = in.bodyA;
    c.bodyB = in.bodyB;

    c.normal = candidate.sat.normal;
    c.separation = candidate.sat.separation;
    c.toi = candidate.sat.toi;

    c.noSolverResponseA = noSolverResponseA;
    c.noSolverResponseB = noSolverResponseB;

    c.contributesMotionA = computeContributesMotion(
        ContactPartnerType::RigidBody,
        *in.bodyA,
        wakeInfo.A
    );

    c.contributesMotionB = computeContributesMotion(
        ContactPartnerType::RigidBody,
        *in.bodyB,
        wakeInfo.B
    );

    // Debug visualization only.
    // #TODO: Should be activated by a debug flag, not always.
    debugSpeculativeContacts->emplace_back();
    DebugSpeculativeContact& debugContact = debugSpeculativeContacts->back();
    debugContact.bodyA = in.bodyHandleA;
    debugContact.bodyB = in.bodyHandleB;
    debugContact.worldPos = candidate.sat.point;
}