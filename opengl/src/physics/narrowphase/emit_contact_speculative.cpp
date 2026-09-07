#include "narrowphase_manager.h"
#include <type_traits>

namespace physics::internal {

//=======================================================
//   Flush pending speculative contacts, 
//   filtering by earliest TOI per sweep owner
//=======================================================
void NarrowphaseManager::flushPendingSweepHits(ContactBatch& batch)
{
    std::unordered_map<uint64_t, float> minToiBySweepOwner;
    minToiBySweepOwner.reserve(pendingSweepHits.size());

    const auto geometryOf = [](const PendingSweepHit& pending)
        -> const SAT::Result&
    {
        return std::visit(
            [](const auto& hit) -> const SAT::Result& {
                return hit.geometry;
            },
            pending
        );
    };

    const auto sweepOwnerOf = [](const PendingSweepHit& pending) {
        return std::visit(
            [](const auto& hit) {
                return hit.sweepOwner;
            },
            pending
        );
    };

    // Pass 1: find min TOI per sweep owner
    for (const PendingSweepHit& pending : pendingSweepHits) {
        const float toi = geometryOf(pending).toi;
        uint64_t ownerKey = packBodyHandle(sweepOwnerOf(pending));

        auto it = minToiBySweepOwner.find(ownerKey);
        if (it == minToiBySweepOwner.end() || toi < it->second) {
            minToiBySweepOwner[ownerKey] = toi;
        }
    }

    // Pass 2: emit contacts close to the earliest TOI
    std::unordered_set<PairKey, PairKeyHash> emittedSpeculativePairs;
    emittedSpeculativePairs.reserve(pendingSweepHits.size());

    // #TODO: Ska egentligen vara ett field framför 
    // collidern som bestämmer vilka träffar som ska generera kontakt.
    const float toiSlop = 5.5f;

    for (const PendingSweepHit& pending : pendingSweepHits) {
        const uint64_t ownerKey = packBodyHandle(sweepOwnerOf(pending));

        const float minToi = minToiBySweepOwner[ownerKey];
        const float toi = geometryOf(pending).toi;

        if (toi > minToi + toiSlop) {
            continue;
        }

        PairKey key = std::visit(
            [this](const auto& hit) -> PairKey {
                using HitType = std::decay_t<decltype(hit)>;

                if constexpr (std::is_same_v<HitType, SweepHit>) {
                    return makeColliderPairKey(
                        hit.pair.a.colliderHandle,
                        hit.pair.b.colliderHandle
                    );
                }
                else {
                    return makeColliderPairKey(
                        hit.collider.colliderHandle,
                        ColliderHandle{}
                    );
                }
            },
            pending
        );

        // Avoid emitting duplicate speculative contacts for the same collider pair.
        // Both (A,B) and (B,A) show up because in swept-vs-swept between two 
        // moving bodies, we don't want to lose the information that the collision 
        // can be relevant for both bodies' own 'first hit' filter.
        if (!emittedSpeculativePairs.insert(key).second) {
            continue;
        }

        std::visit(
            [this, &batch](const auto& hit) {
                emitSpeculativeContact(batch, hit);
            },
            pending
        );
    }
}

//=======================================================
//     Adapt typed sweep hits to common contact emission
//=======================================================
void NarrowphaseManager::emitSpeculativeContact(
    ContactBatch& batch,
    const SweepHit& hit)
{
    emitSpeculativeContact(
        batch,
        hit.pair.a,
        &hit.pair.b,
        ContactPartnerType::RigidBody,
        hit.geometry
    );
}

void NarrowphaseManager::emitSpeculativeContact(
    ContactBatch& batch,
    const TerrainSweepHit& hit)
{
    emitSpeculativeContact(
        batch,
        hit.collider,
        nullptr,
        ContactPartnerType::Terrain,
        hit.geometry
    );
}

//=======================================================
//     Emit speculative contact and wake up bodies if needed
//=======================================================
void NarrowphaseManager::emitSpeculativeContact(
    ContactBatch& batch,
    const ColliderEndpointRef& a,
    const ColliderEndpointRef* b,
    ContactPartnerType partnerTypeB,
    const SAT::Result& geometry)
{
    const bool isRigidA = a.body != nullptr;

    // b == nullptr -> terrain contact
    const bool isRigidB =
        partnerTypeB == ContactPartnerType::RigidBody &&
        b != nullptr &&
        b->body != nullptr;

    if (!isRigidA && !isRigidB) {
        return;
    }

    if (isRigidA && a.body->type == BodyType::Dynamic) {
        SleepState& sleepA = physicsWorld->getSleepState(a.body->sleepStateHandle);
        sleepA.collisionCount++;
    }
    if (isRigidB && b->body->type == BodyType::Dynamic) {
        SleepState& sleepB = physicsWorld->getSleepState(b->body->sleepStateHandle);
        sleepB.collisionCount++;
    }

    SpeculativeContact contact{};

    contact.partnerTypeA = ContactPartnerType::RigidBody;
    contact.partnerTypeB = partnerTypeB;

    contact.bodyHandleA = a.bodyHandle;
    contact.bodyHandleB = b ? b->bodyHandle : BodyHandle{};

    contact.bodyA = a.body;
    contact.bodyB = b ? b->body : nullptr;

    contact.normal = geometry.normal;
    contact.separation = geometry.separation;
    contact.toi = geometry.toi;

    batch.speculativeContacts.push_back(contact);

    if (debugSpeculativeContacts) {
        DebugSpeculativeContact debugContact{};
        debugContact.bodyA = a.bodyHandle;
        debugContact.bodyB = b ? b->bodyHandle : BodyHandle{};
        debugContact.worldPos = geometry.point;

        debugSpeculativeContacts->push_back(debugContact);
    }
}

}
