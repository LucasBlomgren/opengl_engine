#include "narrowphase_manager.h"

namespace physics::internal {

//=======================================================
//     Emit rigid contact and wake up bodies if needed
//=======================================================
void NarrowphaseManager::emitRigidContact(
    ContactBatch& batch,
    OverlapHit& hit)
{
    ColliderEndpointRef& a = hit.pair.a;
    ColliderEndpointRef& b = hit.pair.b;

    tryExportExternalContact(hit.pair, hit.geometry);

    if (a.body->type == BodyType::Kinematic &&
        b.body->type == BodyType::Kinematic) {
        return;
    }

    if (a.body->type == BodyType::Dynamic) {
        SleepState& sleepA = physicsWorld->getSleepState(a.body->sleepStateHandle);
        sleepA.collisionCount++;
    }
    if (b.body->type == BodyType::Dynamic) {
        SleepState& sleepB = physicsWorld->getSleepState(b.body->sleepStateHandle);
        sleepB.collisionCount++;
    }

    ContactRuntime runtimeData = makeRuntimeData(
        a.body,
        b.body,
        a.collider,
        b.collider
    );

    Contact contact(
        a.bodyHandle,
        b.bodyHandle,
        runtimeData,
        hit.geometry.normal
    );

    Contact* contactPtr = createManifold(contact, hit);
    batch.contacts.push_back(contactPtr);
}

//=======================================================
//     Create contact manifold
//=======================================================
Contact* NarrowphaseManager::createManifold(
    Contact& contact,
    OverlapHit& hit)
{
    switch (hit.pair.shapePair) {
    case ShapePairKind::BoxBox:
        return collisionManifold->boxBox(
            contact,
            *contactCache,
            hit.geometry
        );

    case ShapePairKind::BoxSphere:
        return collisionManifold->boxSphere(
            contact,
            *contactCache,
            hit.geometry
        );

    case ShapePairKind::SphereSphere:
        return collisionManifold->sphereSphere(
            contact,
            *contactCache,
            hit.geometry
        );
    }

    return nullptr;
}

//=======================================================
//     Export external motion contacts for character controllers
//=======================================================
bool NarrowphaseManager::tryExportExternalContact(
    const ResolvedColliderPair& pair,
    const SAT::Result& sat)
{
    if (sat.hitType == SAT::HitType::Speculative) {
        return false;
    }

    if (!pair.a.body->reportContacts && 
        !pair.b.body->reportContacts) {
        return false;
    }

    externalContacts.emplace_back(
        pair.a.collider->rigidBodyHandle,
        pair.b.collider->rigidBodyHandle,
        sat.normal,
        sat.depth
    );

    return true;
}


}
