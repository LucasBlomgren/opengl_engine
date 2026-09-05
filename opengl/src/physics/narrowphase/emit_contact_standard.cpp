#include "narrowphase_manager.h"
#include "physics/sleep/wake_sleep_utils.h"

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

    if (a.body->type == BodyType::Dynamic) {
        SleepState& sleepA = physicsWorld->getSleepState(a.body->sleepStateHandle);
        sleepA.collisionCount++;
    }
    if (b.body->type == BodyType::Dynamic) {
        SleepState& sleepB = physicsWorld->getSleepState(b.body->sleepStateHandle);
        sleepB.collisionCount++;
    }

    WakeSleep::WakeUpInfo wakeInfo =
        WakeSleep::computeWakeUpInfo(*physicsWorld, *a.body, *b.body);

    WakeSleep::enqueueWakeRequests(
        *physicsWorld,
        wakeInfo,
        *a.body,
        *b.body,
        a.bodyHandle,
        b.bodyHandle,
        *toWake
    );

    if (tryExportExternalContact(hit.pair, hit.geometry)) {
        return;
    }

    bool noSolverResponseA = computeNoSolverResponse(*a.body, wakeInfo.A);
    bool noSolverResponseB = computeNoSolverResponse(*b.body, wakeInfo.B);

    if (noSolverResponseA && noSolverResponseB) {
        return;
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

    contact.noSolverResponseA = noSolverResponseA;
    contact.noSolverResponseB = noSolverResponseB;

    contact.contributesMotionA = computeContributesMotion(
        contact.partnerTypeA,
        *a.body,
        wakeInfo.A
    );
    contact.contributesMotionB = computeContributesMotion(
        contact.partnerTypeB,
        *b.body,
        wakeInfo.B
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

    bool aCharacter =
        pair.a.body->motionControl == MotionControl::External &&
        pair.a.body->responseMode == ResponseMode::Character;

    bool bCharacter =
        pair.b.body->motionControl == MotionControl::External &&
        pair.b.body->responseMode == ResponseMode::Character;

    if (!aCharacter && !bCharacter) {
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
