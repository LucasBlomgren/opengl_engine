#include "narrowphase_manager.h"
#include "physics/sleep/wake_sleep_utils.h"

//=======================================================
//     Emit rigid contact and wake up bodies if needed
//=======================================================
void NarrowphaseManager::emitRigidContact(
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

    if (tryExportExternalContact(in, candidate.sat)) {
        return;
    }

    bool noSolverResponseA = computeNoSolverResponse(*in.bodyA, wakeInfo.A);
    bool noSolverResponseB = computeNoSolverResponse(*in.bodyB, wakeInfo.B);

    if (noSolverResponseA && noSolverResponseB) {
        return;
    }

    ContactRuntime runtimeData = makeRuntimeData(
        in.bodyA,
        in.bodyB,
        in.colliderA,
        in.colliderB,
        caches->transforms.get(in.bodyA->rootTransformHandle, FUNC_NAME),
        caches->transforms.get(in.bodyB->rootTransformHandle, FUNC_NAME)
    );

    Contact contact(
        in.bodyHandleA,
        in.bodyHandleB,
        runtimeData,
        candidate.sat.normal    
    );

    contact.noSolverResponseA = noSolverResponseA;
    contact.noSolverResponseB = noSolverResponseB;

    contact.contributesMotionA = computeContributesMotion(contact.partnerTypeA, *in.bodyA, wakeInfo.A);
    contact.contributesMotionB = computeContributesMotion(contact.partnerTypeB, *in.bodyB, wakeInfo.B);

    Contact* contactPtr = createManifold(contact, candidate);
    batch.contacts.push_back(contactPtr);
}

//=======================================================
//     Create contact manifold
//=======================================================
Contact* NarrowphaseManager::createManifold(
    Contact& contact,
    DynamicContactCandidate& candidate)
{
    switch (candidate.manifoldType) {
    case ManifoldType::BoxBox:
        return collisionManifold->boxBox(
            contact,
            *contactCache,
            candidate.sat
        );

    case ManifoldType::BoxSphere:
        return collisionManifold->boxSphere(
            contact,
            *contactCache,
            candidate.sat
        );

    case ManifoldType::SphereSphere:
        return collisionManifold->sphereSphere(
            contact,
            *contactCache,
            candidate.sat
        );
    }

    return nullptr;
}

//=======================================================
//     Export external motion contacts for character controllers
//=======================================================
bool NarrowphaseManager::tryExportExternalContact(
    const ContactBuildInput& in,
    const SAT::Result& sat)
{
    if (sat.hitType == SAT::HitType::Speculative) {
        return false;
    }

    bool aCharacter = in.bodyA->motionControl == MotionControl::External && 
        in.bodyA->responseMode == ContactResponseMode::Character;

    bool bCharacter = in.bodyB->motionControl == MotionControl::External &&
        in.bodyB->responseMode == ContactResponseMode::Character;

    if (!aCharacter && !bCharacter) {
        return false;
    }

    externalContacts.emplace_back(
        in.colliderA->rigidBodyHandle,
        in.colliderB->rigidBodyHandle,
        sat.normal,
        sat.depth
    );

    return true;
}