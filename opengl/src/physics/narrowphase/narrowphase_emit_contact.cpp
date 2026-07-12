#include "narrowphase_manager.h"
#include "sleep/wake_sleep_utils.h"

//=======================================================
//     Contact emission
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
//     Export external motion contacts for character controllers
//=======================================================
bool NarrowphaseManager::tryExportExternalContact(
    const ContactBuildInput& in,
    const SAT::Result& sat)
{
    bool aCharacter =
        in.bodyA->motionControl == MotionControl::External &&
        in.bodyA->responseMode == ContactResponseMode::Character;

    bool bCharacter =
        in.bodyB->motionControl == MotionControl::External &&
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

//=======================================================
//     Compute contributes motion
//=======================================================
bool NarrowphaseManager::computeContributesMotion(
    ContactPartnerType partnerType,
    const RigidBody& body,
    bool willWake) const
{
    return
        partnerType == ContactPartnerType::RigidBody &&
        (
            body.type == BodyType::Kinematic ||
            (body.type == BodyType::Dynamic && (!body.asleep || willWake))
            );
}

bool NarrowphaseManager::computeNoSolverResponse(
    const RigidBody& body,
    bool willWake) const
{
    return
        body.motionControl == MotionControl::External ||
        body.type == BodyType::Static ||
        body.type == BodyType::Kinematic ||
        (body.type == BodyType::Dynamic && body.asleep && !willWake);
}

//=======================================================
//     Create contact manifold
//=======================================================
Contact* NarrowphaseManager::createManifold(
    Contact& contact,
    DynamicContactCandidate& candidate)
{
    switch (candidate.manifoldType) {
    case DynamicManifoldType::BoxBox:
        return collisionManifold->boxBox(
            contact,
            *contactCache,
            candidate.sat
        );

    case DynamicManifoldType::BoxSphere:
        return collisionManifold->boxSphere(
            contact,
            *contactCache,
            candidate.sat
        );

    case DynamicManifoldType::SphereSphere:
        return collisionManifold->sphereSphere(
            contact,
            *contactCache,
            candidate.sat
        );
    }

    return nullptr;
}