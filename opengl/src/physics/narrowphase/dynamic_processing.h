#pragma once
#include "narrowphase_manager.h"
#include "physics/wake_sleep_utils.h"

//----------------------------------------------
//     Box vs Box
//----------------------------------------------
void NarrowphaseManager::processBoxBox(
    RigidBodyHandle bodyHandleA,
    RigidBodyHandle bodyHandleB,
    ColliderHandle colliderHandleA,
    ColliderHandle colliderHandleB,
    RigidBody* bodyA,
    RigidBody* bodyB,
    Collider* colliderA,
    Collider* colliderB)
{
    SAT::Result satResult;
    if (!SAT::boxBox(*colliderA, *colliderB, satResult)) {
        return;
    }
    bodyA->totalCollisionCount++;
    bodyB->totalCollisionCount++;

    glm::vec3& centerA = std::get<OOBB>(colliderA->shape).worldCenter;
    glm::vec3& centerB = std::get<OOBB>(colliderB->shape).worldCenter;
    SAT::reverseNormal(centerA, centerB, satResult.normal);

    WakeSleep::WakeUpInfo wakeInfo = WakeSleep::computeWakeUpInfo(*bodyA, *bodyB);
    WakeSleep::enqueueWakeRequests(
        wakeInfo,
        *bodyA, *bodyB,
        bodyHandleA, bodyHandleB,
        *toWake
    );

    // export to character controller
    bool aCharacter = bodyA->motionControl == MotionControl::External &&
        bodyA->responseMode == ContactResponseMode::Character;

    bool bCharacter = bodyB->motionControl == MotionControl::External &&
        bodyB->responseMode == ContactResponseMode::Character;

    if (aCharacter || bCharacter) {
        externalContacts.emplace_back(
            colliderA->rigidBodyHandle,
            colliderB->rigidBodyHandle,
            satResult.normal,
            satResult.depth
        );
        return;
    }

    bool noSolverResponseA =
        bodyA->motionControl == MotionControl::External ||
        bodyA->type == BodyType::Static ||
        bodyA->type == BodyType::Kinematic ||
        (bodyA->type == BodyType::Dynamic && bodyA->asleep && !wakeInfo.A);

    bool noSolverResponseB =
        bodyB->motionControl == MotionControl::External ||
        bodyB->type == BodyType::Static ||
        bodyB->type == BodyType::Kinematic ||
        (bodyB->type == BodyType::Dynamic && bodyB->asleep && !wakeInfo.B);

    // pointer data for solver
    ContactRuntime runtimeData = makeRuntimeData(
        bodyA, bodyB, colliderA, colliderB, caches->transforms.get(bodyA->rootTransformHandle, FUNC_NAME), caches->transforms.get(bodyB->rootTransformHandle, FUNC_NAME)
    );

    // collision manifold generation
    Contact contact(bodyHandleA, bodyHandleB, runtimeData, satResult.normal);
    contact.noSolverResponseA = noSolverResponseA;
    contact.noSolverResponseB = noSolverResponseB;

    bool contributesMotionA =
        contact.partnerTypeA == ContactPartnerType::RigidBody &&
        (bodyA->type == BodyType::Kinematic || (bodyA->type == BodyType::Dynamic && (!bodyA->asleep || wakeInfo.A)));

    bool contributesMotionB =
        contact.partnerTypeB == ContactPartnerType::RigidBody &&
        (bodyB->type == BodyType::Kinematic || (bodyB->type == BodyType::Dynamic && (!bodyB->asleep || wakeInfo.B)));

    contact.contributesMotionA = contributesMotionA;
    contact.contributesMotionB = contributesMotionB;

    collisionManifold->boxBox(contact, *contactCache, satResult);
}

//----------------------------------------------
//     Sphere vs Box
//----------------------------------------------
void NarrowphaseManager::processBoxSphere(
    RigidBodyHandle bodyHandleA,
    RigidBodyHandle bodyHandleB,
    ColliderHandle colliderHandleA,
    ColliderHandle colliderHandleB,
    RigidBody* bodyA,
    RigidBody* bodyB,
    Collider* colliderA,
    Collider* colliderB)
{
    // swap if A is not cuboid
    if (colliderA->type != ColliderType::CUBOID) {
        std::swap(bodyHandleA, bodyHandleB);
        std::swap(colliderHandleA, colliderHandleB);
        std::swap(bodyA, bodyB);
        std::swap(colliderA, colliderB);
    }

    colliderA->pose.ensureInvModelMatrix();

    SAT::Result satResult;
    if (!SAT::boxSphere(*colliderA, *colliderB, colliderA->pose, satResult)) {
        return;
    }

    bodyA->totalCollisionCount++;
    bodyB->totalCollisionCount++;

    WakeSleep::WakeUpInfo wakeInfo = WakeSleep::computeWakeUpInfo(*bodyA, *bodyB);
    WakeSleep::enqueueWakeRequests(
        wakeInfo,
        *bodyA, *bodyB,
        bodyHandleA, bodyHandleB,
        *toWake
    );

    // export to character controller
    bool aCharacter = bodyA->motionControl == MotionControl::External &&
        bodyA->responseMode == ContactResponseMode::Character;

    bool bCharacter = bodyB->motionControl == MotionControl::External &&
        bodyB->responseMode == ContactResponseMode::Character;

    if (aCharacter || bCharacter) {
        externalContacts.emplace_back(
            colliderA->rigidBodyHandle,
            colliderB->rigidBodyHandle,
            satResult.normal,
            satResult.depth
        );
        return;
    }

    bool noSolverResponseA =
        bodyA->motionControl == MotionControl::External ||
        bodyA->type == BodyType::Static ||
        bodyA->type == BodyType::Kinematic ||
        (bodyA->type == BodyType::Dynamic && bodyA->asleep && !wakeInfo.A);

    bool noSolverResponseB =
        bodyB->motionControl == MotionControl::External ||
        bodyB->type == BodyType::Static ||
        bodyB->type == BodyType::Kinematic ||
        (bodyB->type == BodyType::Dynamic && bodyB->asleep && !wakeInfo.B);

    // pointer data for solver
    ContactRuntime runtimeData = makeRuntimeData(
        bodyA, bodyB, colliderA, colliderB, caches->transforms.get(bodyA->rootTransformHandle, FUNC_NAME), caches->transforms.get(bodyB->rootTransformHandle, FUNC_NAME)
    );

    // collision manifold generation
    Contact contact(bodyHandleA, bodyHandleB, runtimeData, satResult.normal);
    contact.noSolverResponseA = noSolverResponseA;
    contact.noSolverResponseB = noSolverResponseB;

    bool contributesMotionA =
        contact.partnerTypeA == ContactPartnerType::RigidBody &&
        (bodyA->type == BodyType::Kinematic || (bodyA->type == BodyType::Dynamic && (!bodyA->asleep || wakeInfo.A)));

    bool contributesMotionB =
        contact.partnerTypeB == ContactPartnerType::RigidBody &&
        (bodyB->type == BodyType::Kinematic || (bodyB->type == BodyType::Dynamic && (!bodyB->asleep || wakeInfo.B)));

    contact.contributesMotionA = contributesMotionA;
    contact.contributesMotionB = contributesMotionB;

    collisionManifold->boxSphere(contact, *contactCache, satResult);
}

//----------------------------------------------
//     Sphere vs Sphere
//----------------------------------------------
void NarrowphaseManager::processSphereSphere(
    RigidBodyHandle bodyHandleA,
    RigidBodyHandle bodyHandleB,
    ColliderHandle colliderHandleA,
    ColliderHandle colliderHandleB,
    RigidBody* bodyA,
    RigidBody* bodyB,
    Collider* colliderA,
    Collider* colliderB)
{
    SAT::Result satResult;
    if (!SAT::sphereSphere(*colliderA, *colliderB, satResult)) {
        return;
    }

    bodyA->totalCollisionCount++;
    bodyB->totalCollisionCount++;

    WakeSleep::WakeUpInfo wakeInfo = WakeSleep::computeWakeUpInfo(*bodyA, *bodyB);
    WakeSleep::enqueueWakeRequests(
        wakeInfo,
        *bodyA, *bodyB,
        bodyHandleA, bodyHandleB,
        *toWake
    );

    // export to character controller
    bool aCharacter = bodyA->motionControl == MotionControl::External &&
        bodyA->responseMode == ContactResponseMode::Character;

    bool bCharacter = bodyB->motionControl == MotionControl::External &&
        bodyB->responseMode == ContactResponseMode::Character;

    if (aCharacter || bCharacter) {
        externalContacts.emplace_back(
            colliderA->rigidBodyHandle,
            colliderB->rigidBodyHandle,
            satResult.normal,
            satResult.depth
        );
        return;
    }

    bool noSolverResponseA =
        bodyA->motionControl == MotionControl::External ||
        bodyA->type == BodyType::Static ||
        bodyA->type == BodyType::Kinematic ||
        (bodyA->type == BodyType::Dynamic && bodyA->asleep && !wakeInfo.A);

    bool noSolverResponseB =
        bodyB->motionControl == MotionControl::External ||
        bodyB->type == BodyType::Static ||
        bodyB->type == BodyType::Kinematic ||
        (bodyB->type == BodyType::Dynamic && bodyB->asleep && !wakeInfo.B);

    // pointer data for solver
    ContactRuntime runtimeData = makeRuntimeData(
        bodyA, bodyB, colliderA, colliderB, caches->transforms.get(bodyA->rootTransformHandle, FUNC_NAME), caches->transforms.get(bodyB->rootTransformHandle, FUNC_NAME)
    );

    // collision manifold generation
    Contact contact(bodyHandleA, bodyHandleB, runtimeData, satResult.normal);
    contact.noSolverResponseA = noSolverResponseA;
    contact.noSolverResponseB = noSolverResponseB;

    bool contributesMotionA =
        contact.partnerTypeA == ContactPartnerType::RigidBody &&
        (bodyA->type == BodyType::Kinematic || (bodyA->type == BodyType::Dynamic && (!bodyA->asleep || wakeInfo.A)));

    bool contributesMotionB =
        contact.partnerTypeB == ContactPartnerType::RigidBody &&
        (bodyB->type == BodyType::Kinematic || (bodyB->type == BodyType::Dynamic && (!bodyB->asleep || wakeInfo.B)));

    contact.contributesMotionA = contributesMotionA;
    contact.contributesMotionB = contributesMotionB;

    collisionManifold->sphereSphere(contact, *contactCache, satResult);
}