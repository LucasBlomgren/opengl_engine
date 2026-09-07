#include "sleep_manager.h"

#include "physics/world/physics_world.h"
#include "physics/broadphase/broadphase_manager.h"
#include "physics/narrowphase/narrowphase_types.h"
#include "physics/narrowphase/collision_manifold.h"

namespace physics::internal {

SleepManager::SleepManager(
    PhysicsWorld& world,
    BroadphaseManager& broadphase)
    : world(world)
    , broadphase(broadphase)
{}

//=====================================================
//  Ensure that a body has a sleep state, 
//  creating one if necessary
//=====================================================
SleepState& SleepManager::ensureSleepState(BodyHandle handle) {
    RigidBody& body = world.getBody(handle);

    SleepState* sleepStatePtr =
        world.tryGetSleepState(body.sleepStateHandle);

    SleepStateHandle sleepStateHandle;

    if (sleepStatePtr) {
        sleepStateHandle = body.sleepStateHandle;
    }
    else {
        SleepState newSleepState(
            true, // allow sleep
            true, // start awake
            body.pose.position // anchor point
        );

        sleepStateHandle =
            world.commitSleepState(std::move(newSleepState));

        body.sleepStateHandle = sleepStateHandle;
    }

    return world.getSleepState(sleepStateHandle);
}

//=====================================================
//  Wake up a body and move it to awake in broadphase
//=====================================================
void SleepManager::wakeBody(
    BodyHandle handle,
    RigidBody& body,
    SleepState& sleepState)
{
    if (body.type != BodyType::Dynamic || 
        !sleepState.asleep) {
        return;
    }

    sleepState.asleep = false;
    sleepState.counter = 0.0f;
    sleepState.anchorTimer = 0.0f;
    sleepState.anchorPoint = body.pose.position;

    broadphase.moveToAwake(handle);
}

//=====================================================
//  Sleep a body and move it to asleep in broadphase
//=====================================================
void SleepManager::sleepBody(
    const BodyHandle handle,
    SleepState& sleepState)
{
    RigidBody& body = world.getBody(handle);

    if (body.type != BodyType::Dynamic || 
        sleepState.asleep ||
        !sleepState.allowSleep) {
        return;
    }
    sleepState.asleep = true;
    sleepState.counter = 0.0f;
    sleepState.anchorTimer = 0.0f;
    sleepState.anchorPoint = body.pose.position;

    body.linearVelocity = glm::vec3(0.0f);
    body.angularVelocity = glm::vec3(0.0f);
    body.biasLinearVelocity = glm::vec3(0.0f);
    body.biasAngularVelocity = glm::vec3(0.0f);

    broadphase.moveToAsleep(handle);
}

//=====================================================
//  Set whether a body is allowed to sleep
//=====================================================
void SleepManager::setBodyAllowSleep(
    const BodyHandle handle,
    bool allowSleep)
{
    RigidBody& body = world.getBody(handle);
    SleepState& sleepState = world.getSleepState(body.sleepStateHandle);
    sleepState.allowSleep = allowSleep;

    if (!allowSleep) {
        wakeBody(handle, body, sleepState);
    }
}

//=====================================================
//  Process contact wake-ups
//=====================================================
void SleepManager::processContactWakeUps(
    const ContactBatch& batch)
{
    wakeMotionBySlot.assign(
        world.bodyStorage().slot_capacity(),
        WakeMotion{}
    );

    // Wake up bodies based on regular contacts
    for (const Contact* contact : batch.contacts) {
        if (!contact) {
            continue;
        }

        RigidBody* A = contact->runtimeData.bodyA;
        RigidBody* B = contact->runtimeData.bodyB;

        // Only B is allowed to be null, as it can be a terrain contact
        if (!A) continue;

        // Body A is dynamic, check if it should wake up due to contact with B
        if (A->type == BodyType::Dynamic) {
            SleepState& sleepA = world.getSleepState(A->sleepStateHandle);

            sleepA.collisionCount++;

            if (B && shouldWake(sleepA, contact->bodyB, *B)) {
                wakeBody(contact->bodyA, *A, sleepA);
            }
        }

        // Body B is dynamic, check if it should wake up due to contact with A
        if (B && B->type == BodyType::Dynamic) {
            SleepState& sleepB = world.getSleepState(B->sleepStateHandle);

            sleepB.collisionCount++;

            if (shouldWake(sleepB, contact->bodyA, *A)) {
                wakeBody(contact->bodyB, *B, sleepB);
            }
        }
    }

    // Wake up bodies based on speculative contacts
    for (const SpeculativeContact& contact :
        batch.speculativeContacts)
    {
        RigidBody* A = contact.bodyA;
        RigidBody* B = contact.bodyB;

        if (!A) {
            continue;
        }

        // Body A is dynamic, check if it should wake up due to contact with B
        if (A->type == BodyType::Dynamic) {
            SleepState& sleepA = world.getSleepState(A->sleepStateHandle);

            if (B && shouldWake(sleepA, contact.bodyHandleB, *B)) {
                wakeBody(contact.bodyHandleA, *A, sleepA);
            }
        }

        if (B && B->type == BodyType::Dynamic) {
            SleepState& sleepB =
                world.getSleepState(B->sleepStateHandle);

            // Body B is dynamic, check if it should wake up due to contact with A
            if (shouldWake(sleepB, contact.bodyHandleA, *A)) {
                wakeBody(contact.bodyHandleB, *B, sleepB);
            }
        }
    }
}

bool SleepManager::shouldWake(
    const SleepState& sleepState,
    BodyHandle otherHandle,
    const RigidBody& otherBody)
{
    if (!sleepState.asleep) {
        return false;
    }

    const WakeMotion& motion =
        getWakeMotion(otherHandle, otherBody);

    return motion.v2 > v2_threshold ||
           motion.w2 > w2_threshold;
}

const SleepManager::WakeMotion& SleepManager::getWakeMotion(
    BodyHandle handle,
    const RigidBody& body)
{
    WakeMotion& motion = wakeMotionBySlot[handle.slot];

    if (!motion.valid) {
        motion.v2 = glm::dot(body.linearVelocity, body.linearVelocity);
        motion.w2 = glm::dot(body.angularVelocity, body.angularVelocity);
        motion.valid = true;
    }

    return motion;
}

//=====================================================
//  Process sleep candidates
//=====================================================
void SleepManager::processSleepCandidates(
    const std::vector<BodyHandle>& awakeHandles, 
    float dt) 
{
    for (const BodyHandle& handle : awakeHandles) {
        RigidBody& body = world.getBody(handle);
        if (body.type != BodyType::Dynamic) continue;

        SleepState& sleepState = world.getSleepState(body.sleepStateHandle);
        if (sleepState.asleep) continue;
        if (!sleepState.allowSleep) continue;

        // Update collision history
        sleepState.collisionHistory.push(sleepState.collisionCount);
        sleepState.collisionCount = 0;

        if (sleepState.getCollisionHistoryAverage() <= 0.0f) {
            continue;
        }

        // Sleep check based on anchor point:
        // if the body is staying close to the anchor point, increment the
        // anchor timer.
        // Example: even if the body is moving fast, but is stuck somewhere,
        // it will eventually go to sleep.
        if (glm::abs(sleepState.anchorPoint.x - body.pose.position.x) < jitterThreshold &&
            glm::abs(sleepState.anchorPoint.y - body.pose.position.y) < jitterThreshold &&
            glm::abs(sleepState.anchorPoint.z - body.pose.position.z) < jitterThreshold)
        {
            sleepState.anchorTimer += dt;
        }
        else {
            sleepState.anchorTimer = glm::max(0.0f, sleepState.anchorTimer - dt);
        }

        if (sleepState.anchorTimer == 0.0f) {
            sleepState.anchorPoint = body.pose.position;
        }

        if (sleepState.anchorTimer >= anchorTimerThreshold) {
            sleepBody(handle, sleepState);
            continue;
        }

        // Sleep check based on velocity thresholds:
        // if the body is moving slowly, increment the sleep counter.
        std::optional<std::pair<float, float>> thresholds = 
            calculateSleepThresholds(body, sleepState);

        if (!thresholds) {
            continue;
        }

        float linearVelocityThreshold = thresholds->first;
        float angularVelocityThreshold = thresholds->second;

        if (glm::length(body.linearVelocity) < linearVelocityThreshold &&
            glm::length(body.angularVelocity) < angularVelocityThreshold)
        {
            sleepState.counter += dt;
        }
        else {
            sleepState.counter = 0.0f;
        }

        if (sleepState.counter >= sleepState.counterThreshold) {
            sleepBody(handle, sleepState);
        }
    }
}

std::optional<std::pair<float, float>> SleepManager::calculateSleepThresholds(
    RigidBody& body,
    SleepState& sleepState)
{
    float avg = sleepState.collisionHistory.average();

    if (avg <= 0.0f) {
        if (std::abs(avg - sleepState.lastAvg) >= 1) {
            sleepState.counter = 0.0f;
        }
        sleepState.lastAvg = avg;

        return std::nullopt;
    }

    avg = std::max(avg, 1.0f);
    sleepState.lastAvg = avg;

    constexpr float linearFactor = 0.2f;
    constexpr float angularFactor = 0.1f;

    // set thresholds
    float linearVelocityThreshold = avg * linearFactor;
    float angularVelocityThreshold = avg * angularFactor * body.invRadius;

    return std::pair<float, float>(linearVelocityThreshold, angularVelocityThreshold);
}

//=====================================================
//  Add sleep damping to bodies that are close to going to sleep
//=====================================================
void SleepManager::applySleepDamping(
    const std::vector<BodyHandle>& awakeHandles, 
    float dt) 
{
    for (const BodyHandle& handle : awakeHandles) {
        RigidBody& body = world.getBody(handle);

        if (body.type != BodyType::Dynamic) continue;

        SleepState& sleepState = world.getSleepState(body.sleepStateHandle);
        if (sleepState.asleep) continue;
        if (!sleepState.allowSleep) continue;

        float sleepT = 
            glm::clamp(sleepState.counter / sleepState.counterThreshold, 0.0f, 1.0f);

        // Smoothstep
        sleepT = sleepT * sleepT * (3.0f - 2.0f * sleepT);

        constexpr float linearDampingStrength = 5.0f;
        constexpr float angularDampingStrength = 4.5f;

        // Apply damping
        float linearFactor = std::exp(-linearDampingStrength * sleepT * dt);
        float angularFactor = std::exp(-angularDampingStrength * sleepT * dt);

        body.linearVelocity *= linearFactor;
        body.angularVelocity *= angularFactor;
    }
}
}