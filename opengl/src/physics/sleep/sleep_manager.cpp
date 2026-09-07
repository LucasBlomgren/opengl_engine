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
//  Put a body to sleep and move it to asleep in broadphase
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
//  Process contact wake-ups
//=====================================================
void SleepManager::processContactWakeUps(
    const ContactBatch& batch)
{
    for (const Contact* contact : batch.contacts) {
        if (!contact) {
            continue;
        }

        RigidBody* A = contact->runtimeData.bodyA;
        RigidBody* B = contact->runtimeData.bodyB;

        // Terrain on B
        if (!A || !B) {
            continue;
        }

        SleepState* sleepA = nullptr;
        SleepState* sleepB = nullptr;

        if (A->type == BodyType::Dynamic) {
            sleepA = &world.getSleepState(A->sleepStateHandle);
        }

        if (B->type == BodyType::Dynamic) {
            sleepB = &world.getSleepState(B->sleepStateHandle);
        }

        auto [wakeA, wakeB] = computeWakeUp(*A, *B, sleepA, sleepB);

        if (wakeA) {
            wakeBody(contact->bodyA, *A, *sleepA);
        }

        if (wakeB) {
            wakeBody(contact->bodyB, *B, *sleepB);
        }
    }

    // Speculative contacts
    for (const SpeculativeContact& contact :
        batch.speculativeContacts)
    {
        RigidBody* A = contact.bodyA;
        RigidBody* B = contact.bodyB;

        // Terrain on one side
        if (!A || !B) {
            continue;
        }

        SleepState* sleepA = nullptr;
        SleepState* sleepB = nullptr;

        if (A->type == BodyType::Dynamic) {
            sleepA = &world.getSleepState(A->sleepStateHandle);
        }

        if (B->type == BodyType::Dynamic) {
            sleepB = &world.getSleepState(B->sleepStateHandle);
        }

        auto [wakeA, wakeB] =
            computeWakeUp(*A, *B, sleepA, sleepB);

        if (wakeA) {
            wakeBody(contact.bodyHandleA, *A, *sleepA);
        }

        if (wakeB) {
            wakeBody(contact.bodyHandleB, *B, *sleepB);
        }
    }
}

std::pair<bool, bool> SleepManager::computeWakeUp(
    const RigidBody& A,
    const RigidBody& B,
    const SleepState* sleepA,
    const SleepState* sleepB)
{
    const float Av2 = glm::dot(A.linearVelocity, A.linearVelocity);
    const float Aw2 = glm::dot(A.angularVelocity, A.angularVelocity);
    const float Bv2 = glm::dot(B.linearVelocity, B.linearVelocity);
    const float Bw2 = glm::dot(B.angularVelocity, B.angularVelocity);

    bool wakeA =
        sleepA &&
        sleepA->asleep &&
        (Bv2 > v2_threshold || Bw2 > w2_threshold);

    bool wakeB =
        sleepB &&
        sleepB->asleep &&
        (Av2 > v2_threshold || Aw2 > w2_threshold);

    return { wakeA, wakeB };
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
    sleepState.collisionHistory.push(sleepState.collisionCount);
    sleepState.collisionCount = 0;
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
void SleepManager::applySleepDamping(const std::vector<BodyHandle>& awakeHandles, float dt) 
{
    for (const BodyHandle& handle : awakeHandles) {
        RigidBody& body = world.getBody(handle);

        if (body.type != BodyType::Dynamic) continue;

        SleepState& sleepState = world.getSleepState(body.sleepStateHandle);
        if (sleepState.asleep) continue;
        if (!sleepState.allowSleep) continue;
        if (sleepState.inSleepTransition) continue;

        float sleepT = glm::clamp(sleepState.counter / sleepState.counterThreshold, 0.0f, 1.0f);

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