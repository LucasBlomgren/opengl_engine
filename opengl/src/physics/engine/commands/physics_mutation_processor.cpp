#include "pch.h"

#include "physics/engine/physics_engine_impl.h"

namespace {
    void refreshBodyInertia(
        PhysicsWorld& physicsWorld,
        RigidBody& body)
    {
        if (body.type != BodyType::Dynamic || body.colliderHandles.empty()) {
            return;
        }

        Collider* collider = nullptr;

        for (ColliderHandle colliderHandle : body.colliderHandles) {
            if (!physicsWorld.isColliderActive(colliderHandle)) {
                continue;
            }

            Collider* candidate = physicsWorld.getCollider(colliderHandle);

            if (candidate && candidate->enabled) {
                collider = candidate;
                break;
            }
        }

        if (!collider) {
            return;
        }

        Transform inertiaTransform;

        if (collider->type == ColliderType::SPHERE) {
            const Sphere& sphere = std::get<Sphere>(collider->shape);
            inertiaTransform.scale = glm::vec3(sphere.radiusWorld * 2.0f);
        }
        else {
            inertiaTransform.scale = collider->worldScale;
        }

        body.calculateInverseInertia(collider->type, *collider, inertiaTransform);
        body.updateInertiaWorld();
    }
}

//=========================================
// Mutation command processing
//=========================================
void PhysicsEngine::Impl::applyMutationCommands(
    const std::vector<PhysicsCommandBuffer::Mutation>& mutations)
{
    for (const PhysicsCommandBuffer::Mutation& mutation : mutations) {
        std::visit([this](const auto& command) {
            applyCommand(command);
        }, mutation);
    }
}

void PhysicsEngine::Impl::applyCommand(
    const PhysicsCommandBuffer::ApplyLinearImpulse& command)
{
    (void)command;
}

void PhysicsEngine::Impl::applyCommand(
    const PhysicsCommandBuffer::SetLinearVelocity& command)
{
    (void)command;
}

void PhysicsEngine::Impl::applyCommand(
    const PhysicsCommandBuffer::SetAngularVelocity& command)
{
    (void)command;
}

void PhysicsEngine::Impl::applyCommand(
    const PhysicsCommandBuffer::SetKinematicTarget& command)
{
    (void)command;
}

void PhysicsEngine::Impl::applyCommand(
    const PhysicsCommandBuffer::SetRigidBodyAwake& command)
{
    (void)command;
}

void PhysicsEngine::Impl::applyCommand(
    const PhysicsCommandBuffer::SetRigidBodyType& command)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (!body || !physicsWorld.isRigidBodyActive(command.body)) {
        return;
    }

    body->type = command.type;

    switch (command.type) {
    case BodyType::Dynamic:
        if (body->mass <= 0.0f) {
            body->mass = 1.0f;
        }

        body->invMass = 1.0f / body->mass;

        if (body->sleepCounterThreshold <= 0.0f) {
            body->sleepCounterThreshold = 1.5f;
        }

        refreshBodyInertia(physicsWorld, *body);

        if (body->allowSleep && body->asleep) {
            body->setAsleep();
            broadphaseManager.moveToAsleep(command.body);
        }
        else {
            body->setAwake();
            broadphaseManager.moveToAwake(command.body);
        }

        break;

    case BodyType::Kinematic:
        body->invMass = 0.0f;
        body->linearVelocity = glm::vec3(0.0f);
        body->angularVelocity = glm::vec3(0.0f);
        body->setAwake();
        broadphaseManager.moveToAwake(command.body);
        break;

    case BodyType::Static:
        body->mass = 0.0f;
        body->invMass = 0.0f;
        body->linearVelocity = glm::vec3(0.0f);
        body->angularVelocity = glm::vec3(0.0f);
        broadphaseManager.moveToStatic(command.body);
        break;
    }

    // Contact invalidation remains unchanged until it can be selective.
}

void PhysicsEngine::Impl::applyCommand(
    const PhysicsCommandBuffer::SetRigidBodyMotionControl& command)
{
    (void)command;
}

void PhysicsEngine::Impl::applyCommand(
    const PhysicsCommandBuffer::SetColliderLocalPose& command)
{
    (void)command;
}

void PhysicsEngine::Impl::applyCommand(
    const PhysicsCommandBuffer::SetColliderEnabled& command)
{
    (void)command;
}

void PhysicsEngine::Impl::applyCommand(
    const PhysicsCommandBuffer::SetColliderTrigger& command)
{
    (void)command;
}

void PhysicsEngine::Impl::applyCommand(
    const PhysicsCommandBuffer::SleepAllObjects&)
{
    auto& bodyMap = physicsWorld.getRigidBodiesMap();
    auto& dense = bodyMap.dense();

    for (uint32_t i = 0; i < static_cast<uint32_t>(dense.size()); ++i) {
        RigidBody& body = dense[i];

        if (body.asleep) continue;
        if (body.type == BodyType::Static) continue;
        if (body.type == BodyType::Kinematic) continue;
        if (body.motionControl == MotionControl::External) continue;

        RigidBodyHandle handle = bodyMap.handle_from_dense_index(i);
        broadphaseManager.moveToAsleep(handle);
    }
}

void PhysicsEngine::Impl::applyCommand(
    const PhysicsCommandBuffer::AwakenAllObjects&)
{
    auto& bodyMap = physicsWorld.getRigidBodiesMap();
    auto& dense = bodyMap.dense();

    for (uint32_t i = 0; i < static_cast<uint32_t>(dense.size()); ++i) {
        RigidBody& body = dense[i];

        if (!body.asleep) continue;
        if (body.type == BodyType::Static) continue;
        if (body.type == BodyType::Kinematic) continue;
        if (body.motionControl == MotionControl::External) continue;

        RigidBodyHandle handle = bodyMap.handle_from_dense_index(i);
        broadphaseManager.moveToAwake(handle);
    }
}

void PhysicsEngine::Impl::applyCommand(
    const PhysicsCommandBuffer::SyncBodyFromTransform& command)
{
    RigidBody* body = caches.bodies.get(command.body, FUNC_NAME);

    if (!body) {
        return;
    }

    Transform* rootTransform =
        caches.transforms.get(body->rootTransformHandle, FUNC_NAME);

    if (!rootTransform) {
        return;
    }

    rootTransform->updateCache();

    body->pose.position = rootTransform->position;
    body->pose.orientation = rootTransform->orientation;
    body->scale = rootTransform->scale;

    updateCollidersAndBodyAABB(body, rootTransform);

    if (!body->colliderHandles.empty()) {
        Collider* mainCollider =
            caches.colliders.get(body->colliderHandles[0], FUNC_NAME);

        if (mainCollider) {
            Transform inertiaTransform;

            if (mainCollider->type == ColliderType::SPHERE) {
                const Sphere& sphere = std::get<Sphere>(mainCollider->shape);
                inertiaTransform.scale =
                    glm::vec3(sphere.radiusWorld * 2.0f);
            }
            else {
                inertiaTransform.scale = mainCollider->worldScale;
            }

            body->calculateInverseInertia(
                mainCollider->type,
                *mainCollider,
                inertiaTransform);
        }
    }

    body->updateInertiaWorld();

    if (body->broadphaseHandle.bucket != BroadphaseBucket::None) {
        setBVHDirty(command.body);
    }
}
