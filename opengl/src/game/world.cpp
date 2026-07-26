#include "pch.h"
#include "world.h"

#include "graphics/renderer/renderer.h"
#include "graphics/textures/texture_manager.h"
#include "graphics/mesh/mesh_manager.h"
#include "graphics/shaders/shader_manager.h"

#include "physics/physics_engine.h"
#include "physics/public/physics_handles.h"

#include "game/game_handles.h"


void World::clear() {
    gameObjects.clear();
    transforms.clear();
    objectId = 0;
}

//----------------------------------
//  Getters
//----------------------------------
GameObject* World::getGameObject(const GameObjectHandle& h) {
    return gameObjects.try_get(h);
    //return gameObjects.get(h);
}
Transform* World::getTransform(const TransformHandle& h) {
    //return transforms.try_get(h);
    return transforms.get(h);
}

RigidBody* World::getRigidBody(const GameObjectHandle& h) {
    GameObject* obj = gameObjects.try_get(h);
    if (!obj) return nullptr;
    return physicsEngine.getPhysicsWorld()->getRigidBody(obj->rigidBodyHandle);
}
RigidBody* World::getRigidBody(const RigidBodyHandle& h) {
    return physicsEngine.getPhysicsWorld()->getRigidBody(h);
}

Collider* World::getCollider(const ColliderHandle& h) {
    return physicsEngine.getPhysicsWorld()->getCollider(h);
}

//----------------------------------
//  Creation
//----------------------------------
TransformHandle World::createTransform(const glm::vec3& position, const glm::quat& orientation, const glm::vec3& scale) {
    return transforms.create(position, orientation, scale);
}

GameObjectHandle World::createGameObject(GameObjectDesc& objDesc) {
    Transform* rootTransform = transforms.try_get(objDesc.rootTransformHandle);

    if (!rootTransform) {
        std::cerr << "[World] Cannot create GameObject '" << objDesc.name << "': invalid root transform.\n";
        return {};
    }

    //----------------------------------
    // Create game object
    //----------------------------------
    GameObjectHandle gameObjectHandle = gameObjects.create(objectId, objDesc.rootTransformHandle);
    GameObject* gameObject = gameObjects.try_get(gameObjectHandle);

    if (!gameObject) {
        return {};
    }

    gameObject->name = objDesc.name;

    //----------------------------------
    // Create pending rigid body
    //----------------------------------
    RigidBodyDesc bodyDesc;
    bodyDesc.pose.position = rootTransform->position;
    bodyDesc.pose.orientation = rootTransform->orientation;
    bodyDesc.scale = rootTransform->scale;
    bodyDesc.type = objDesc.bodyType;
    bodyDesc.mass = objDesc.mass;
    bodyDesc.allowSleep = objDesc.allowSleep;
    bodyDesc.startAsleep = objDesc.asleep;
    bodyDesc.sleepCounterThreshold = objDesc.sleepCounterThreshold;

    RigidBodyHandle bodyHandle = physicsEngine.createRigidBody(bodyDesc);

    if (!bodyHandle.isValid()) {
        std::cerr << "[World] Cannot create physics body for GameObject '" << objDesc.name << "'.\n";
        gameObjects.destroy(gameObjectHandle);
        return {};
    }

    gameObject->rigidBodyHandle = bodyHandle;

    //----------------------------------
    // Temporary legacy body binding
    //----------------------------------
    RigidBody* body = getRigidBody(bodyHandle);

    if (!body) {
        std::cerr << "[World] Cannot access pending rigid body for GameObject '" << objDesc.name << "'.\n";

        physicsEngine.destroyRigidBody(bodyHandle);
        gameObjects.destroy(gameObjectHandle);

        return {};
    }

    body->gameObjectHandle = gameObjectHandle;
    body->rootTransformHandle = objDesc.rootTransformHandle;

    body->pose.position = rootTransform->position;
    body->pose.orientation = rootTransform->orientation;
    body->scale = rootTransform->scale;
    body->anchorPoint = rootTransform->position;

    //----------------------------------
    // Create parts and pending colliders
    //----------------------------------
    std::vector<ColliderHandle> createdColliderHandles;
    createdColliderHandles.reserve(objDesc.parts.size());

    for (const SubPartDesc& partDesc : objDesc.parts) {
        Transform* partTransform = transforms.try_get(partDesc.localTransformHandle);

        if (!partTransform) {
            std::cerr << "[World] Skipping part '" << partDesc.name << "' with invalid local transform.\n";
            continue;
        }

        SubPart part;
        part.name = partDesc.name;
        part.localTransformHandle = partDesc.localTransformHandle;
        part.shader = shaderManager.getShader(partDesc.shaderName);
        part.mesh = meshManager.getMesh(partDesc.meshName);

        if (!part.mesh) {
            std::cerr << "[World] Warning: Mesh not found for part '" << partDesc.name << "' of GameObject '" << objDesc.name << "'. Using default cube mesh.\n";
            part.mesh = meshManager.getMesh("cube");
        }

        if (!part.mesh) {
            std::cerr << "[World] Skipping part '" << partDesc.name << "': fallback cube mesh was not found.\n";
            continue;
        }

        if (partDesc.textureName == "plain") {
            part.textureId = 999;
        }
        else {
            part.textureId = textureManager.getTexture(partDesc.textureName);
        }

        part.color = partDesc.color;
        part.seeThrough = partDesc.seeThrough;

        //----------------------------------
        // Calculate local mesh bounds
        //----------------------------------
        glm::vec3 localCenter{ 0.0f };
        glm::vec3 localHalfExtents{ 0.5f };

        if (!part.mesh->vertices.empty()) {
            glm::vec3 minimum = part.mesh->vertices.front().position;
            glm::vec3 maximum = part.mesh->vertices.front().position;

            for (const Vertex& vertex : part.mesh->vertices) {
                minimum = glm::min(minimum, vertex.position);
                maximum = glm::max(maximum, vertex.position);
            }

            localCenter = (minimum + maximum) * 0.5f;
            localHalfExtents = (maximum - minimum) * 0.5f;
        }

        //----------------------------------
        // Build public collider descriptor
        //----------------------------------
        ColliderDesc colliderDesc;
        colliderDesc.localPose.position = partTransform->position;
        colliderDesc.localPose.orientation = partTransform->orientation;
        colliderDesc.localScale = partTransform->scale;
        colliderDesc.enabled = true;
        colliderDesc.isTrigger = false;

        if (partDesc.colliderType == ColliderType::CUBOID) {
            BoxShapeDesc boxDesc;
            boxDesc.center = localCenter;
            boxDesc.halfExtents = localHalfExtents;
            colliderDesc.shape = boxDesc;
        }
        else if (partDesc.colliderType == ColliderType::SPHERE) {
            float radius = 0.5f;

            if (!part.mesh->vertices.empty()) {
                radius = 0.0f;

                for (const Vertex& vertex : part.mesh->vertices) {
                    radius = (std::max)(radius, glm::length(vertex.position - localCenter));
                }
            }

            SphereShapeDesc sphereDesc;
            sphereDesc.center = localCenter;
            sphereDesc.radius = radius;
            colliderDesc.shape = sphereDesc;
        }
        else {
            std::cerr << "[World] Skipping part '" << partDesc.name << "': unsupported collider type.\n";
            continue;
        }

        //----------------------------------
        // Create pending collider
        //----------------------------------
        ColliderHandle colliderHandle = physicsEngine.createCollider(bodyHandle, colliderDesc);

        if (!colliderHandle.isValid()) {
            std::cerr << "[World] Failed to create collider for part '" << partDesc.name << "'.\n";
            continue;
        }

        Collider* collider = getCollider(colliderHandle);

        if (!collider) {
            std::cerr << "[World] Cannot access pending collider for part '" << partDesc.name << "'.\n";
            physicsEngine.destroyCollider(colliderHandle);
            continue;
        }

        //----------------------------------
        // Temporary legacy collider binding
        //----------------------------------
        collider->localTransformHandle = partDesc.localTransformHandle;

        collider->localPose.position = partTransform->position;
        collider->localPose.orientation = partTransform->orientation;
        collider->localScale = partTransform->scale;

        collider->updateWorldPose(body->pose, body->scale);
        collider->transformCache.ensureModelMatrix(collider->worldPose);
        collider->updateShape();
        collider->updateAABB();

        //----------------------------------
        // Temporary pending body-collider link
        //----------------------------------
        if (std::find(body->colliderHandles.begin(), body->colliderHandles.end(), colliderHandle) == body->colliderHandles.end()) {
            body->colliderHandles.push_back(colliderHandle);
        }

        part.colliderHandle = colliderHandle;

        gameObject->parts.push_back(part);
        createdColliderHandles.push_back(colliderHandle);
    }

    //----------------------------------
    // Validate collider creation
    //----------------------------------
    if (createdColliderHandles.empty()) {
        std::cerr << "[World] Cannot create GameObject '" << objDesc.name << "': no colliders were created.\n";

        physicsEngine.destroyRigidBody(bodyHandle);
        gameObjects.destroy(gameObjectHandle);

        return {};
    }

    //----------------------------------
    // Build temporary body AABB
    //----------------------------------
    Collider* mainCollider = nullptr;

    for (ColliderHandle colliderHandle : createdColliderHandles) {
        Collider* collider = getCollider(colliderHandle);

        if (collider && collider->enabled) {
            mainCollider = collider;
            break;
        }
    }

    if (!mainCollider) {
        std::cerr << "[World] Cannot create GameObject '" << objDesc.name << "': no enabled colliders were created.\n";

        physicsEngine.destroyRigidBody(bodyHandle);
        gameObjects.destroy(gameObjectHandle);

        return {};
    }

    body->aabb = mainCollider->getAABB();

    for (ColliderHandle colliderHandle : createdColliderHandles) {
        Collider* collider = getCollider(colliderHandle);

        if (!collider || !collider->enabled || collider == mainCollider) {
            continue;
        }

        const AABB& colliderAABB = collider->getAABB();

        body->aabb.growToInclude(colliderAABB.worldMin);
        body->aabb.growToInclude(colliderAABB.worldMax);
    }

    body->aabb.worldCenter = (body->aabb.worldMin + body->aabb.worldMax) * 0.5f;
    body->aabb.worldHalfExtents = (body->aabb.worldMax - body->aabb.worldMin) * 0.5f;

    const float boundingRadius = glm::length(body->aabb.worldHalfExtents);
    body->invRadius = boundingRadius > 0.0f ? 1.0f / boundingRadius : 0.0f;

    //----------------------------------
    // Build temporary body inertia
    //----------------------------------
    if (body->type == BodyType::Dynamic) {
        Transform inertiaTransform;

        if (mainCollider->type == ColliderType::SPHERE) {
            const Sphere& sphere = std::get<Sphere>(mainCollider->shape);
            inertiaTransform.scale = glm::vec3(sphere.radiusWorld * 2.0f);
        }
        else {
            inertiaTransform.scale = mainCollider->worldScale;
        }

        body->calculateInverseInertia(mainCollider->type, *mainCollider, inertiaTransform);
        body->updateInertiaWorld();
    }

    //----------------------------------
    // Register for rendering
    //----------------------------------
    renderer.addObjectToBatch(gameObjectHandle);

    ++objectId;

    return gameObjectHandle;
}
//----------------------------------
//  Deletion
//----------------------------------
void World::deleteGameObject(GameObjectHandle handle) {
    GameObject* obj = gameObjects.try_get(handle);
    if (obj) {
        physicsEngine.destroyRigidBody(obj->rigidBodyHandle);
        renderer.removeObjectFromBatch(handle);
        gameObjects.destroy(handle);
    }
    else {
        std::cerr << "[World] Warning: Tried to delete non-existing GameObject with handle (slot: " << handle.slot << ", gen: " << handle.gen << ")\n";
    }

}
