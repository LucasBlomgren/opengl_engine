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

// physics getters, #TODO: refactor to world only dependent on physics world, not physics engine
RigidBody* World::getRigidBody(const GameObjectHandle& h) {
    GameObject* obj = gameObjects.try_get(h);
    if (!obj) return nullptr;
    return physicsEngine.getPhysicsWorld()->getRigidBodiesMap().try_get(obj->rigidBodyHandle);
}
RigidBody* World::getRigidBody(const RigidBodyHandle& h) {
    return physicsEngine.getPhysicsWorld()->getRigidBodiesMap().try_get(h);
}

Collider* World::getCollider(const ColliderHandle& h) {
    return physicsEngine.getPhysicsWorld()->getCollidersMap().try_get(h);
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
        std::cerr << "[World] Cannot create GameObject '" << objDesc.name
            << "': invalid root transform.\n";
        return {};
    }

    GameObjectHandle gameObjectHandle = gameObjects.create(objectId, objDesc.rootTransformHandle);
    GameObject& gameObject = *gameObjects.try_get(gameObjectHandle);
    gameObject.name = objDesc.name;

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
        std::cerr << "[World] Cannot create physics body for GameObject '"
            << objDesc.name << "'.\n";
        gameObjects.destroy(gameObjectHandle);
        return {};
    }

    gameObject.rigidBodyHandle = bodyHandle;

    // Temporary GameObject/Transform binding used by the current renderer,
    // editor and raycast code. Body creation itself goes through the public API.
    RigidBody* body = getRigidBody(bodyHandle);
    if (body) {
        body->gameObjectHandle = gameObjectHandle;
        body->rootTransformHandle = objDesc.rootTransformHandle;
    }

    for (const SubPartDesc& partDesc : objDesc.parts) {
        Transform* partTransform = transforms.try_get(partDesc.localTransformHandle);
        if (!partTransform) {
            std::cerr << "[World] Skipping part '" << partDesc.name
                << "' with invalid local transform.\n";
            continue;
        }

        SubPart part;
        part.name = partDesc.name;
        part.localTransformHandle = partDesc.localTransformHandle;
        part.shader = shaderManager.getShader(partDesc.shaderName);
        part.mesh = meshManager.getMesh(partDesc.meshName);

        if (part.mesh == nullptr) {
            std::cerr << "[World] Warning: Mesh not found for part '" << partDesc.name << "' of GameObject '" << objDesc.name << "'. Using default cube mesh.\n";
            part.mesh = meshManager.getMesh("cube");
        }

        if (partDesc.textureName == "plain") {
            part.textureId = 999;
        } else {
            part.textureId = textureManager.getTexture(partDesc.textureName);
        }

        part.color = partDesc.color;
        part.seeThrough = partDesc.seeThrough;

        std::vector<glm::vec3> verticesPositions;
        for (const Vertex& vertex : part.mesh->vertices) {
            verticesPositions.push_back(vertex.position);
        }

        glm::vec3 localCenter{ 0.0f };
        glm::vec3 localHalfExtents{ 0.5f };
        if (!verticesPositions.empty()) {
            glm::vec3 minimum = verticesPositions.front();
            glm::vec3 maximum = verticesPositions.front();
            for (const glm::vec3& vertex : verticesPositions) {
                minimum = glm::min(minimum, vertex);
                maximum = glm::max(maximum, vertex);
            }
            localCenter = (minimum + maximum) * 0.5f;
            localHalfExtents = (maximum - minimum) * 0.5f;
        }

        ColliderDesc colliderDesc;
        colliderDesc.localPose.position = partTransform->position;
        colliderDesc.localPose.orientation = partTransform->orientation;
        colliderDesc.localScale = partTransform->scale;

        if (partDesc.colliderType == ColliderType::CUBOID) {
            colliderDesc.shape = BoxShapeDesc{ localCenter, localHalfExtents };
        }
        else if (partDesc.colliderType == ColliderType::SPHERE) {
            float radius = 0.5f;
            if (!verticesPositions.empty()) {
                radius = 0.0f;
                for (const glm::vec3& vertex : verticesPositions) {
                    radius = (std::max)(radius, glm::length(vertex - localCenter));
                }
            }
            colliderDesc.shape = SphereShapeDesc{ localCenter, radius };
        }

        ColliderHandle colliderHandle = physicsEngine.createCollider(bodyHandle, colliderDesc);
        if (!colliderHandle.isValid()) {
            std::cerr << "[World] Failed to create collider for part '"
                << partDesc.name << "'.\n";
            continue;
        }

        part.colliderHandle = colliderHandle;
        if (Collider* collider = getCollider(colliderHandle)) {
            collider->localTransformHandle = partDesc.localTransformHandle;
        }

        gameObject.parts.push_back(part);
    }

    if (!body || body->colliderHandles.empty()) {
        std::cerr << "[World] Warning: Created GameObject with no colliders. GameObject ID: " << gameObject.id << "\n";
    }

    renderer.addObjectToBatch(gameObjectHandle);

    objectId++;
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

    // #TODO: mark physics objects as pending deletion so they wont be used in physics
    // so they wont mess up parallelization of the physics step loop
}
