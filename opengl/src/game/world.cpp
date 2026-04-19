#include "pch.h"
#include "world.h"

#include "renderer/renderer.h"
#include "textures/texture_manager.h"
#include "mesh/mesh_manager.h"
#include "shaders/shader_manager.h"
#include "lighting/light_manager.h"
#include "physics.h"

void World::clear() {
    gameObjects = SlotMap<GameObject, GameObjectHandle>();
    objectId = 0;
}

//----------------------------------
//  Getters
//----------------------------------
GameObject* World::getGameObject(const GameObjectHandle& h) {
    return gameObjects.try_get(h);
}
Transform* World::getTransform(const TransformHandle& h) {
    return transforms.try_get(h);
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
    GameObjectHandle gameObjectHandle = gameObjects.create(objectId, objDesc.rootTransformHandle);
    GameObject& gameObject = *gameObjects.try_get(gameObjectHandle);
    gameObject.name = objDesc.name;

    PhysicsWorld* physicsWorld = physicsEngine.getPhysicsWorld();
    Transform* rootTransform = transforms.try_get(objDesc.rootTransformHandle);

    // create rigid body
    RigidBodyHandle bodyHandle = physicsWorld->createRigidBody();
    gameObject.rigidBodyHandle = bodyHandle;

    RigidBody& body = *physicsWorld->getRigidBodiesMap().try_get(bodyHandle);
    body.gameObjectHandle = gameObjectHandle;
    body.rootTransformHandle = objDesc.rootTransformHandle;
    body.type = objDesc.bodyType;
    body.mass = objDesc.mass;

    if (objDesc.bodyType == BodyType::Static || objDesc.bodyType == BodyType::Kinematic) {
        body.invMass = 0.0f;
    } else {
        body.invMass = 1.0f / objDesc.mass;
    }

    body.asleep = objDesc.asleep;
    body.sleepCounterThreshold = objDesc.sleepCounterThreshold;
    body.anchorPoint = rootTransform->position;
    body.invRadius = 1.0f / (0.5f * glm::length(rootTransform->scale));

    // broadphase bucket
    BroadphaseBucket target;
    switch (objDesc.bodyType) {
    case BodyType::Dynamic:
        target = objDesc.asleep ? BroadphaseBucket::Asleep : BroadphaseBucket::Awake;
        break;
    case BodyType::Kinematic:
        target = BroadphaseBucket::Awake;
        break;
    case BodyType::Static:
        target = BroadphaseBucket::Static;
        break;
    }
    body.broadphaseHandle.bucket = target;

    // create parts
    for (const SubPartDesc& partDesc : objDesc.parts) {
        Transform* partTransform = transforms.try_get(partDesc.localTransformHandle);
        SubPart part;
        part.name = partDesc.name;
        part.localTransformHandle = partDesc.localTransformHandle;
        part.shader = shaderManager.getShader(partDesc.shaderName);
        part.mesh = meshManager.getMesh(partDesc.meshName);

        if (partDesc.textureName == "plain") {
            part.textureId = 999;
        } else {
            part.textureId = textureManager.getTexture(partDesc.textureName);
        }

        part.color = partDesc.color;
        part.seeThrough = partDesc.seeThrough;

        // create collider for part
        ColliderHandle colliderHandle = physicsWorld->createCollider();
        Collider& collider = *physicsWorld->getCollidersMap().try_get(colliderHandle);
        collider.rigidBodyHandle = bodyHandle;
        collider.type = partDesc.colliderType;
        collider.localTransformHandle = partDesc.localTransformHandle;

        collider.pose.position = partTransform->position;
        collider.pose.orientation = partTransform->orientation;
        collider.pose.scale = partTransform->scale;
        collider.pose.combineIntoColliderPose(*rootTransform, *partTransform);
        collider.pose.ensureModelMatrix();

        // aabb init
        std::vector<glm::vec3> verticesPositions;
        for (const Vertex& vertex : part.mesh->vertices) {
            verticesPositions.push_back(vertex.position);
        }
        collider.aabb.init(verticesPositions);
        collider.aabb.update(collider.pose);

        // collider shape init
        if (partDesc.colliderType == ColliderType::CUBOID) {
            OOBB box(verticesPositions, collider.pose);
            collider.shape = box;
        }
        else if (partDesc.colliderType == ColliderType::SPHERE) {
            Sphere sphere(collider.pose);
            collider.shape = sphere;
        }

        part.colliderHandle = colliderHandle;

        gameObject.parts.push_back(part);
        body.colliderHandles.push_back(colliderHandle);
    }

    if (body.colliderHandles.empty()) {
        std::cerr << "[World] Warning: Created GameObject with no colliders. GameObject ID: " << gameObject.id << "\n";
    }

    Collider* mainCollider = physicsWorld->getCollidersMap().try_get(body.colliderHandles[0]);
    body.aabb = mainCollider->getAABB();

    if (body.isCompound()) {
        for (size_t i = 1; i < body.colliderHandles.size(); ++i) {
            Collider* c = physicsWorld->getCollidersMap().try_get(body.colliderHandles[i]);

            // grow body AABB to include all colliders
            body.aabb.growToInclude(c->getAABB().worldMin);
            body.aabb.growToInclude(c->getAABB().worldMax);
        }

        body.aabb.worldCenter = (body.aabb.worldMin + body.aabb.worldMax) * 0.5f;
        body.aabb.worldHalfExtents = (body.aabb.worldMax - body.aabb.worldMin) * 0.5f;
        body.aabb.setSurfaceArea();
    }

    // inertia init 
    // #TODO: refactor to support multiple colliders per rigid body (compound objects)
    Collider* collider = physicsWorld->getCollidersMap().try_get(body.colliderHandles[0]);
    body.calculateInverseInertia(objDesc.parts[0].colliderType, *collider, *rootTransform);

    // add body to broadphase
    physicsEngine.queueAdd(bodyHandle, body.broadphaseHandle.bucket);


    // #TODO: should add parts instead of whole game object
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
        physicsEngine.queueRemove(obj->rigidBodyHandle);
        renderer.removeObjectFromBatch(handle);
        gameObjects.destroy(handle);
    }
    else {
        std::cerr << "[World] Warning: Tried to delete non-existing GameObject with handle (slot: " << handle.slot << ", gen: " << handle.gen << ")\n";
    }
}