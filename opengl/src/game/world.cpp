#include "pch.h"
#include "world.h"

#include "renderer/renderer.h"
#include "textures/texture_manager.h"
#include "mesh/mesh_manager.h"
#include "shaders/shader_manager.h"
#include "lighting/light_manager.h"
#include "physics.h"

void World::clear() {
    m_gameObjects = SlotMap<GameObject, GameObjectHandle>();
    objectId = 0;
}

GameObjectHandle World::createGameObject(
    const std::string& textureName,
    const std::string& meshName,
    ColliderType colliderType,
    const glm::vec3& pos,
    const glm::vec3& size,
    float mass,
    bool isStatic,
    const glm::quat& orientation,
    float sleepCounterThreshold,
    bool asleep,
    const glm::vec3& color,
    const bool seeThrough)
{
    Shader* shader = m_shaderManager.getShader("default");
    Mesh* mesh = m_meshManager.getMesh(meshName);
    unsigned int textureId;

    if (textureName == "plain") {
        textureId = 999;
    } else {
        textureId = m_textureManager.getTexture(textureName);
    }

    Transform transform;
    transform.position = pos;
    transform.orientation = orientation;
    transform.scale = size;
    transform.updateCache();

    GameObjectHandle gameObjectHandle = m_gameObjects.create(
        objectId, 
        shader, 
        mesh, 
        textureId, 
        transform,
        color, 
        seeThrough
    );
   
    BroadphaseBucket targetBucket = asleep ? BroadphaseBucket::Asleep : (isStatic ? BroadphaseBucket::Static : BroadphaseBucket::Awake);

    // ------ rendering initialization ----------
    if (!seeThrough) {
        m_renderer.addObjectToBatch(gameObjectHandle);
    }

    // ------ physics initialization ---------
    PhysicsWorld* physicsWorld = m_physicsEngine.getPhysicsWorld();

    // temporay solution to get the inverse inertia from the game object, will be refactored when physics instead works with PhysicsWorld and Collider/RigidBody handles
    GameObject& gameObject = *m_gameObjects.try_get(gameObjectHandle);

    // create rigid body
    RigidBodyHandle bodyHandle = physicsWorld->createRigidBody();
    RigidBody& body = *physicsWorld->getRigidBodies().try_get(bodyHandle);
    body.position = pos;
    body.orientation = orientation;
    body.mass = mass;
    body.invMass = isStatic ? 0.0f : 1.0f / mass;
    body.isStatic = isStatic;
    body.asleep = asleep;
    body.sleepCounterThreshold = sleepCounterThreshold;
    body.anchorPoint = pos;
    body.calculateInverseInertia(colliderType, gameObject.transform);

    // create collider
    ColliderHandle colliderHandle = physicsWorld->createCollider();
    Collider& collider = *physicsWorld->getColliders().try_get(colliderHandle);
    collider.ownerHandle = gameObjectHandle;
    collider.rigidBodyHandle = bodyHandle;

    // aabb init
    std::vector<glm::vec3> verticesPositions;   
    for (const Vertex& vertex : mesh->vertices) {
        verticesPositions.push_back(vertex.position);
    }
    collider.aabb.Init(verticesPositions);
    collider.aabb.update(gameObject.transform);

    // collider shape init
    if (colliderType == ColliderType::CUBOID) {
        OOBB box(verticesPositions, gameObject.transform);
        collider.shape = box;
    }
    else if (colliderType == ColliderType::SPHERE) {
        Sphere sphere(gameObject.transform);
        collider.shape = sphere;
    }

    // collider broadphase bucket
    if (asleep) {
        collider.broadphaseHandle.bucket = BroadphaseBucket::Asleep;
    } else if (isStatic) {
        collider.broadphaseHandle.bucket = BroadphaseBucket::Static;
    } else {
        collider.broadphaseHandle.bucket = BroadphaseBucket::Awake;
    }

    body.colliderHandle = colliderHandle;
    gameObject.colliderHandle = colliderHandle;
    gameObject.rigidBodyHandle = bodyHandle;

    m_physicsEngine.queueAdd(colliderHandle, collider.broadphaseHandle.bucket);

    objectId++;
    return gameObjectHandle;
}

void World::deleteGameObject(GameObjectHandle handle) {
    GameObject* obj = m_gameObjects.try_get(handle);
    if (obj) {
        //m_physicsEngine.queueRemove(handle);
        if (!obj->seeThrough) {
            m_renderer.removeObjectFromBatch(handle);
        }
        m_gameObjects.destroy(handle);
    }
}