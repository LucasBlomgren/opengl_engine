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

    GameObjectHandle handle = m_gameObjects.create(objectId, shader, mesh, pos, size, colliderType, mass, isStatic, textureId, orientation, sleepCounterThreshold, asleep, color, seeThrough);
   
    BroadphaseBucket targetBucket = asleep ? BroadphaseBucket::Asleep : (isStatic ? BroadphaseBucket::Static : BroadphaseBucket::Awake);
    m_physicsEngine.queueAdd(handle, targetBucket);

    if (!seeThrough) {
        m_renderer.addObjectToBatch(handle);
    }

    PhysicsWorld* world = m_physicsEngine.getPhysicsWorld();
    world->createRigidBody();
    world->createCollider();

    objectId++;
    return handle;
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