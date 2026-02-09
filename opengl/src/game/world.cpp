#include "pch.h"
#include "world.h"

#include "renderer/renderer.h"
#include "textures/texture_manager.h"
#include "mesh/mesh_manager.h"
#include "shaders/shader_manager.h"
#include "lighting/light_manager.h"
#include "physics.h"

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
    unsigned int textureId;
    if (textureName == "plain") {
        textureId = 999;
    }
    else {
        textureId = m_textureManager.getTexture(textureName);
    }

    Mesh* mesh = m_meshManager.getMesh(meshName);

    GameObjectHandle handle = m_gameObjects.create(objectId, mesh, pos, size, colliderType, mass, isStatic, textureId, orientation, sleepCounterThreshold, asleep, color, seeThrough);

    GameObject& newObject = m_gameObjects.dense().back();
    //newObject.dynamicObjectIdx = static_cast<int>(m_gameObjects.dense().size()) - 1;
    newObject.handle = handle;
    newObject.shader = m_shaderManager.getShader("default");

    //m_physicsEngine.queueAdd(handle);

    if (!seeThrough) {
        m_renderer.addObjectToBatch(&newObject);
    }

    objectId++;
    return handle;
}

void World::removeGameObject(GameObjectHandle handle) {
    GameObject* obj = m_gameObjects.try_get(handle);
    if (obj) {
        m_physicsEngine.queueRemove(obj);
        if (!obj->seeThrough) {
            m_renderer.removeObjectFromBatch(obj);
        }
        m_gameObjects.destroy(handle);
    }
}