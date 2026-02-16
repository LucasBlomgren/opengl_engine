#pragma once

#include "core/slot_map.h"
#include "game/game_object.h"
#include "physics/colliders/collider.h"

class GameObject;
class TextureManager;
class LightManager;
class PhysicsEngine;
class ShaderManager;
class MeshManager;
class Renderer;

class World {
public:
    World(
        PhysicsEngine& pe, Renderer& re, TextureManager& tm, MeshManager& mm, ShaderManager& sm) :
        m_physicsEngine(pe), m_renderer(re), m_textureManager(tm), m_meshManager(mm), m_shaderManager(sm)
    {
    }

    SlotMap<GameObject, GameObjectHandle>& getGameObjects() { return m_gameObjects; }

    GameObjectHandle createGameObject(
        const std::string& textureName,
        const std::string& meshName,
        ColliderType colliderType,
        const glm::vec3& pos,
        const glm::vec3& size,
        float mass,
        bool isStatic,
        const glm::quat& orientation = glm::quat(1, 0, 0, 0),
        float sleepCounterThreshold = 1.0f,
        bool asleep = 0,
        const glm::vec3& color = glm::vec3(255.0f, 255.0f, 255.0f),
        const bool seeThrough = false
    );

    void removeGameObject(GameObjectHandle handle);

private:
    int objectId = 0;
    SlotMap<GameObject, GameObjectHandle> m_gameObjects;

    PhysicsEngine& m_physicsEngine;
    Renderer& m_renderer;
    TextureManager& m_textureManager;
    MeshManager& m_meshManager;
    ShaderManager& m_shaderManager;
};