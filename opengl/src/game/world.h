#pragma once

#include "core/slot_map.h"
#include "game/game_object.h"
#include "physics/colliders/collider.h"

class RigidBody;
enum class BodyType;

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

    void clear();

    SlotMap<GameObject, GameObjectHandle>& getGameObjectsMap() { return m_gameObjects; }
    GameObject* getGameObject(const GameObjectHandle& handle);
    GameObject* getGameObject(const ColliderHandle& handle);
    RigidBody* getRigidBody(const GameObjectHandle& handle);
    RigidBody* getRigidBody(const RigidBodyHandle& handle);
    Collider* getCollider(const GameObjectHandle& handle);
    Collider* getCollider(const ColliderHandle& handle);

    GameObjectHandle createGameObject(
        const std::string& textureName,
        const std::string& meshName,
        ColliderType colliderType,
        BodyType bodyType,
        const glm::vec3& pos,
        const glm::vec3& size,
        float mass,
        const glm::quat& orientation,
        float sleepCounterThreshold,
        bool asleep,
        const glm::vec3& color,
        const bool seeThrough
    );

    void deleteGameObject(GameObjectHandle handle);

private:
    int objectId = 0;
    SlotMap<GameObject, GameObjectHandle> m_gameObjects;

    PhysicsEngine& m_physicsEngine;
    Renderer& m_renderer;
    TextureManager& m_textureManager;
    MeshManager& m_meshManager;
    ShaderManager& m_shaderManager;
};