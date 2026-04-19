#pragma once

#include "core/slot_map.h"
#include "game/game_object.h"
#include "physics/colliders/collider.h"
#include "rigid_body.h"

class TextureManager;
class LightManager;
class PhysicsEngine;
class ShaderManager;
class MeshManager;
class Renderer;

struct SubPartDesc {
    std::string name = "SubPart";
    TransformHandle localTransformHandle;
    ColliderType colliderType = ColliderType::CUBOID;

    // render
    std::string shaderName = "default";
    std::string textureName = "plain";
    std::string meshName = "cube";
    glm::vec3 color = glm::vec3(255.0f);
    bool seeThrough = false;
};

struct GameObjectDesc {
    std::string name = "GameObject";
    std::vector<SubPartDesc> parts;
    TransformHandle rootTransformHandle;

    // rigid body
    BodyType bodyType = BodyType::Dynamic;
    float mass = 1.0f;
    float sleepCounterThreshold = 1.5f;
    bool asleep = false;
};

class World {
public:
    World(
        PhysicsEngine& pe, Renderer& re, TextureManager& tm, MeshManager& mm, ShaderManager& sm) :
        physicsEngine(pe), renderer(re), textureManager(tm), meshManager(mm), shaderManager(sm)
    {}

    void clear();

    SlotMap<GameObject, GameObjectHandle>& getGameObjectsMap() { return gameObjects; }
    SlotMap<Transform, TransformHandle>& getTransformsMap() { return transforms; }
    GameObject* getGameObject(const GameObjectHandle& handle);
    Transform* getTransform(const TransformHandle& handle); 
    RigidBody* getRigidBody(const GameObjectHandle& handle);
    RigidBody* getRigidBody(const RigidBodyHandle& handle);
    Collider* getCollider(const ColliderHandle& handle);

    GameObjectHandle createGameObject(GameObjectDesc& obj);
    TransformHandle createTransform(
        const glm::vec3& position = glm::vec3{ 0.0f },
        const glm::quat& orientation = glm::quat{ 1.0f, 0.0f, 0.0f, 0.0f },
        const glm::vec3& scale = glm::vec3{ 1.0f }
    );

    void deleteGameObject(GameObjectHandle handle);

private:
    int objectId = 0;
    SlotMap<GameObject, GameObjectHandle> gameObjects;
    SlotMap<Transform, TransformHandle> transforms;

    PhysicsEngine& physicsEngine;
    Renderer& renderer;
    TextureManager& textureManager;
    MeshManager& meshManager;
    ShaderManager& shaderManager;
};