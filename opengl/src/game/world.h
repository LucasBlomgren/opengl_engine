#pragma once

#include "core/slot_map.h"
#include "game/game_object.h"
#include "game/transform.h"
#include "physics/physics.h"

#include <unordered_map>

class TextureManager;
class LightManager;
namespace physics { class Engine; }
class ShaderManager;
class MeshManager;
class Renderer;

struct SubPartDesc {
    std::string name = "SubPart";
    TransformHandle localTransformHandle;
    physics::ColliderType colliderType = physics::ColliderType::CUBOID;

    // render
    std::string shaderName = "default";
    std::string textureName = "plain";
    std::string meshName = "cube";
    glm::vec3 color = glm::vec3(1.0f); // white by default
    bool seeThrough = false;
};

struct GameObjectDesc {
    std::string name = "GameObject";
    std::vector<SubPartDesc> parts;
    TransformHandle rootTransformHandle;

    // rigid body
    physics::BodyType bodyType = physics::BodyType::Dynamic;
    float mass = 1.0f;
    float sleepCounterThreshold = 1.5f;
    bool asleep = false;
    bool allowSleep = true;
    bool allowGravity = true;
    bool canMoveLinearly = true;
    physics::MotionControl motionControl = physics::MotionControl::Physics;
    physics::ResponseMode responseMode = physics::ResponseMode::Normal;
};

class World {
public:
    World(
        physics::Engine& pe, Renderer& re, TextureManager& tm, MeshManager& mm, ShaderManager& sm) :
        physicsEngine(pe), renderer(re), textureManager(tm), meshManager(mm), shaderManager(sm)
    {}

    void clear();

    SlotMap<GameObject, GameObjectHandle>& getGameObjectsMap() { return gameObjects; }
    SlotMap<Transform, TransformHandle>& getTransformsMap() { return transforms; }
    GameObject* getGameObject(const GameObjectHandle& handle);
    Transform* getTransform(const TransformHandle& handle); 
    GameObjectHandle getGameObjectHandle(physics::BodyHandle body) const;

    GameObjectHandle createGameObject(GameObjectDesc& obj);
    TransformHandle createTransform(
        const glm::vec3& position = glm::vec3{ 0.0f },
        const glm::quat& orientation = glm::quat{ 1.0f, 0.0f, 0.0f, 0.0f },
        const glm::vec3& scale = glm::vec3{ 1.0f }
    );

    void deleteGameObject(GameObjectHandle handle);
    void syncTransformsToPhysics();
    void syncPhysicsToTransforms();
    void syncGameObjectTransformToPhysics(GameObjectHandle handle);

private:
    int objectId = 0;
    SlotMap<GameObject, GameObjectHandle> gameObjects;
    SlotMap<Transform, TransformHandle> transforms;
    std::unordered_map<physics::BodyHandle, GameObjectHandle>
        bodyToGameObject;

    physics::Engine& physicsEngine;
    Renderer& renderer;
    TextureManager& textureManager;
    MeshManager& meshManager;
    ShaderManager& shaderManager;
};
