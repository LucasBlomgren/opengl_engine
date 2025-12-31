#pragma once

#include "input.h"
#include "aabb.h"
#include "physics/raycast.h"

struct GLFWwindow;
class InputManager;
class EngineState;
class SceneBuilder;
class PhysicsEngine;
class Camera;
class SkyboxManager;
class Shader;

class Editor : public IInputReceiver {
public:
    void addInputRouter(InputRouter& router);
    void handleInput(const InputFrame& in, const InputContext& ctx, Consumed& consumed, FrameWants& wants);

    void setPointers(SceneBuilder* sceneBuilder, PhysicsEngine* physicsEngine, Camera* camera);

    // activate/deactivate editor mode
    void activate();
    void deactivate();

    // update
    void fixedUpdate(float fixedTimeStep);
    void update(Shader& shader);

    // select object
    GameObject* selectedObject = nullptr;
    glm::vec3 selectionOffsetLocal{ 0.0f, 0.0f, 0.0f };
    void syncSelectionOffset();
    void selectObject();
    void dropObject();
    void updateSelectedObject(float fixedTimeStep);

    // raycast for placement/selection/etc
    RaycastHit rayCast(float length);
    RaycastHit& getLastRayHit();
    AABB aabbToPlace;
    bool placementObstructed = true;
    void placeObject();
    void createPlaceObjectAABB(Shader& shader);
    void drawAABB(const AABB& aabb, Shader& shader, glm::vec3 color = { 0.9f,0.7f,0.2f });

    glm::vec3 objectRainPos = { 0.0f, 100.0f, 0.0f };
    bool objectRainBlocks = false;
    bool objectRainSpheres = false;

private:
    SceneBuilder* sceneBuilder = nullptr;
    PhysicsEngine* physicsEngine = nullptr;
    Camera* camera = nullptr;

    glm::vec3 moveInput{ 0.0f, 0.0f, 0.0f };

    RaycastHit lastHitData;
    bool drawPlacementAABB = false;
    constexpr static float SELECT_RANGE = 5000.0f;
    constexpr static float OBJ_PLACE_DISTANCE = 150.0f;
    constexpr static glm::vec3 OBJ_PLACE_SIZE{ 1.0f, 1.0f, 1.0f };
};