#pragma once

#include <vector>

#include "input_manager.h"
#include "engine_state.h"
#include "scene_builder.h"
#include "physics.h"
#include "game_object.h"
#include "camera.h"
#include "skybox/skybox_manager.h"
#include "vertex.h"

class Editor {
public:
    void setPointers(
        GLFWwindow* window,
        InputManager* inputManager,
        EngineState* engineState,
        SceneBuilder* sceneBuilder,
        PhysicsEngine* physicsEngine,
        Camera* camera,
        SkyboxManager* skyboxManager
    );

    void cameraMode();
    void update(float& deltaTime, Shader& shader);
    RaycastHit rayCast(float length);
    RaycastHit& getLastRayHit();

    // select object
    void selectObject();
    void dropObject();
    void updateSelectedObject(float deltaTime);
    GameObject* selectedObject = nullptr;
    glm::vec3 selectionOffsetLocal{ 0.0f, 0.0f, 0.0f };

    void placeObject();
    void createPlaceObjectAABB(Shader& shader);
    void drawAABB(const AABB& aabb, Shader& shader, glm::vec3 color = { 0.9f,0.7f,0.2f });
    AABB aabbToPlace;
    bool placementObstructed = true;
    bool objectRainBlocks = false;
    bool objectRainSpheres = false;

private:
    bool drawPlacementAABB = false;
    glm::vec3 selectedObjVelocity = { 0.0f, 0.0f, 0.0f };

    GLFWwindow* window = nullptr;
    EngineState* engineState = nullptr;
    InputManager* inputManager = nullptr;
    SceneBuilder* sceneBuilder = nullptr;
    PhysicsEngine* physicsEngine = nullptr;
    SkyboxManager* skyboxManager = nullptr;
    Camera* camera = nullptr;

    RaycastHit lastHitData;

    double savedMouseX, savedMouseY;

    constexpr static glm::vec3 OBJ_PLACE_SIZE{ 1.0f, 1.0f, 1.0f };
    constexpr static bool EDITOR_RAYCAST_ENABLED = true;
};