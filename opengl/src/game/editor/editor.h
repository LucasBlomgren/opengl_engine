#pragma once

#include "engine_state.h"
#include "scene_builder.h"
#include "physics.h"
#include "game_object.h"
#include "camera.h"
#include "skybox_manager.h"
#include "vertex.h"
#include <vector>

#define objPlaceSize 1.0f, 1.0f, 1.0f

struct rayCast {
    glm::vec3 start;
    glm::vec3 end;
};

class Editor 
{
public:
    void setPointers(
        EngineState* engineState,
        SceneBuilder* sceneBuilder,
        PhysicsEngine* physicsEngine,
        Camera* camera,
        SkyboxManager* skyboxManager,
        std::vector<Vertex>* cubeVertices,
        std::vector<unsigned int>* indices
    );
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

    EngineState* engineState = nullptr;
    SceneBuilder* sceneBuilder = nullptr;
    PhysicsEngine* physicsEngine = nullptr;
    SkyboxManager* skyboxManager = nullptr;
    Camera* camera = nullptr;

    std::vector<Vertex>* cubeVertices = nullptr;
    std::vector<unsigned int>* indices = nullptr;

    RaycastHit lastHitData;
};