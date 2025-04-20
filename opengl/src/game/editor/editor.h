#pragma once

#include "engine_state.h"
#include "scene_builder.h"
#include "physics.h"
#include "ray.h"
#include "game_object.h"
#include "camera.h"
#include "vertex.h"
#include <vector>

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
        std::vector<Vertex>* cubeVertices,
        std::vector<unsigned int>* indices
    );
    void update();
    RaycastHit rayCast();
    RaycastHit& getLastRayHit();
    bool getRayCastHasHit() const;

private:
    EngineState* engineState = nullptr;
    SceneBuilder* sceneBuilder = nullptr;
    PhysicsEngine* physicsEngine = nullptr;
    Camera* camera = nullptr;

    std::vector<Vertex>* cubeVertices = nullptr;
    std::vector<unsigned int>* indices = nullptr;

    RaycastHit lastHitData;
    bool rayCastHasHit = false;
};