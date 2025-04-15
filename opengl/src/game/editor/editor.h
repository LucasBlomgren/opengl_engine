#pragma once

#include "engine_state.h"
#include "scene_builder.h"
#include "physics.h"
#include "game_object.h"
#include "camera.h"
#include "vertex.h"
#include "ray_place_object.h"
#include <vector>

struct rayCast {
    glm::vec3 start;
    glm::vec3 end;
};

class Editor 
{
public:
    void setPointers(
        EngineState& engineState,
        SceneBuilder& sceneBuilder,
        PhysicsEngine& physicsEngine,
        Camera& camera,
        std::vector<Vertex>& cubeVertices,
        std::vector<unsigned int>& indices
    );
    void update();
    void performRaycastPlacement();

    rayCast ray;

private:
    EngineState* engineState = nullptr;
    SceneBuilder* sceneBuilder = nullptr;
    PhysicsEngine* physicsEngine = nullptr;
    Camera* camera = nullptr;

    std::vector<Vertex>* cubeVertices = nullptr;
    std::vector<unsigned int>* indices = nullptr;
};