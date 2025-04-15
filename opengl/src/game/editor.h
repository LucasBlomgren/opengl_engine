#pragma once

#include "engine_state.h"
#include "scene_builder.h"
#include "physics.h"
#include "game_object.h"
#include "camera.h"
#include "vertex.h"
#include <vector>

class Editor 
{
public:
    Editor(EngineState& engineState, SceneBuilder& sceneBuilder, PhysicsEngine& physicsEngine, std::vector<Vertex>& cubeVertices, std::vector<unsigned int>& indices)
        : engineState(engineState),
        sceneBuilder(sceneBuilder),
        physicsEngine(physicsEngine),
        cubeVertices(cubeVertices),
        indices(indices)
    {}

    void update(const Camera& camera);

private:
    EngineState& engineState;
    SceneBuilder& sceneBuilder;
    PhysicsEngine& physicsEngine;
    std::vector<Vertex>& cubeVertices;
    std::vector<unsigned int>& indices;
};