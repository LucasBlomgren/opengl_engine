#include "editor.h"

void Editor::setPointers(
    EngineState& engineState,
    SceneBuilder& sceneBuilder,
    PhysicsEngine& physicsEngine,
    Camera& camera,
    std::vector<Vertex>& cubeVertices,
    std::vector<unsigned int>& indices) 
{
        this->engineState = &engineState;
        this->sceneBuilder = &sceneBuilder;
        this->physicsEngine = &physicsEngine;
        this->camera = &camera;
        this->cubeVertices = &cubeVertices;
        this->indices = &indices;
}

void Editor::update() {
    if (engineState->GetPressedKey() == "H") {
        sceneBuilder->createScene(*physicsEngine, *cubeVertices, *indices);
    }
    if (engineState->GetPressedKey() == "5") {
        sceneBuilder->toggleDayTime();
    }

    if (engineState->GetPressedKey() == "Mouse1") {
        GameObject& newObject = sceneBuilder->createObject(*physicsEngine, "crate", (camera->Position + camera->Front * 30.0f), glm::vec3(10, 10, 10), 1, 0, *cubeVertices, *indices);
        newObject.linearVelocity = camera->Front * 0.0f;
    }
    if (engineState->GetPressedKey() == "Mouse2") {
        GameObject& newObject = sceneBuilder->createObject(*physicsEngine, "crate", (camera->Position + camera->Front * 30.0f), glm::vec3(5, 5, 5), 0.5, 0, *cubeVertices, *indices);
        newObject.linearVelocity = camera->Front * 300.0f;
    }
    if (engineState->GetPressedKey() == "Mouse3") {
        GameObject& newObject = sceneBuilder->createObject(*physicsEngine, "crate", (camera->Position + camera->Front * 30.0f), glm::vec3(10, 10, 10), 1, 0, *cubeVertices, *indices);
        newObject.linearVelocity = camera->Front * 2500.0f;
    }

    if (engineState->GetPressedKey() == "K") {
        performRaycastPlacement();
    }

    engineState->clearPressedKey();
}

void Editor::performRaycastPlacement()
{
    std::vector<Edge>* edges = physicsEngine->getSortedEdges();
    if (!edges) {
        return;
    }

    float rayLength = 100.0f;

    std::pair<Edge, Edge> edgePair;
    createRayAABB(rayLength, camera->Position, camera->Front, physicsEngine->getSelectedAxis(), edgePair);
    this->ray = { camera->Position, camera->Position + camera->Front * rayLength };

    std::vector<Edge> edgesCopy = *edges;
    std::vector<int> collisionCandidates;
    rayBroadphase(collisionCandidates, edges, edgesCopy, edgePair);
}