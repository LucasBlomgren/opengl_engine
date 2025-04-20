#include "editor.h"

void Editor::setPointers(
    EngineState* engineState,
    SceneBuilder* sceneBuilder,
    PhysicsEngine* physicsEngine,
    Camera* camera,
    std::vector<Vertex>* cubeVertices,
    std::vector<unsigned int>* indices) 
{
        this->engineState = engineState;
        this->sceneBuilder = sceneBuilder;
        this->physicsEngine = physicsEngine;
        this->camera = camera;
        this->cubeVertices = cubeVertices;
        this->indices = indices;
}

void Editor::update() {
    RaycastHit hitData = rayCast();
    if (hitData.objIndex != -1) {
        rayCastHasHit = true;
        lastHitData = hitData;
    }
    else {
        rayCastHasHit = false;
    }

    if (engineState->GetPressedKey() == "H") {
        sceneBuilder->createScene(*physicsEngine, *cubeVertices, *indices);
    }
    if (engineState->GetPressedKey() == "5") {
        sceneBuilder->toggleDayTime();
    }

    if (engineState->GetPressedKey() == "Mouse1") {
        glm::vec3 spawnPos = camera->Position + camera->Front * 100.0f;
        GameObject& newObject = sceneBuilder->createObject(*physicsEngine, "crate", spawnPos, glm::vec3(10.0f), 1, 0, *cubeVertices, *indices);
        newObject.linearVelocity = camera->Front * 0.0f;
    }
    if (engineState->GetPressedKey() == "Mouse2") {
        GameObject& newObject = sceneBuilder->createObject(*physicsEngine, "crate", (camera->Position + camera->Front * 30.0f), glm::vec3(5), 0.01, 0, *cubeVertices, *indices);
        newObject.linearVelocity = camera->Front * 300.0f;
    }
    if (engineState->GetPressedKey() == "Mouse3") {
        GameObject& newObject = sceneBuilder->createObject(*physicsEngine, "crate", (camera->Position + camera->Front * 30.0f), glm::vec3(10), 1000, 0, *cubeVertices, *indices);
        newObject.linearVelocity = camera->Front * 2500.0f;
    }

    engineState->clearPressedKey();
}

RaycastHit Editor::rayCast()
{
    float rayLength = 10000.0f;
    Physics::Ray ray(camera->Position, camera->Front, rayLength);
    RaycastHit hitData = physicsEngine->performRayCastFinite(ray);

    return hitData;
}

RaycastHit& Editor::getLastRayHit() {
    return lastHitData;
}
bool Editor::getRayCastHasHit() const {
    return rayCastHasHit;
}