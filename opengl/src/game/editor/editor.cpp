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

void Editor::update(float deltaTime) 
{
    updateSelectedObject(deltaTime);

    if (engineState->GetPressedKey() == "H") {
        sceneBuilder->createScene(*physicsEngine, *cubeVertices, *indices);
        selectedObject = nullptr;
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
        selectObject();
    }
    if (engineState->GetPressedKey() == "Mouse3") {
        GameObject& newObject = sceneBuilder->createObject(*physicsEngine, "crate", (camera->Position + camera->Front * 30.0f), glm::vec3(10), 1, 0, *cubeVertices, *indices);
        newObject.linearVelocity = camera->Front * 2500.0f;
        newObject.asleep = false;
    }

    engineState->clearPressedKey();
}

void Editor::selectObject() 
{
    // drop object if an object is selected
    if (selectedObject) {
        selectedObject->selectedByEditor = false;
        selectedObject->hasGravity = true;
        selectedObject->sleepCounterThreshold = 0.5f;
        selectedObject->sleepCounter = 0.0f;
        selectedObject = nullptr;
        return;
    }

    RaycastHit hitData = rayCast();

    // no hit return
    if (hitData.objIndex == -1) {
        return;
    }
    selectedObject = &sceneBuilder->getGameObjectList()[hitData.objIndex];
    if (selectedObject->isStatic) {
        selectedObject = nullptr;
        return;
    }

    selectedObject->selectedByEditor = true;
    selectedObject->asleep = false;
    selectedObject->hasGravity = false;
    selectedObject->sleepCounterThreshold = 1000000.0f;

    selectedObject->linearVelocity = glm::vec3(0.0f);
    selectedObject->angularVelocity = glm::vec3(0.0f);

    glm::vec3 worldOffset = selectedObject->position - camera->Position;
    // Projicera worldOffset på kamerans lokala axlar:
    selectionOffsetLocal.x = glm::dot(worldOffset, camera->Right);
    selectionOffsetLocal.y = glm::dot(worldOffset, camera->Up);
    selectionOffsetLocal.z = glm::dot(worldOffset, camera->Front);
}

void Editor::updateSelectedObject(float deltaTime) 
{
    if (!selectedObject) { return; }

    glm::vec3 worldOffset = camera->Right * selectionOffsetLocal.x + camera->Up * selectionOffsetLocal.y + camera->Front * selectionOffsetLocal.z;

    // Slutgiltig position:
    glm::vec3 newPos = camera->Position + worldOffset;
    selectedObject->linearVelocity = (newPos - selectedObject->lastPosition) / deltaTime;
}

RaycastHit Editor::rayCast()
{
    float rayLength = 200.0f;
    Physics::Ray ray(camera->Position, camera->Front, rayLength);
    RaycastHit hitData = physicsEngine->performRayCastFinite(ray);

    lastHitData = hitData;
    return hitData;
}

RaycastHit& Editor::getLastRayHit() {
    return lastHitData;
}
bool Editor::getRayCastHasHit() const {
    return rayCastHasHit;
}