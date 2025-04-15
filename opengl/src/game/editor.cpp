#include "editor.h"

void Editor::update(const Camera& camera) {
    if (engineState.GetPressedKey() == "H") {
        sceneBuilder.createScene(physicsEngine, cubeVertices, indices);
    }
    if (engineState.GetPressedKey() == "5") {
        sceneBuilder.toggleDayTime();
    }

    if (engineState.GetPressedKey() == "Mouse1") {
        GameObject& newObject = sceneBuilder.createObject(physicsEngine, "crate", (camera.Position + camera.Front * 30.0f), glm::vec3(10, 10, 10), 1, 0, cubeVertices, indices);
        newObject.linearVelocity = camera.Front * 0.0f;
    }
    if (engineState.GetPressedKey() == "Mouse2") {
        GameObject& newObject = sceneBuilder.createObject(physicsEngine, "crate", (camera.Position + camera.Front * 30.0f), glm::vec3(5, 5, 5), 0.5, 0, cubeVertices, indices);
        newObject.linearVelocity = camera.Front * 300.0f;
    }
    if (engineState.GetPressedKey() == "Mouse3") {
        GameObject& newObject = sceneBuilder.createObject(physicsEngine, "crate", (camera.Position + camera.Front * 30.0f), glm::vec3(10, 10, 10), 1, 0, cubeVertices, indices);
        newObject.linearVelocity = camera.Front * 2500.0f;
    }

    engineState.clearPressedKey();
}