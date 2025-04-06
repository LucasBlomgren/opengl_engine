#include "sceneBuilder.h"

void SceneBuilder::createScene(PhysicsEngine& physicsEngine, std::vector<GameObject>& GameObjectList, std::vector<Vertex>& cubeVertices, std::vector<unsigned int>& indices)
{
    objectId = 0;
    GameObjectList.clear();
    physicsEngine.clearPhysicsData();
    physicsEngine.allEdgesX.clear();
    physicsEngine.allEdgesY.clear();
    physicsEngine.allEdgesZ.clear();

    amountObjects = 5;
    amountStacks = 1;

    //lightStrength = 100.0f;
    //lightStartingPos = { 250,180,200 };
    //lightPos = lightStartingPos;
    //light = nullptr;

    // floor
    createObject(physicsEngine, GameObjectList, glm::vec3(250, -5, 250), glm::vec3(500, 10, 500), 0, 1, 2, cubeVertices, indices);

    //// creating 100 tiles of floor 
    //for (int i = 0; i < 10; i++)
    //    for (int j = 0; j < 10; j++)
    //        createObject(physicsEngine, GameObjectList, glm::vec3(250+i*500, -5, 250+j * -500), glm::vec3(500, 10, 500), 0, 1, 2, cubeVertices, indices);

    // player controlled box
    createObject(physicsEngine, GameObjectList, glm::vec3(245, 60, 100), glm::vec3(10, 10, 10), 1, 0, 1, cubeVertices, indices);

    // slanted platform
    createObject(physicsEngine, GameObjectList, glm::vec3(245, 30, 100), glm::vec3(40, 2, 40), 0, 1, 1, cubeVertices, indices);
    GameObjectList[objectId - 1].orientation = glm::angleAxis(glm::radians(25.0f), glm::vec3(1.0f, 0.5f, 0.0f));

    //---catapult---
    // support
    createObject(physicsEngine, GameObjectList, glm::vec3(345, 15, 200), glm::vec3(5, 30, 20), 1, 1, 1, cubeVertices, indices);
    // plank
    int mass = 50;
    glm::vec3 size = glm::vec3(140, 2, 5);
    createObject(physicsEngine, GameObjectList, glm::vec3(345, 31, 200), size, mass, 0, 1, cubeVertices, indices);
    float I_x = (1.0f / 12.0f) * mass * (size.y * size.y + size.z * size.z);
    float I_y = (1.0f / 12.0f) * mass * (size.x * size.x + size.y * size.y);
    float I_z = (1.0f / 12.0f) * mass * (size.x * size.x + size.z * size.z);

    GameObjectList[objectId - 1].inverseInertia = glm::mat3(
        glm::vec3(1.0f / I_x, 0.0f, 0.0f),
        glm::vec3(0.0f, 1.0f / I_y, 0.0f),
        glm::vec3(0.0f, 0.0f, 1.0f / I_z)
    );
    // projectile
    createObject(physicsEngine, GameObjectList, glm::vec3(410, 34.5, 200), glm::vec3(5, 5, 5), 1, 0, 1, cubeVertices, indices);
    // counterweight
    createObject(physicsEngine, GameObjectList, glm::vec3(282.5, 200, 200), glm::vec3(12, 12, 12), 100, 0, 1, cubeVertices, indices);

    // box stacks
    for (int j = 0; j < amountStacks; j++)
        for (int i = 0; i < amountObjects; i++)
            createObject(physicsEngine, GameObjectList, glm::vec3(245 + j * 10.2, (10 * i) + 5, 245), glm::vec3(10, 10, 10), 1, 0, 1, cubeVertices, indices);

    // light
    //lightPos = lightStartingPos;
    //light->position = lightStartingPos;
    //light->linearVelocity = glm::vec3(0, 0, 0);
}

void SceneBuilder::createObject(PhysicsEngine& physicsEngine, std::vector<GameObject>&GameObjectList, glm::vec3 pos, glm::vec3 size, float mass, bool isStatic, int textureID, std::vector<Vertex>& cubeVertices, std::vector<unsigned int>& indices)
{
    GameObject object(objectId, cubeVertices, indices, pos, size, mass, isStatic, textureID);

    GameObjectList.emplace_back(object);
    objectId++;

    physicsEngine.allEdgesX.push_back(object.AABB.Box.min.x);
    physicsEngine.allEdgesX.push_back(object.AABB.Box.max.x);
    physicsEngine.allEdgesY.push_back(object.AABB.Box.min.y);
    physicsEngine.allEdgesY.push_back(object.AABB.Box.max.y);
    physicsEngine.allEdgesZ.push_back(object.AABB.Box.min.z);
    physicsEngine.allEdgesZ.push_back(object.AABB.Box.max.z);
}