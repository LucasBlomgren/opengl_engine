#include "scene_builder.h"

std::vector<GameObject>& SceneBuilder::getGameObjectList() {
    return GameObjectList;
}

void SceneBuilder::setPointers(TextureManager* tm, LightManager* lm) {
    textureManager = tm;
    lightManager = lm;
}

void SceneBuilder::toggleDayTime() {
    dayTime = !dayTime;

    if (dayTime) {
        // sun light
        lightManager->setDirectionalLight(glm::vec3(0.8f, -1.0f, 0.4f), glm::vec3(0.1), glm::vec3(1.0), glm::vec3(0.5));
        lightManager->clearLights();
    }
    else {
        // red light
        Light light(glm::vec3(350, 160, 320), glm::vec3(5, 2, 5), glm::vec3(1.0, 0.0, 0.0), 60);
        lightManager->addLight(light);

        // green
        Light light2(glm::vec3(150, 220, 200), glm::vec3(20, 2, 20), glm::vec3(0.0, 1.0, 0.0), 75);
        lightManager->addLight(light2);

        // blue light
        Light light3(glm::vec3(1050, 220, 1000), glm::vec3(20, 2, 20), glm::vec3(0.0, 0.0, 1.0), 100);
        lightManager->addLight(light3);
        
        lightManager->clearDirectionalLight();
    }
}

void SceneBuilder::createScene(PhysicsEngine& physicsEngine)
{
    objectId = 0;
    GameObjectList.clear();
    physicsEngine.clearPhysicsData();

    // ----------- floor tiles -----------
    int floorWidth = 5; 
    int floorHeight = 5;
    for (int i = 0; i < floorWidth; i++)
        for (int j = 0; j < floorHeight; j++) {
            glm::quat orientation = glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0f, 0.5f, 0.0f));
            createObject(physicsEngine, "uvmap", glm::vec3(250 + i * 500, -5, 250 + j * 500), glm::vec3(500, 10, 500), 0, 1, orientation);
            
        }

    // ----------- slanted platform -----------
    glm::quat orientation = glm::angleAxis(glm::radians(25.0f), glm::vec3(1.0f, 0.5f, 0.0f));
    createObject(physicsEngine, "crate", glm::vec3(245, 30, 100), glm::vec3(40, 2, 40), 0, 1, orientation);

    // ----------- catapult -----------
    // support
    createObject(physicsEngine,  "crate", glm::vec3(345, 15, 200), glm::vec3(5, 30, 20), 1, 1);

    // plank
    int mass = 50;
    glm::vec3 size = glm::vec3(140, 2, 5);
    createObject(physicsEngine, "crate", glm::vec3(345, 31, 200), size, mass, 0, {}, 999);

    float I_x = (1.0f / 12.0f) * mass * (size.y * size.y + size.z * size.z);
    float I_y = (1.0f / 12.0f) * mass * (size.x * size.x + size.y * size.y);
    float I_z = (1.0f / 12.0f) * mass * (size.x * size.x + size.z * size.z);
    GameObjectList.back().inverseInertia = glm::mat3(
        glm::vec3(1.0f / I_x, 0.0f, 0.0f),
        glm::vec3(0.0f, 1.0f / I_y, 0.0f),
        glm::vec3(0.0f, 0.0f, 1.0f / I_z)
    );

    // projectile
    createObject(physicsEngine, "crate", glm::vec3(412.5, 34.5, 200), glm::vec3(5, 5, 5), 5, 0, {}, 999);
    // projectile2
    createObject(physicsEngine, "crate", glm::vec3(277.5, 34.5, 200), glm::vec3(5, 5, 5), 5, 0, {}, 999);
    // counterweight
    //createObject(physicsEngine, "crate", glm::vec3(282.5, 200, 200), glm::vec3(12, 12, 12), 100, 0, cubeVertices, indices);

    // ----------- box stacks -----------
    int amountObjects = 5;
    int amountStacks = 1;
    for (int j = 0; j < amountStacks; j++)
        for (int i = 0; i < amountObjects; i++)
            createObject(physicsEngine, "crate", glm::vec3(245 + j * 10.2, 5 + (10 * i), 245), glm::vec3(10, 10, 10), 1, 0);


    // ----------- brick wall -----------
    int wallHeight = 4;
    int wallWidth = 0;
    int brickWidth = 10;
    int brickLength = 10;
    int brickHeight = 10;
    int brickDistance = 0;

    for (int row = 0; row < wallHeight; row++) {
        for (int col = 0; col < wallWidth; col++)
        {
            float x = 545;
            float y = brickHeight / 2 + row * brickHeight;
            float z = 80 + row * brickWidth / 2 + (col * (brickWidth + brickDistance / 2));

            createObject(physicsEngine, "crate", glm::vec3(x, y, z), glm::vec3(brickWidth, brickHeight, brickLength), 1, 0);
        }
        wallWidth -= 1;
    }

    // ----------- pyramid -----------
    int pyramidHeight = 8;
    int pyramidWidth = 0;
    int stoneWidth = 10;
    int stoneLength = 10;
    int stoneHeight = 10;
    int stoneDistance = 0;

    for (int y = 0; y < pyramidHeight; y++) {
        for (int x = 0; x < pyramidWidth; x++) {
            for (int z = 0; z < pyramidWidth; z++)
            {
                float xPos = 80 + x * (stoneWidth + stoneDistance) + y * (stoneWidth/2 + stoneDistance);
                float yPos = stoneHeight / 2 + y * stoneHeight;
                float zPos = 345 + z * (stoneWidth + stoneDistance) + y * (stoneWidth / 2 + stoneDistance);
                createObject(physicsEngine, "crate", glm::vec3(xPos, yPos, zPos), glm::vec3(stoneWidth, stoneHeight, stoneLength), 1, 0);
            }
        }
        pyramidWidth -= 1;
    }

    // ----------- light sources -----------
    lightManager->clearLights();
    lightManager->clearDirectionalLight();

    if (this->dayTime) {
        // sun light
        lightManager->setDirectionalLight(glm::vec3(0.8f, -1.0f, 0.4f), glm::vec3(0.1), glm::vec3(1.0), glm::vec3(0.5));
    }
    else {
        // red light
        Light light(glm::vec3(350, 160, 320), glm::vec3(5, 2, 5), glm::vec3(1.0, 0.0, 0.0), 60);
        lightManager->addLight(light);
        // green
        Light light2(glm::vec3(150, 220, 200), glm::vec3(20, 2, 20), glm::vec3(0.0, 1.0, 0.0), 75);
        lightManager->addLight(light2);
        // blue light
        Light light3(glm::vec3(1050, 220, 1000), glm::vec3(20, 2, 20), glm::vec3(0.0, 0.0, 1.0), 100);
        lightManager->addLight(light3);
    }

    // sphere
    //GameObject& sphere = createObject(physicsEngine, "crate", glm::vec3(500,500,500), glm::vec3(10), 1, 0);
    //sphere.textureID = 999;
}

GameObject& SceneBuilder::createObject
(
    PhysicsEngine& physicsEngine,
    const std::string& textureName,
    glm::vec3 pos,
    glm::vec3 size,
    float mass,
    bool isStatic,
    glm::quat orientation,
    float sleepCounterThreshold
)
{
    unsigned int textureID = textureManager->getTexture(textureName);
    GameObject object(objectId, cubeVertices, indices, pos, size, mass, isStatic, textureID, orientation, sleepCounterThreshold);

    GameObjectList.emplace_back(object);
    objectId++;

    physicsEngine.addAabbEdges(object.AABB);

    return GameObjectList.back();
}