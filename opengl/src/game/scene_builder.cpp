#include "scene_builder.h"

std::vector<GameObject>& SceneBuilder::getDynamicObjects() {
    return dynamicObjects;
}

std::vector<Tri>& SceneBuilder::getTerrainTriangles() {
    return terrainTriangles;
}

void SceneBuilder::setPointers(TextureManager* tm, LightManager* lm, std::mt19937& rng) {
    this->textureManager = tm;
    this->lightManager = lm;
    this->rng = &rng;
}

int SceneBuilder::randomRange(int start, int end) {
   if (start > end or start == end) {
      std::cerr << "Invalid range: " << start << " to " << end << std::endl;
      return -1; 
   }

   std::uniform_int_distribution<int> dist(start, end);
   return dist(*this->rng);
}

void SceneBuilder::toggleLightsState() {
    lightsState++;
    if (lightsState > 2)
        lightsState = 0;

    setLights();
}

void SceneBuilder::setLights() {
    lightManager->clearLights();
    lightManager->clearDirectionalLight();

    if (lightsState == 0) {
        // sun light
        lightManager->setDirectionalLight(glm::vec3(0.8f, -1.0f, 0.4f), glm::vec3(0.1f), glm::vec3(1.0), glm::vec3(0.5));
    }
    else if (lightsState == 1) {
        // red light
        Light light(glm::vec3(35, 16, 32), glm::vec3(0.5f, 0.2f, 0.5f), glm::vec3(1.0, 0.0, 0.0), 6.0f);
        lightManager->addLight(light);
        // green
        Light light2(glm::vec3(15, 22, 20), glm::vec3(2, 0.2f, 2), glm::vec3(0.0, 1.0, 0.0), 7.5f);
        lightManager->addLight(light2);
        // blue light
        Light light3(glm::vec3(105, 22, 100), glm::vec3(2, 0.2f, 2), glm::vec3(0.0, 0.0, 1.0), 10.0f);
        lightManager->addLight(light3);
    }
    else if (lightsState == 2) {
        // create no lights
    }
}

void SceneBuilder::createScene(PhysicsEngine& physicsEngine)
{
    bool onlyFloor = 1;
    bool onlyFalling = 0;

    // reset scene        
    objectId = 0;

    dynamicObjects.clear();
    dynamicObjects.shrink_to_fit();
    dynamicObjects.reserve(10000);
    //staticObjects.clear();
    //staticObjects.reserve(10000);
    terrainTriangles.clear();
    terrainTriangles.reserve(10000);

    physicsEngine.clearPhysicsData();
    setLights();

    haloA.clear();
    haloB.clear();
    haloC.clear();

    //testScene();
    mainScene();

    generateFlatTerrain(
        /*gridX=*/100,
        /*gridZ=*/100,
        /*cellSize=*/5.0f,
        /*maxHeight=*/5.0f,
        /*flatness=*/1.0f);

    std::cout << terrainTriangles.size() << " terrain triangles created." << std::endl;

    physicsEngine.setupScene(&dynamicObjects, &terrainTriangles);
}

GameObject& SceneBuilder::createObject(
    const std::string& textureName,
    ColliderType colliderType,
    glm::vec3 pos,
    glm::vec3 size,
    float mass,
    bool isStatic,
    glm::quat orientation,
    float sleepCounterThreshold,
    bool asleep,
    glm::vec3 color
)
{
    unsigned int textureID;
    if (textureName == "plain") 
        textureID = 999;
    else 
        textureID = textureManager->getTexture(textureName);

    dynamicObjects.emplace_back(objectId, cubeVertices, indices, pos, size, colliderType, mass, isStatic, textureID, orientation, sleepCounterThreshold, asleep, color);

    objectId++;
    return dynamicObjects.back();
}

void SceneBuilder::generateFlatTerrain(
    int   gridSizeX,
    int   gridSizeZ,
    float cellSize,
    float maxHeight,
    float flatness /* i [0,1], 0 = helt plant, 1 = full variation */
) {
    // 1) Skapa en hödkarta: slumpa eller perlin-noise-baserad, här enkel rand
    std::mt19937                             rng{ std::random_device{}() };
    std::uniform_real_distribution<float>    distH(0.0f, maxHeight * flatness);

    // 2) Fyll en (gridSizeX+1)x(gridSizeZ+1)-array med höjder
    std::vector<std::vector<float>> heightMap(gridSizeX + 1, std::vector<float>(gridSizeZ + 1));

    for (int x = 0; x <= gridSizeX; ++x) {
        for (int z = 0; z <= gridSizeZ; ++z) {
            heightMap[x][z] = distH(rng);
        }
    }

    glm::vec3 offset{ -600.0f, 0.0f, 0.0f };
    // 3) Bygg trianglar: två per cell
    for (int x = 0; x < gridSizeX; ++x) {
        for (int z = 0; z < gridSizeZ; ++z) {
            // hörnen i cellen
            glm::vec3 v00 = offset + glm::vec3{ x * cellSize, heightMap[x][z],     z * cellSize };
            glm::vec3 v10 = offset + glm::vec3{ (x + 1) * cellSize, heightMap[x + 1][z],   z * cellSize }; 
            glm::vec3 v01 = offset + glm::vec3{ x * cellSize, heightMap[x][z + 1],   (z + 1) * cellSize };
            glm::vec3 v11 = offset + glm::vec3{ (x + 1) * cellSize, heightMap[x + 1][z + 1], (z + 1) * cellSize }; 

            // dela cellen diagonalt: diagonal v00→v11
            // triangel 1: v00, v10, v11
            terrainTriangles.emplace_back(v00, v10, v11);
            // triangel 2: v00, v11, v01
            terrainTriangles.emplace_back(v00, v11, v01);
        }
    }
}

void SceneBuilder::objectRain(float& current_time, std::mt19937& rng) {
    constexpr float interval = 1.0f / 1000.0f;
    if (current_time - lastTime < interval)
        return;

    lastTime = current_time;

    // position
    constexpr glm::vec3 spawnPoint = glm::vec3(1250, 650, 1250);
    float varianceRange = 1000.0f;
    float xVariance = randomRange(-varianceRange, varianceRange);
    float yVariance = randomRange(-25, 25);
    float zVariance = randomRange(-varianceRange, varianceRange);
    glm::vec3 color = glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255));
    glm::vec3 spawnPos = spawnPoint + glm::vec3(xVariance, yVariance, zVariance);

    // orientation
    float randomAng = randomRange(0, 360);
    glm::vec3 randomAxis = glm::vec3(randomRange(-1, 1), randomRange(-1, 1), randomRange(-1, 1));
    glm::quat orientation = glm::angleAxis(glm::radians(randomAng), randomAxis);

    createObject("plain", ColliderType::CUBOID, spawnPos, glm::vec3(10.0f), 1, 0, orientation, 2.0f, 0, color);
}