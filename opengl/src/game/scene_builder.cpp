#include "pch.h"
#include "scene_builder.h"

#include "geometry/vertex.h"
#include "geometry/geometry_loader.h"

#include "texture_manager.h"
#include "light_manager.h"
#include "physics.h"

std::vector<GameObject>& SceneBuilder::getDynamicObjects() {
    return dynamicObjects;
}

SceneBuilder::TerrainData& SceneBuilder::getTerrainData() {
    return terrainData;
}

void SceneBuilder::setPointers(PhysicsEngine* pm, TextureManager* tm, LightManager* lm, std::mt19937& rng) {
    this->physicsEngine = pm;
    this->textureManager = tm;
    this->lightManager = lm;
    this->rng = &rng;
}

float SceneBuilder::randomRange(float start, float end) {
   if (start > end or start == end) {
      std::cerr << "Invalid range: " << start << " to " << end << std::endl;
      return -1; 
   }

   std::uniform_real_distribution<float> dist(start, end);
   return dist(*this->rng);
}

void SceneBuilder::toggleLightsState() {
    lightsState++;
    if (lightsState > 2)
        lightsState = 0;

    setLights();
}
 
void SceneBuilder::toggleDayNight() { 
    dayNightCycle++;
    if (dayNightCycle > 1)
        dayNightCycle = 0;

    setLights();
}

//----------------------------------
//         Set Lights
//----------------------------------
void SceneBuilder::setLights() {
    lightManager->clearLights();
    lightManager->clearDirectionalLight();

    if (lightsState == 0) {
        // sun light
        if (dayNightCycle == 0) {
            lightManager->setDirectionalLight(glm::vec3(0.45f, -0.8f, 0.9f), glm::vec3(0.3f), glm::vec3(0.7), glm::vec3(0.5)); 
        }
        else {
            lightManager->setDirectionalLight(glm::vec3(0.45f, -0.8f, 0.9f), glm::vec3(0.3f), glm::vec3(0.0), glm::vec3(0.0));
        }
    }
    else if (lightsState == 1) {
        // red light
        Light light(glm::vec3(125, 15, 300), glm::vec3(0.5f, 0.2f, 0.5f), glm::vec3(1.0, 0.0, 0.0), 35.0f);
        lightManager->addLight(light);
        // green
        Light light2(glm::vec3(125, 15, 150), glm::vec3(2, 0.2f, 2), glm::vec3(0.0, 1.0, 0.0), 35.0f);
        lightManager->addLight(light2);
        // blue light
        Light light3(glm::vec3(125, 15, 0), glm::vec3(2, 0.2f, 2), glm::vec3(0.0, 0.0, 1.0), 35.0f);
        lightManager->addLight(light3);
    }
    else if (lightsState == 2) {
        // create no lights
    }
}

//----------------------------------
//         Scene Creation
//----------------------------------
void SceneBuilder::createScene(PhysicsEngine& physicsEngine, int sceneID)
{
    sceneDirty = true;
    physicsEngine.clearPhysicsData();

    objectId = 0;
    dynamicObjects.clear();
    dynamicObjects.reserve(20000);

    terrainData.triangles.clear(); 
    terrainData.triangles.reserve(20000);
    terrainData.vertices.clear(); 
    terrainData.vertices.reserve(20000);
    terrainData.indices.clear();
    terrainData.indices.reserve(20000); 

    setLights();

    allHalos.clear();

    if (sceneID == 2) {
        mainScene(); 
    }
    else if (sceneID == 1) {
        terrainScene(); 
    }
    else if (sceneID == 0) {
        testFloorScene(); 
    }
    else if (sceneID == 3) {
        tumblerScene(); 
    }

    physicsEngine.setupScene(&dynamicObjects, &terrainData.triangles);
}

//----------------------------------
//          Game Object
//----------------------------------
GameObject& SceneBuilder::createObject(
    const std::string& textureName,
    const ColliderType colliderType,
    const glm::vec3& pos,
    const glm::vec3& size,
    float mass,
    bool isStatic,
    const glm::quat& orientation,
    float sleepCounterThreshold,
    bool asleep,
    const glm::vec3& color
)
{
    unsigned int textureID;
    if (textureName == "plain") 
        textureID = 999;
    else 
        textureID = textureManager->getTexture(textureName);

    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    if (colliderType == ColliderType::CUBOID) {
        vertices = cubeVertices;
        indices = cubeIndices;
    } 
    else if (colliderType == ColliderType::SPHERE) {
        vertices = sphereVertices;
        indices = sphereIndices;
    }

    dynamicObjects.emplace_back(objectId, vertices, indices, pos, size, colliderType, mass, isStatic, textureID, orientation, sleepCounterThreshold, asleep, color);
    physicsEngine->queueAdd(&dynamicObjects.back());
    dynamicObjects.back().dynamicObjectIdx = static_cast<int>(dynamicObjects.size()) - 1; 

    objectId++;
    return dynamicObjects.back();
}

//----------------------------------
//           Heightmap
//----------------------------------
void SceneBuilder::generateFlatTerrain(
    glm::vec3 offset,
    int   gridSizeX,
    int   gridSizeZ,
    float cellSize,
    float maxHeight
) {
    float startHeight = maxHeight * 0.5f;  // t.ex. halvvägs upp
    float maxStep = maxHeight * 0.1f;      // hur stort steg vi tillåter 

    // Fyll en (gridSizeX+1)x(gridSizeZ+1)-array med höjder
    std::vector<std::vector<float>> heightMap(gridSizeX + 1, std::vector<float>(gridSizeZ + 1));
    for (int x = 0; x <= gridSizeX; ++x) {
        for (int z = 0; z <= gridSizeZ; ++z) {
            if (x == 0 && z == 0) {
                heightMap[0][0] = startHeight;
                continue;
            }

            float sum = 0.0f;
            int count = 0;

            // granne bakåt (i X-led)
            if (x > 0) {
                sum += heightMap[x - 1][z];
                ++count;
            }
            // granne bakåt (i Z-led)
            if (z > 0) {
                sum += heightMap[x][z - 1];
                ++count;
            }

            // basera på medelvärdet av befintliga grannar
            float base = sum / count;

            // nytt slumpsteg
            float delta = randomRange(-maxStep, +maxStep);
            float h = base + delta;

            // kläm mellan 0 och maxHeight
            h = std::max(0.0f, std::min(h, maxHeight));
            heightMap[x][z] = h;
        }
    }

    smoothHeightMap(heightMap, 0.5f, 75);

    std::vector<Tri>& triangles = terrainData.triangles;
    std::vector<Vertex>& vertices = terrainData.vertices; 
    std::vector<uint32_t>& indices = terrainData.indices;

    // points
    std::vector<glm::vec3> points;
    points.reserve((gridSizeX + 1) * (gridSizeZ + 1));
    for (int z = 0; z <= gridSizeZ; ++z) {
        for (int x = 0; x <= gridSizeX; ++x) {
            points.emplace_back(offset + glm::vec3{ x * cellSize, heightMap[x][z],     z * cellSize });
        }
    }

    // indices
    terrainData.indices.reserve(gridSizeX * gridSizeZ * 6);
    for (int z = 4; z < gridSizeZ-4; ++z) {
        for (int x = 4; x < gridSizeX-4; ++x) {
            // triangel 1:
            indices.emplace_back(x + z * (gridSizeX + 1));
            indices.emplace_back(x + (z + 1) * (gridSizeX + 1));
            indices.emplace_back((x + 1) + z * (gridSizeX + 1));

            // triangel 2:
            indices.emplace_back((x + 1) + z * (gridSizeX + 1));
            indices.emplace_back(x + (z + 1) * (gridSizeX + 1));
            indices.emplace_back((x + 1) + (z + 1) * (gridSizeX + 1));
        }
    }

    // face normals
    std::vector<glm::vec3> faceNormals;
    faceNormals.reserve(indices.size() / 3);
    for (size_t i = 0; i < indices.size(); i += 3) {
        glm::vec3 v0 = points[indices[i]];
        glm::vec3 v1 = points[indices[i + 1]]; 
        glm::vec3 v2 = points[indices[i + 2]]; 
        glm::vec3 edgeA = v1 - v0; 
        glm::vec3 edgeB = v2 - v1;
        faceNormals.emplace_back(glm::normalize(glm::cross(edgeA, edgeB)));
    }

    // vertex normals
    std::vector<glm::vec3> vertexNormals; 
    vertexNormals.resize(points.size(), glm::vec3(0.0f));
    for (size_t f = 0; f < faceNormals.size(); ++f) { 
        uint32_t i0 = indices[3 * f + 0]; 
        uint32_t i1 = indices[3 * f + 1]; 
        uint32_t i2 = indices[3 * f + 2]; 
        vertexNormals[i0] += faceNormals[f]; 
        vertexNormals[i1] += faceNormals[f]; 
        vertexNormals[i2] += faceNormals[f]; 
    }

    for (auto& n : vertexNormals) { 
        n = glm::normalize(n); 
    }

    // Tri colliders
    for (size_t i = 0; i < indices.size(); i += 3) { 
        uint32_t i0 = indices[i + 0]; 
        uint32_t i1 = indices[i + 1]; 
        uint32_t i2 = indices[i + 2]; 

        glm::vec3 v0 = points[i0]; 
        glm::vec3 v1 = points[i1]; 
        glm::vec3 v2 = points[i2]; 

        triangles.emplace_back(objectId++, v0, v1, v2);  
    }

    // Vertices
    for (size_t i = 0; i < points.size(); ++i) {
        glm::vec3 pos = points[i];
        glm::vec3 normal = vertexNormals[i];

        int x = i % (gridSizeX + 1);
        int z = i / (gridSizeX + 1);
        glm::vec2 uv{ float(x) / gridSizeX, 1.0f - float(z) / gridSizeZ };

        vertices.emplace_back(pos, normal, uv);
    }
}

void SceneBuilder::smoothHeightMap(std::vector<std::vector<float>>& H, float smoothness, int passes) {
    int W = H.size(), D = H[0].size();
    for (int pass = 0; pass < passes; ++pass) {
        auto copy = H;
        for (int x = 1; x < W - 1; ++x) {
            for (int z = 1; z < D - 1; ++z) {
                float sum = copy[x][z]
                    + copy[x - 1][z] + copy[x + 1][z]
                    + copy[x][z - 1] + copy[x][z + 1];
                float avg = sum / 5.0f;
                // weight = hur mycket vi drar mot grannarnas medel
                H[x][z] = glm::mix(copy[x][z], avg, smoothness);
            }
        }
    }
}

//----------------------------------
//         Object rain
//----------------------------------
void SceneBuilder::objectRain(float& current_time, std::mt19937& rng, int mode) {
    constexpr float interval = 1.0f / 30.0f;
    if (current_time - lastTime < interval)
        return;

    lastTime = current_time;

    for (int i = 0; i < 10; i++) 
    {
        // position
        constexpr glm::vec3 spawnPoint = glm::vec3(125, 365, 125);
        float varianceRange = 5.0f;
        float xVariance = randomRange(-varianceRange, varianceRange);
        float yVariance = randomRange(-25, 25);
        float zVariance = randomRange(-varianceRange, varianceRange);
        glm::vec3 color = glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255));
        glm::vec3 spawnPos = spawnPoint + glm::vec3(xVariance, yVariance, zVariance);

        // orientation
        float randomAng = randomRange(0, 360);
        glm::vec3 randomAxis = glm::vec3(randomRange(-1, 1), randomRange(-1, 1), randomRange(-1, 1));
        glm::quat orientation = glm::angleAxis(glm::radians(randomAng), randomAxis);

        // blocks
        if (mode == 0) {
            //xVariance = randomRange(0.5, 4);
            //yVariance = randomRange(0.5, 4);
            //zVariance = randomRange(0.5, 4);
            //glm::vec3 size{ xVariance, yVariance, zVariance };
            //float mass = xVariance * yVariance * zVariance;

            glm::vec3 size{ 4.0f };
            float mass = 2.0f;

            createObject("plain", ColliderType::CUBOID, spawnPos, size, mass, 0, orientation, 2.0f, 0, color);
        }
        // spheres
        else if (mode == 1) {
            //float variance = randomRange(0.5, 2);
            //glm::vec3 size{ variance };
            //float mass = (variance * 3.0f) / 2.0f;

            glm::vec3 size{ 2.0f };
            float mass = 2.0f;

            createObject("plain", ColliderType::SPHERE, spawnPos, size, mass, 0, orientation, 2.0f, 0, color);
        }
    }
}

//----------------------------------
//         Block Pyramid
//----------------------------------
void SceneBuilder::createBlockPyramid(
    const std::string& textureName,
    glm::vec3 color,
    const glm::vec3& pos,
    int pWidth,
    int pHeight,
    float sWidth,
    float sLength,
    float sHeight,
    float sDistance,
    float sWeight,
    bool asleep)
{
    // random color
    bool randomColor = false;
    if (color.x == -1 && color.y == -1 && color.z == -1)
        randomColor = true;

    int pWidthCounter = pWidth;
    for (int y = 0; y < pHeight; y++) {
        for (int x = 0; x < pWidthCounter; x++) {
            for (int z = 0; z < pWidthCounter; z++) {
                float xPos = pos.x + x * (sWidth + sDistance) + y * (sWidth / 2 + sDistance);
                float yPos = pos.y + sHeight / 2 + (y * sHeight);
                float zPos = pos.z + z * (sWidth + sDistance) + y * (sWidth / 2 + sDistance);

                if (randomColor) {
                    color = glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255));
                }

                createObject("plain", ColliderType::CUBOID, glm::vec3(xPos, yPos, zPos), glm::vec3(sWidth, sHeight, sLength), sWeight, 0, glm::quat(1, 0, 0, 0), 0.75f, asleep, color);
            }
        }
        pWidthCounter -= 1;
    }
}

//----------------------------------
//         Sphere Pyramid
//----------------------------------
void SceneBuilder::createSpherePyramid(
    const std::string& textureName,
    glm::vec3 color,
    const glm::vec3& pos,
    int pWidth,
    int pHeight,
    float sRadius,
    float sDistance,
    float sWeight,
    bool asleep)
{
    const float sqrt2 = sqrt(2.0f);

    // random color
    bool randomColor = false;
    if (color.x == -1 && color.y == -1 && color.z == -1)
        randomColor = true;

    float sDiameter = sRadius * 2.0f;

    int pWidthCounter = pWidth;
    for (int y = 0; y < pHeight; y++) {
        for (int x = 0; x < pWidthCounter; x++) {
            for (int z = 0; z < pWidthCounter; z++) {
                float xPos = pos.x + x * (sDiameter + sDistance) + y * (sRadius + sDistance);
                float yPos = pos.y + y * sqrt2 * sRadius; 
                float zPos = pos.z + z * (sDiameter + sDistance) + y * (sRadius + sDistance);

                if (randomColor) {
                    color = glm::vec3(randomRange(0, 255), randomRange(0, 255), randomRange(0, 255));
                }

                createObject("plain", ColliderType::SPHERE, glm::vec3(xPos, yPos, zPos), glm::vec3(sRadius), sWeight, 0, glm::quat(1, 0, 0, 0), 0.75f, asleep, color);
            }
        }
        pWidthCounter -= 1;
    }
}

//----------------------------------
//              Halo
//----------------------------------
void SceneBuilder::createHalo(
    float width,
    float height,
    float length,
    glm::vec3 baseRotation,
    glm::vec3 rotDir,
    float rotSpeed,
    glm::vec3 pos,
    int segments,
    glm::vec3 color,
    bool createsShadows
) 
{
    float baseRotAng = 90.0f;
    glm::quat baseRot = glm::angleAxis(glm::radians(baseRotAng), baseRotation);
    float halfDepth = length * 0.5f;
    glm::vec3 desiredCenter = glm::vec3(125, 0, 125);
    glm::vec3 lastPos = glm::vec3(500, 500, 500);
    float lastAngle = 45.0f;
    glm::quat lastOrient = glm::angleAxis(glm::radians(lastAngle), glm::vec3(1, 0, 0));

    float stepSize = 360.0f / segments;

    Halo halo;
    halo.rotDir = rotDir;
    halo.rotSpeed = rotSpeed;

    for (int i = 0; i < segments; ++i) {
        float newAngle = lastAngle - 5.0f;
        glm::quat newOrientLocal = glm::angleAxis(glm::radians(newAngle), glm::vec3(1, 0, 0));
        glm::vec3 frontTip1 = lastPos + lastOrient * glm::vec3(0, 0, halfDepth);
        glm::vec3 newPosLocal = frontTip1 + newOrientLocal * glm::vec3(0, 0, halfDepth);
        glm::vec3 newPos = baseRot * newPosLocal;
        glm::quat newOrient = baseRot * newOrientLocal;

        createObject("plain", ColliderType::CUBOID, newPos, glm::vec3(width, height, length), 0, 1, newOrient, 999, 0, color);
        GameObject& obj = dynamicObjects.back();
        obj.isInsideShadowFrustum = createsShadows;

        lastAngle = newAngle;
        lastOrient = newOrientLocal;
        lastPos = newPosLocal;

        halo.indices.push_back(dynamicObjects.back().id);
        halo.center = newPos;
    }

    // calculate haloCenter
    glm::vec3 sum = glm::vec3(0.0f);
    for (int i = 0; i < halo.indices.size(); i++)
        sum += dynamicObjects[halo.indices[i]].position;
    halo.center = sum / static_cast<float>(halo.indices.size());

    for (int i = 0; i < halo.indices.size(); i++) {
        GameObject& obj = dynamicObjects[halo.indices[i]];
        glm::vec3 relativePos = desiredCenter - halo.center;
        obj.position += relativePos;
    }
    halo.center = desiredCenter;

    allHalos.push_back(halo);
}