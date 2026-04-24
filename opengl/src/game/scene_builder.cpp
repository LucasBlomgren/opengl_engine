#include "pch.h"
#include "scene_builder.h"

#include "geometry/vertex.h"
#include "renderer/renderer.h"
#include "lighting/light_manager.h"
#include "physics.h"

SceneBuilder::TerrainData& SceneBuilder::getTerrainData() {
    return terrainData;
}

float SceneBuilder::randomRange(float start, float end) {
   if (start > end or start == end) {
      std::cerr << "Invalid range: " << start << " to " << end << std::endl;
      return -1; 
   }

   std::uniform_real_distribution<float> dist(start, end);
   return dist(this->rng);
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
    lightManager.clearLights();
    lightManager.clearDirectionalLight();

    if (lightsState == 0) {
        // sun light
        if (dayNightCycle == 0) {
            lightManager.setDirectionalLight(glm::vec3(0.45f, -0.8f, 0.9f), glm::vec3(0.3f), glm::vec3(0.7f), glm::vec3(0.5f)); 
        }
        else {
            lightManager.setDirectionalLight(glm::vec3(0.45f, -0.8f, 0.9f), glm::vec3(0.3f), glm::vec3(0.0f), glm::vec3(0.0f));
        }
    }
    else if (lightsState == 1) {
        // red light
        Light light(glm::vec3(125, 35, 250), glm::vec3(0.5f, 0.2f, 0.5f), glm::vec3(1.0, 0.0, 0.0), 25.0f);
        lightManager.addLight(light);
        // green
        Light light2(glm::vec3(125, 35, 120), glm::vec3(2, 0.2f, 2), glm::vec3(0.0, 1.0, 0.0), 25.0f);
        lightManager.addLight(light2);
        // blue light
        Light light3(glm::vec3(125, 35, 0), glm::vec3(2, 0.2f, 2), glm::vec3(0.0, 0.0, 1.0), 25.0f);
        lightManager.addLight(light3);
    }
    else if (lightsState == 2) {
        // create no lights
    }
}

//----------------------------------
//         Scene Creation
//----------------------------------
void SceneBuilder::createScene(int sceneID, bool isPlayerMode)
{
    if (isPlayerMode) {
        player.resetState();
    } else {
        editor.resetState();
    }

    sceneDirty = true;
    world.clear();
    physicsEngine.clear();
    renderer.clearRenderBatches();

    terrainData.triangles.clear(); 
    terrainData.triangles.reserve(5000000);
    terrainData.vertices.clear(); 
    terrainData.vertices.reserve(5000000);
    terrainData.indices.clear();
    terrainData.indices.reserve(5000000); 

    setLights();

    switch (sceneID) {
    case 0: testFloorScene(); break;
    case 1: emptyFloorScene(); break;
    case 2: tallStructureScene(); break;
    case 3: mainScene(); break;
    case 5: tallStructureScene(); break;
    case 6: castleScene(); break;
    case 7: containerScene(); break;
    case 8: shapePileScene(); break;
    default: break;
    }

    physicsEngine.setupScene(&terrainData.triangles);

    glcount::print();
}

//----------------------------------
//           Heightmap
//----------------------------------
void SceneBuilder::generateFlatTerrain(
    glm::vec3 offset,
    int   gridSizeX,
    int   gridSizeZ,
    float cellSize,
    float maxHeight)
{
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

    int cutOff = 4;

    // indices
    terrainData.indices.reserve(gridSizeX * gridSizeZ * 6);
    for (int z = cutOff; z < gridSizeZ - cutOff; ++z) {
        for (int x = cutOff; x < gridSizeX - cutOff; ++x) {
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

        glm::vec3 n = faceNormals.back();
        if (n.y < 0) {
            std::cout << "Inverted normal detected at triangle " << (i / 3) << ": " << n.x << ", " << n.y << ", " << n.z << std::endl;
        }
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
    for (int i = 0; i < points.size(); ++i) {
        glm::vec3 pos = points[i];
        glm::vec3 normal = vertexNormals[i];

        int x = i % (gridSizeX + 1);
        int z = i / (gridSizeX + 1);
        glm::vec2 uv{ float(x) / gridSizeX, 1.0f - float(z) / gridSizeZ };

        vertices.emplace_back(pos, normal, uv);
    }
}

void SceneBuilder::smoothHeightMap(std::vector<std::vector<float>>& H, float smoothness, int passes) {
    int W = static_cast<int>(H.size());
    int D = static_cast<int>(H[0].size());
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
void SceneBuilder::objectRain(float& current_time, glm::vec3& pos, int mode) {
    constexpr float interval = 1.0f / 15.0f;
    if (current_time - lastTime < interval)
        return;

    lastTime = current_time;

    for (int i = 0; i < 10; i++)
    {
        // position
        glm::vec3& spawnPoint = pos;
        // constexpr glm::vec3 spawnPoint = glm::vec3(25, 275, 25);
        float varianceRange = 20.0f;
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
            xVariance = randomRange(2.0, 5.0);
            yVariance = randomRange(2.0, 5.0);
            zVariance = randomRange(2.0, 5.0);
            glm::vec3 size{ xVariance, yVariance, zVariance };
            float mass = xVariance * yVariance * zVariance;
            //world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Dynamic, spawnPos, size, mass, orientation, 2.0f, false, color, false);
        }
        // spheres
        else if (mode == 1) {
            glm::vec3 size{ 2.0f };
            float mass = (size.x * 3.0f) / 2.0f;

            GameObjectDesc sphere;
            sphere.name = "sphere";
            sphere.rootTransformHandle = world.createTransform(spawnPos, orientation, size);
            sphere.mass = mass;

            SubPartDesc part;
            part.localTransformHandle = world.createTransform();
            part.colliderType = ColliderType::SPHERE;
            part.meshName = "sphere";
            part.textureName = "plain";
            part.color = color / 255.0f;
            sphere.parts.push_back(part);
            world.createGameObject(sphere);
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

                //world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Dynamic, glm::vec3(xPos, yPos, zPos), glm::vec3(sWidth, sHeight, sLength), sWeight, glm::quat(1, 0, 0, 0), 1.5f, asleep, color, false);
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

                //world.createGameObject("plain", "sphere", ColliderType::SPHERE, BodyType::Dynamic, glm::vec3(xPos, yPos, zPos), glm::vec3(sRadius), sWeight, glm::quat(1, 0, 0, 0), 1.5f, asleep, color, false);
            }
        }
        pWidthCounter -= 1;
    }
}

//----------------------------------
//         Brick Wall
//----------------------------------
void SceneBuilder::createBrickWall(
    glm::vec3 startPos,
    int wallDirection,
    float wallHeight,
    float wallWidth,
    glm::vec3 brickSize,
    float brickDistance,
    float brickWeight,
    int brickDecrease,
    glm::vec2 colorRange,
    bool fullColorRange)
{
    if (brickWeight < wallHeight) {
        brickWeight = wallHeight;
    }

    glm::vec3 edgeBrickSize = brickSize;
    if (wallDirection == 0) {
        edgeBrickSize.x = ((wallWidth * brickSize.x + (wallWidth - 1) * brickDistance) - (wallWidth - 3 * brickSize.x + (wallWidth - 2) * brickDistance)) / 2;
    }
    else {
        edgeBrickSize.z = ((wallWidth * brickSize.z + (wallWidth - 1) * brickDistance) - (wallWidth - 3 * brickSize.z + (wallWidth - 2) * brickDistance)) / 2;
    }

    // col
    for (int col = 0; col < static_cast<int>(wallHeight) / 2; col++) {
        // row 0, 2, 4, 6...
        for (int row = 0; row < static_cast<int>(wallWidth); row++) {
            glm::vec3 pos = startPos;
            pos.y += brickSize.y / 2 + col * brickSize.y * 2;
            if (wallDirection == 0) {
                pos.x = startPos.x + row * brickSize.x + brickDistance * row;
                pos.z = startPos.z;
            }
            else {
                pos.x = startPos.x;
                pos.z = startPos.z + row * brickSize.z + brickDistance * row;
            }
            glm::vec3 randomColor;
            if (fullColorRange) {
                randomColor = glm::vec3(randomRange(colorRange.x, colorRange.y), randomRange(colorRange.x, colorRange.y), randomRange(colorRange.x, colorRange.y));
            }
            else {
                float c = randomRange(colorRange.x, colorRange.y);
                randomColor = glm::vec3(c, c, c);
            }
            //world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Dynamic, pos, brickSize, brickWeight, glm::quat(1, 0, 0, 0), 1.5f, true, randomColor, false);
        }
        brickWeight -= brickDecrease;
        // row 1, 3, 5, 7...
        // edge brick
        glm::vec3 pos = startPos;
        pos.y += brickSize.y + brickSize.y / 2 + col * brickSize.y * 2;
        if (wallDirection == 0) {
            pos.x = startPos.x + (edgeBrickSize.x - brickSize.x) / 2;
            pos.z = startPos.z;
        }
        else {
            pos.x = startPos.x;
            pos.z = startPos.z + (edgeBrickSize.z - brickSize.z) / 2;
        }
        glm::vec3 randomColor;
        if (fullColorRange) {
            randomColor = glm::vec3(randomRange(colorRange.x, colorRange.y), randomRange(colorRange.x, colorRange.y), randomRange(colorRange.x, colorRange.y));
        }
        else {
            float c = randomRange(colorRange.x, colorRange.y);
            randomColor = glm::vec3(c, c, c);
        }
        //world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Dynamic, pos, edgeBrickSize, brickWeight, glm::quat(1, 0, 0, 0), 1.5f, true, randomColor, false);
        // middle bricks
        for (int row = 1; row < wallWidth - 2; row++) {
            glm::vec3 pos = startPos;
            pos.y += brickSize.y + brickSize.y / 2 + col * brickSize.y * 2;
            if (wallDirection == 0) {
                pos.x = brickSize.x / 2 + startPos.x + row * brickSize.x + brickDistance * row;
                pos.z = startPos.z;
            }
            else {
                pos.x = startPos.x;
                pos.z = brickSize.z / 2 + startPos.z + row * brickSize.z + brickDistance * row;
            }
            glm::vec3 randomColor;
            if (fullColorRange) {
                randomColor = glm::vec3(randomRange(colorRange.x, colorRange.y), randomRange(colorRange.x, colorRange.y), randomRange(colorRange.x, colorRange.y));
            }
            else {
                float c = randomRange(colorRange.x, colorRange.y);
                randomColor = glm::vec3(c, c, c);
            }
            //world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Dynamic, pos, brickSize, brickWeight, glm::quat(1, 0, 0, 0), 1.5f, true, randomColor, false);
        }
        // edge brick
        pos = startPos;
        pos.y += brickSize.y + brickSize.y / 2 + col * brickSize.y * 2;
        if (wallDirection == 0) {
            pos.x = startPos.x + wallWidth - 1 * brickSize.x + brickDistance * wallWidth - (edgeBrickSize.x - brickSize.x) / 2;
            pos.z = startPos.z;
        }
        else {
            pos.x = startPos.x;
            pos.z = startPos.z + wallWidth - 1 * brickSize.z + brickDistance * wallWidth - (edgeBrickSize.z - brickSize.z) / 2;
        }
        if (fullColorRange) {
            randomColor = glm::vec3(randomRange(colorRange.x, colorRange.y), randomRange(colorRange.x, colorRange.y), randomRange(colorRange.x, colorRange.y));
        }
        else {
            float c = randomRange(colorRange.x, colorRange.y);
            randomColor = glm::vec3(c, c, c);
        }
        //world.createGameObject("plain", "cube", ColliderType::CUBOID, BodyType::Dynamic, pos, edgeBrickSize, brickWeight, glm::quat(1, 0, 0, 0), 1.5f, true, randomColor, false);
        brickWeight -= brickDecrease;
    }
}