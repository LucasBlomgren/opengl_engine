#include "pch.h"
#include "scene_builder.h"

#include "geometry/vertex.h"
#include "graphics/renderer/renderer.h"
#include "graphics/lighting/light_manager.h"
#include "physics/physics_engine.h"

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
            lightManager.setDirectionalLight(
                glm::vec3(0.45f, -0.8f, 0.9f), 
                glm::vec3(0.3f), 
                glm::vec3(0.7f), 
                glm::vec3(0.5f)
            ); 
        }
        else {
            lightManager.setDirectionalLight(
                glm::vec3(0.45f, -0.8f, 0.9f), 
                glm::vec3(0.3f), 
                glm::vec3(0.0f), 
                glm::vec3(0.0f)
            );
        }
    }
    else if (lightsState == 1) {
        // red light
        Light light(
            glm::vec3(125, 35, 250), 
            glm::vec3(0.5f, 0.2f, 0.5f), 
            glm::vec3(1.0, 0.0, 0.0), 
            25.0f);
        lightManager.addLight(light);
        // green
        Light light2(
            glm::vec3(125, 35, 120), 
            glm::vec3(2, 0.2f, 2), 
            glm::vec3(0.0, 1.0, 0.0), 
            25.0f);
        lightManager.addLight(light2);
        // blue light
        Light light3(
            glm::vec3(125, 35, 0), 
            glm::vec3(2, 0.2f, 2), 
            glm::vec3(0.0, 0.0, 1.0), 
            25.0f);
        lightManager.addLight(light3);
    }
    else if (lightsState == 2) {
        // create no lights
    }
}

//----------------------------------
//         Scene Creation
//----------------------------------
void SceneBuilder::createScene(int sceneID, bool isPlayerMode, int amount)
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
    case 0: testFloorScene(amount); break;
    case 1: sandBox(); break;
    case 2: terrainScene(); break;
    case 3: tallStructureScene(); break;
    case 4: castleScene(); break;
    default: break;
    }

    physicsEngine.setupScene(terrainData.triangles);

    glcount::print();
}

//----------------------------------
//           Heightmap
//----------------------------------
void SceneBuilder::generateFlatTerrain(
    glm::vec3 offset,
    int gridSizeX,
    int gridSizeZ,
    float cellSize,
    float maxHeight)
{
    const float startHeight = maxHeight * 0.5f;
    const float maxStep = maxHeight * 0.1f;

    // -------------------------------------------------
    // Height map
    // -------------------------------------------------
    std::vector<std::vector<float>> heightMap(
        gridSizeX + 1,
        std::vector<float>(gridSizeZ + 1)
    );

    for (int x = 0; x <= gridSizeX; ++x) {
        for (int z = 0; z <= gridSizeZ; ++z) {
            if (x == 0 && z == 0) {
                heightMap[0][0] = startHeight;
                continue;
            }

            float sum = 0.0f;
            int count = 0;

            if (x > 0) {
                sum += heightMap[x - 1][z];
                ++count;
            }

            if (z > 0) {
                sum += heightMap[x][z - 1];
                ++count;
            }

            const float base = sum / count;
            const float delta = randomRange(-maxStep, maxStep);

            float h = base + delta;
            h = std::max(0.0f, std::min(h, maxHeight));

            heightMap[x][z] = h;
        }
    }

    smoothHeightMap(heightMap, 0.5f, 150);


    // -------------------------------------------------
    // Terrain data
    // -------------------------------------------------
    std::vector<physics::Triangle>& triangles = terrainData.triangles;
    std::vector<Vertex>& vertices = terrainData.vertices;
    std::vector<uint32_t>& indices = terrainData.indices;

    const int cutOff = 4;

    const size_t vertexCount =
        static_cast<size_t>(gridSizeX + 1) *
        static_cast<size_t>(gridSizeZ + 1);

    const int cellsX = std::max(0, gridSizeX - 2 * cutOff);
    const int cellsZ = std::max(0, gridSizeZ - 2 * cutOff);

    const size_t triangleCount =
        static_cast<size_t>(cellsX) *
        static_cast<size_t>(cellsZ) * 2;

    vertices.reserve(vertexCount);
    indices.reserve(triangleCount * 3);
    triangles.reserve(triangleCount);

    // Vertices
    for (int z = 0; z <= gridSizeZ; ++z) {
        for (int x = 0; x <= gridSizeX; ++x) {
            glm::vec3 position =
                offset +
                glm::vec3(
                    x * cellSize,
                    heightMap[x][z],
                    z * cellSize
                );

            vertices.emplace_back(
                position,
                glm::vec3(0.0f), // normals calculated later
                glm::vec2(0.0f)
            );
        }
    }

    // Indices
    for (int z = cutOff; z < gridSizeZ - cutOff; ++z) {
        for (int x = cutOff; x < gridSizeX - cutOff; ++x) {
            const uint32_t i00 =
                x + z * (gridSizeX + 1);

            const uint32_t i01 =
                x + (z + 1) * (gridSizeX + 1);

            const uint32_t i10 =
                (x + 1) + z * (gridSizeX + 1);

            const uint32_t i11 =
                (x + 1) + (z + 1) * (gridSizeX + 1);

            // Triangle 1
            indices.emplace_back(i00);
            indices.emplace_back(i01);
            indices.emplace_back(i10);

            // Triangle 2
            indices.emplace_back(i10);
            indices.emplace_back(i01);
            indices.emplace_back(i11);
        }
    }

    // Face normals + vertex normals + physics triangles
    for (size_t i = 0; i < indices.size(); i += 3) {
        const uint32_t i0 = indices[i + 0];
        const uint32_t i1 = indices[i + 1];
        const uint32_t i2 = indices[i + 2];

        const glm::vec3& v0 = vertices[i0].position;
        const glm::vec3& v1 = vertices[i1].position;
        const glm::vec3& v2 = vertices[i2].position;

        const glm::vec3 edgeA = v1 - v0;
        const glm::vec3 edgeB = v2 - v0;

        const glm::vec3 faceNormal =
            glm::normalize(glm::cross(edgeA, edgeB));

        if (faceNormal.y < 0.0f) {
            std::cout
                << "Inverted normal detected at triangle "
                << (i / 3)
                << ": "
                << faceNormal.x
                << ", "
                << faceNormal.y
                << ", "
                << faceNormal.z
                << std::endl;
        }

        vertices[i0].normal += faceNormal;
        vertices[i1].normal += faceNormal;
        vertices[i2].normal += faceNormal;

        triangles.emplace_back(
            objectId++,
            v0,
            v1,
            v2
        );
    }


    // Normalize vertex normals
    for (Vertex& vertex : vertices) {
        const float lengthSquared =
            glm::dot(vertex.normal, vertex.normal);

        if (lengthSquared > 0.0f) {
            vertex.normal = glm::normalize(vertex.normal);
        }
    }
}

void SceneBuilder::smoothHeightMap(
    std::vector<std::vector<float>>& H,
    float smoothness,
    int passes)
{
    const int W = static_cast<int>(H.size());
    const int D = static_cast<int>(H[0].size());

    std::vector<std::vector<float>> src = H;
    std::vector<std::vector<float>> dst = H;

    for (int pass = 0; pass < passes; ++pass) {
        for (int x = 1; x < W - 1; ++x) {
            for (int z = 1; z < D - 1; ++z) {
                const float sum =
                    src[x][z]
                    + src[x - 1][z]
                    + src[x + 1][z]
                    + src[x][z - 1]
                    + src[x][z + 1];

                const float avg = sum / 5.0f;

                dst[x][z] =
                    glm::mix(
                        src[x][z],
                        avg,
                        smoothness
                    );
            }
        }

        std::swap(src, dst);
    }

    H = std::move(src);
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
        float varianceRange = 20.0f;
        float xVariance = randomRange(-varianceRange, varianceRange);
        float yVariance = randomRange(-25, 25);
        float zVariance = randomRange(-varianceRange, varianceRange);
        glm::vec3 color = glm::vec3(randomRange(0, 255));
        glm::vec3 spawnPos = spawnPoint + glm::vec3(xVariance, yVariance, zVariance);

        // orientation
        float randomAng = randomRange(0, 360);
        glm::vec3 randomAxis = glm::vec3(randomRange(-1, 1));
        glm::quat orientation = 
            glm::normalize(glm::angleAxis(glm::radians(randomAng), randomAxis));

        // blocks
        if (mode == 0) {
            xVariance = randomRange(2.0, 5.0);
            yVariance = randomRange(2.0, 5.0);
            zVariance = randomRange(2.0, 5.0);
            glm::vec3 size{ xVariance, yVariance, zVariance };
            float mass = xVariance * yVariance * zVariance;

            // cube
            GameObjectDesc cube;
            cube.name = "cube";
            cube.rootTransformHandle = 
                world.createTransform(spawnPos, orientation, size);
            cube.mass = mass;
            SubPartDesc part;
            part.localTransformHandle = world.createTransform();
            part.meshName = "cube";
            part.textureName = "checker_magenta";
            part.color = color / 255.0f;
            cube.parts.push_back(part);
            world.createGameObject(cube);
            
            //// chair
            //GameObjectDesc chair;
            //chair.name = "Chair";
            //glm::quat orientation = 
            //    glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
            //glm::vec3 scale{ 2.0f };
            //chair.rootTransformHandle = 
            //    world.createTransform(spawnPos, orientation, scale);
            //chair.bodyType = physics::BodyType::Dynamic;
            //chair.mass = 2.0f;

            //glm::vec3 color = glm::vec3(
            //    static_cast<float>(rand()) / RAND_MAX,
            //    static_cast<float>(rand()) / RAND_MAX,
            //    static_cast<float>(rand()) / RAND_MAX
            //);

            //// seat
            //SubPartDesc seat;
            //seat.name = "Seat";
            //glm::vec3 positionSeat = { 0,0,0 };
            //glm::quat orientationSeat = 
            //    glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
            //glm::vec3 scaleSeat{ 1.0f, 0.2f, 1.0f };
            //seat.localTransformHandle = 
            //    world.createTransform(positionSeat, orientationSeat, scaleSeat);
            //seat.meshName = "cube";
            //seat.textureName = "checker_magenta";
            //seat.shaderName = "default";
            //seat.color = color;
            //seat.colliderType = physics::ColliderType::CUBOID;
            //chair.parts.push_back(seat);

            //// backrest
            //SubPartDesc backrest;
            //backrest.name = "Backrest";
            //glm::vec3 positionBackrest = { -0.4f, 0.6f, 0.0f };
            //glm::quat orientationBackrest = 
            //    glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
            //glm::vec3 scaleBackrest{ 0.2f, 1.0f, 1.0f };
            //backrest.localTransformHandle = 
            //    world.createTransform(
            //        positionBackrest, 
            //        orientationBackrest, 
            //        scaleBackrest
            //    );
            //backrest.meshName = "cube";
            //backrest.textureName = "checker_magenta";
            //backrest.shaderName = "default";
            //backrest.color = color;
            //backrest.colliderType = physics::ColliderType::CUBOID;
            //chair.parts.push_back(backrest);

            //// legs
            //std::array<glm::vec3, 4> legPositions{
            //    glm::vec3(-0.4f, -0.6f, -0.4f),
            //    glm::vec3(0.4f, -0.6f, -0.4f),
            //    glm::vec3(-0.4f, -0.6f, 0.4f),
            //    glm::vec3(0.4f, -0.6f, 0.4f)
            //};
            //for (int i = 0; i < 4; i++) {
            //    SubPartDesc leg;
            //    leg.name = "Leg" + std::to_string(i);
            //    glm::vec3 positionLeg = legPositions[i];
            //    glm::quat orientationLeg = 
            //        glm::angleAxis(glm::radians(0.0f), glm::vec3(1.0, 0.5, 0.0));
            //    glm::vec3 scaleLeg{ 0.2f, 1.0f, 0.2f };
            //    leg.localTransformHandle = 
            //        world.createTransform(positionLeg, orientationLeg, scaleLeg);
            //    leg.meshName = "cube";
            //    leg.textureName = "checker_magenta";
            //    leg.shaderName = "default";
            //    leg.color = color;
            //    leg.colliderType = physics::ColliderType::CUBOID;
            //    chair.parts.push_back(leg);
            //}

            //GameObjectHandle h = world.createGameObject(chair);
        }
        // spheres
        else if (mode == 1) {
            glm::vec3 size{ 2.0f };
            float mass = (size.x * 3.0f) / 2.0f;

            GameObjectDesc sphere;
            sphere.name = "sphere";
            sphere.rootTransformHandle = 
                world.createTransform(spawnPos, orientation, size);
            sphere.mass = mass;

            SubPartDesc part;
            part.localTransformHandle = world.createTransform();
            part.colliderType = physics::ColliderType::SPHERE;
            part.meshName = "sphere";
            part.textureName = "checker_gray";
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
                float xPos = 
                    pos.x + x * (sWidth + sDistance) + y * (sWidth / 2 + sDistance);
                float yPos = 
                    pos.y + sHeight / 2 + (y * sHeight);
                float zPos =
                    pos.z + z * (sWidth + sDistance) + y * (sWidth / 2 + sDistance);

                if (randomColor) {
                    color = glm::vec3(randomRange(0, 255));
                }

                GameObjectDesc cube;
                cube.name = "cube";
                cube.rootTransformHandle = 
                    world.createTransform(
                        glm::vec3(xPos, yPos, zPos), 
                        glm::quat(1, 0, 0, 0), 
                        glm::vec3(sWidth, sHeight, sLength)
                    );
                cube.mass = sWeight;
                cube.asleep = asleep;

                SubPartDesc part;
                part.localTransformHandle = world.createTransform();
                part.textureName = "plain";
                part.color = color / 255.0f;

                cube.parts.push_back(part);
                world.createGameObject(cube);
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
                float xPos = 
                    pos.x + x * (sDiameter + sDistance) + y * (sRadius + sDistance);
                float yPos = 
                    pos.y + y * sqrt2 * sRadius;
                float zPos =
                    pos.z + z * (sDiameter + sDistance) + y * (sRadius + sDistance);

                if (randomColor) {
                    color = glm::vec3(randomRange(0, 255));
                }
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
        edgeBrickSize.x = (
                (wallWidth * brickSize.x + 
                (wallWidth - 1) * brickDistance) - 
                (wallWidth - 3 * brickSize.x + (wallWidth - 2) * brickDistance)
            ) / 2;
    }
    else {
        edgeBrickSize.z = (
                (wallWidth * brickSize.z + 
                (wallWidth - 1) * brickDistance) - 
                (wallWidth - 3 * brickSize.z + (wallWidth - 2) * brickDistance)
            ) / 2;
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
                randomColor = glm::vec3(
                    randomRange(colorRange.x, colorRange.y),
                    randomRange(colorRange.x, colorRange.y),
                    randomRange(colorRange.x, colorRange.y)
                );
            }
            else {
                float c = randomRange(colorRange.x, colorRange.y);
                randomColor = glm::vec3(c, c, c);
            }

            GameObjectDesc cube;
            cube.name = "cube";
            cube.rootTransformHandle = 
                world.createTransform(pos, glm::quat(1, 0, 0, 0), brickSize);
            cube.mass = brickWeight;
            cube.asleep = true;
            SubPartDesc part;
            part.localTransformHandle = world.createTransform();
            part.color = randomColor / 255.0f;
            cube.parts.push_back(part);
            world.createGameObject(cube);
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
            randomColor = glm::vec3(
                randomRange(colorRange.x, colorRange.y),
                randomRange(colorRange.x, colorRange.y),
                randomRange(colorRange.x, colorRange.y)
            );
        }
        else {
            float c = randomRange(colorRange.x, colorRange.y);
            randomColor = glm::vec3(c, c, c);
        }
        
        GameObjectDesc cube;
        cube.name = "cube";
        cube.rootTransformHandle = 
            world.createTransform(pos, glm::quat(1, 0, 0, 0), edgeBrickSize);
        cube.mass = brickWeight;
        cube.asleep = true;
        SubPartDesc part;
        part.localTransformHandle = world.createTransform();
        part.color = randomColor / 255.0f;
        cube.parts.push_back(part);
        world.createGameObject(cube);

        // middle bricks
        for (int row = 1; row < wallWidth - 2; row++) {
            glm::vec3 pos = startPos;
            pos.y += brickSize.y + brickSize.y / 2 + col * brickSize.y * 2;
            if (wallDirection == 0) {
                pos.x = 
                    brickSize.x / 2 + startPos.x + 
                    row * brickSize.x + 
                    brickDistance * row;
                pos.z = startPos.z;
            }
            else {
                pos.x = startPos.x;
                pos.z = 
                    brickSize.z / 2 + 
                    startPos.z + 
                    row * brickSize.z + 
                    brickDistance * row;
            }
            glm::vec3 randomColor;
            if (fullColorRange) {
                randomColor = glm::vec3(
                    randomRange(colorRange.x, colorRange.y),
                    randomRange(colorRange.x, colorRange.y),
                    randomRange(colorRange.x, colorRange.y)
                );
            }
            else {
                float c = randomRange(colorRange.x, colorRange.y);
                randomColor = glm::vec3(c, c, c);
            }
            
            GameObjectDesc cube;
            cube.name = "cube";
            cube.rootTransformHandle = 
                world.createTransform(pos, glm::quat(1, 0, 0, 0), brickSize);
            cube.mass = brickWeight;
            cube.asleep = true;
            SubPartDesc part;
            part.localTransformHandle = world.createTransform();
            part.color = randomColor / 255.0f;
            cube.parts.push_back(part);
            world.createGameObject(cube);
        }
        // edge brick
        pos = startPos;
        pos.y += brickSize.y + brickSize.y / 2 + col * brickSize.y * 2;
        if (wallDirection == 0) {
            pos.x =
                startPos.x +
                (wallWidth - 1) * (brickSize.x + brickDistance) -
                (edgeBrickSize.x - brickSize.x) / 2;
            pos.z = startPos.z;
        }
        else {
            pos.x = startPos.x;
            pos.z = 
                startPos.z + 
                (wallWidth - 1) * (brickSize.z + brickDistance) - 
                (edgeBrickSize.z - brickSize.z) / 2;
        }
        if (fullColorRange) {
            randomColor = glm::vec3(
                randomRange(colorRange.x, colorRange.y),
                randomRange(colorRange.x, colorRange.y),
                randomRange(colorRange.x, colorRange.y)
            );
        }
        else {
            float c = randomRange(colorRange.x, colorRange.y);
            randomColor = glm::vec3(c, c, c);
        }
        
        GameObjectDesc cube2;
        cube2.name = "cube";
        cube2.rootTransformHandle = 
            world.createTransform(pos, glm::quat(1, 0, 0, 0), edgeBrickSize);
        cube2.mass = brickWeight;
        cube2.asleep = true;
        SubPartDesc part2;
        part2.localTransformHandle = world.createTransform();
        part2.color = randomColor / 255.0f;
        cube2.parts.push_back(part2);
        world.createGameObject(cube2);

        brickWeight -= brickDecrease;
    }
}
